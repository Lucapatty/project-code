import pyb
import machine
import micropython
import array
import math
import time

micropython.alloc_emergency_exception_buf(100)


class EMGSampler:
    """Samples ADC channels at a fixed rate using a hardware timer.
    The ISR only stores raw 12-bit counts. The main loop reads them with pop()."""

    ADC_MAX = 4095
    VREF = 3.3

    def __init__(self, pins=("A0", "A1"), fs=1000, buf_len=256, timer_id=7):
        self.fs = fs
        self.n_ch = len(pins)
        self.size = buf_len

        self._adcs = [machine.ADC(machine.Pin(p)) for p in pins]
        self._bufs = [array.array("H", [0] * buf_len) for _ in pins]

        self._head = 0
        self._tail = 0
        self.overruns = 0

        self._timer_id = timer_id
        self._timer = None
        self._cb = self._isr

    def _isr(self, _t):
        head = self._head
        nxt = head + 1
        if nxt == self.size:
            nxt = 0
        if nxt == self._tail:
            self.overruns += 1
            return
        for i in range(self.n_ch):
            self._bufs[i][head] = self._adcs[i].read_u16() >> 4
        self._head = nxt

    def start(self):
        self._head = 0
        self._tail = 0
        self.overruns = 0
        self._timer = pyb.Timer(self._timer_id, freq=self.fs)
        self._timer.callback(self._cb)

    def stop(self):
        if self._timer is not None:
            self._timer.callback(None)
            self._timer.deinit()
            self._timer = None

    def available(self):
        n = self._head - self._tail
        if n < 0:
            n += self.size
        return n

    def pop(self):
        t = self._tail
        sample = tuple(self._bufs[i][t] for i in range(self.n_ch))
        nxt = t + 1
        if nxt == self.size:
            nxt = 0
        self._tail = nxt
        return sample

    def counts_to_volts(self, counts):
        return counts * self.VREF / self.ADC_MAX




class Biquad:
    """One second-order IIR section (Direct Form II Transposed)."""

    def __init__(self, b0, b1, b2, a1, a2):
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.a1 = a1
        self.a2 = a2
        self.z1 = 0.0
        self.z2 = 0.0

    def process(self, x):
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0

    @classmethod
    def highpass(cls, fs, fc, q=0.70710678):
        w0 = 2.0 * math.pi * fc / fs
        c = math.cos(w0)
        alpha = math.sin(w0) / (2.0 * q)
        a0 = 1.0 + alpha
        b0 = (1.0 + c) / 2.0 / a0
        b1 = -(1.0 + c) / a0
        b2 = b0
        a1 = -2.0 * c / a0
        a2 = (1.0 - alpha) / a0
        return cls(b0, b1, b2, a1, a2)


class EMGProcessor:
    """Raw ADC counts -> offset-free, high-passed signal."""

    def __init__(self, fs=1000, hp_fc=20.0):
        self.fs = fs
        self.offset = 0.0
        self.hp = Biquad.highpass(fs, hp_fc)

    def calibrate_offset(self, sampler, discard_ms=500, measure_ms=1500):
        """Call right after sampler.start() with the arm relaxed."""
        n_discard = self.fs * discard_ms // 1000
        n_meas = self.fs * measure_ms // 1000
        got = 0
        total = 0
        while got < n_discard + n_meas:
            while sampler.available():
                s = sampler.pop()[0]
                got += 1
                if got > n_discard:
                    total += s
        self.offset = total / n_meas
        self.hp.reset()

    def process(self, raw):
        x = raw - self.offset
        return self.hp.process(x)

class Envelope:
    """Full-wave rectify + first-order low-pass (exponential moving average)."""

    def __init__(self, fs=1000, fc=4.0):
        self.alpha = 1.0 - math.exp(-2.0 * math.pi * fc / fs)
        self.y = 0.0

    def process(self, x):
        self.y += self.alpha * (abs(x) - self.y)
        return self.y

    def reset(self, value=0.0):
        self.y = value

class Normalizer:
    """Envelope -> 0..1 command with deadband and sub-maximal full scale."""

    def __init__(self, frac=0.5, k=3.0, min_db=0.1):
        self.frac = frac      # fraction of (MVC - rest) that counts as full scale
        self.k = k            # deadband = k * rest std ...
        self.min_db = min_db  # ... or this fraction of the span, whichever is larger
        self.low = 0.0
        self.high = 1.0

    def set_levels(self, rest_mean, rest_std, mvc_mean):
        span = mvc_mean - rest_mean
        self.low = rest_mean + max(self.k * rest_std, self.min_db * span)
        self.high = rest_mean + self.frac * span
        if self.high < self.low + 1.0:
            self.high = self.low + 1.0

    def process(self, e):
        u = (e - self.low) / (self.high - self.low)
        if u < 0.0:
            return 0.0
        if u > 1.0:
            return 1.0
        return u


class Calibrator:
    """Laptop-guided calibration for N channels (prompts via print/input).
    Order: relax both -> flex ch0 -> flex ch1."""

    def __init__(self, fs=1000, names=("LEFT bicep (A0)", "RIGHT bicep (A1)")):
        self.fs = fs
        self.names = names
        self.n_ch = len(names)
        self.rest = [(0.0, 0.0)] * self.n_ch   # (mean, std) per channel
        self.mvc = [0.0] * self.n_ch

    @staticmethod
    def reset_chain(procs, envs):
        for p in procs:
            p.hp.reset()
        for e in envs:
            e.reset()

    def _raw_means(self, sampler, discard_ms, measure_ms):
        """Mean raw counts per channel (used for the DC offset)."""
        n_discard = self.fs * discard_ms // 1000
        n_meas = self.fs * measure_ms // 1000
        got = 0
        used = 0
        tot = [0] * self.n_ch
        while got < n_discard + n_meas:
            while sampler.available() and got < n_discard + n_meas:
                s = sampler.pop()
                got += 1
                if got > n_discard:
                    used += 1
                    for i in range(self.n_ch):
                        tot[i] += s[i]
        return [t / used for t in tot]

    def _collect(self, sampler, procs, envs, ms, skip_ms):
        """Run the chain for ms; return [(mean, std)] of each envelope after skip_ms."""
        n_total = self.fs * ms // 1000
        n_skip = self.fs * skip_ms // 1000
        n = 0
        used = 0
        s1 = [0.0] * self.n_ch
        s2 = [0.0] * self.n_ch
        while n < n_total:
            while sampler.available() and n < n_total:
                s = sampler.pop()
                n += 1
                for i in range(self.n_ch):
                    e = envs[i].process(procs[i].process(s[i]))
                    if n > n_skip:
                        s1[i] += e
                        s2[i] += e * e
                if n > n_skip:
                    used += 1
        out = []
        for i in range(self.n_ch):
            if used == 0:
                out.append((0.0, 0.0))
                continue
            m = s1[i] / used
            v = s2[i] / used - m * m
            out.append((m, math.sqrt(v) if v > 0.0 else 0.0))
        return out

    def run(self, sampler, procs, envs, norms):
        """Returns a list of booleans (one per channel): True = calibration valid."""
        # ---- 1. rest: offsets + rest level of all channels ----
        input("\n[1/%d] Relax BOTH muscles completely. Press Enter to start..." % (self.n_ch + 1))
        print("Measuring rest (about 4 s), stay relaxed...")
        sampler.start()
        offsets = self._raw_means(sampler, 1000, 1500)
        for i in range(self.n_ch):
            procs[i].offset = offsets[i]
        self.reset_chain(procs, envs)
        self.rest = self._collect(sampler, procs, envs, 3000, 1000)
        sampler.stop()
        for i in range(self.n_ch):
            print("  ch%d offset=%.1f  rest mean=%.2f std=%.2f"
                  % (i, offsets[i], self.rest[i][0], self.rest[i][1]))

        # ---- 2. flex each muscle in turn ----
        for ch in range(self.n_ch):
            input("\n[%d/%d] Press Enter, then IMMEDIATELY flex the %s hard and hold (5 s)..."
                  % (ch + 2, self.n_ch + 1, self.names[ch]))
            print(">>> FLEX NOW! <<<")
            self.reset_chain(procs, envs)
            sampler.start()
            stats = self._collect(sampler, procs, envs, 5000, 1500)
            sampler.stop()
            print(">>> relax <<<")
            self.mvc[ch] = stats[ch][0]
            print("  ch%d mvc mean=%.2f" % (ch, self.mvc[ch]))
            for other in range(self.n_ch):
                if other != ch:
                    print("  crosstalk: ch%d read %.2f while ch%d flexed (rest was %.2f)"
                          % (other, stats[other][0], ch, self.rest[other][0]))

        # ---- 3. validate and configure normalizers ----
        ok = []
        for i in range(self.n_ch):
            rm, rs = self.rest[i]
            valid = self.mvc[i] > rm + 10.0 * rs and self.mvc[i] > 2.0 * rm
            if valid:
                norms[i].set_levels(rm, rs, self.mvc[i])
                print("ch%d OK:     deadband low=%.2f  full scale high=%.2f"
                      % (i, norms[i].low, norms[i].high))
            else:
                print("ch%d FAILED: no clear flex detected" % i)
            ok.append(valid)
        return ok