import pyb
import machine
import micropython
import array
import math

micropython.alloc_emergency_exception_buf(100)


class EMGSampler:
    """Samples ADC channels at a fixed rate using a hardware timer.
    The ISR only stores raw 12-bit counts. The main loop reads them with pop()."""

    ADC_MAX = 4095
    VREF = 3.3

    def __init__(self, pins=("A0", "A1"), fs=1000, buf_len=256, timer_id=7):
        self.fs = fs
        self.n_ch = len(pins)
        self.size = buf_len          # usable capacity is buf_len - 1

        self._adcs = [machine.ADC(machine.Pin(p)) for p in pins]
        self._bufs = [array.array("H", [0] * buf_len) for _ in pins]

        self._head = 0               # written only by the ISR
        self._tail = 0               # written only by the main loop
        self.overruns = 0

        self._timer_id = timer_id
        self._timer = None
        self._cb = self._isr         # bind once, so the ISR does not allocate

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

    def read(self, ch):
        """Value of channel ch at the current tail (call before advance())."""
        return self._bufs[ch][self._tail]

    def advance(self):
        nxt = self._tail + 1
        if nxt == self.size:
            nxt = 0
        self._tail = nxt

    def pop(self):
        """Convenience: returns a tuple of all channels (allocates a tuple)."""
        t = self._tail
        sample = tuple(self._bufs[i][t] for i in range(self.n_ch))
        self.advance()
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
    """Raw ADC counts -> offset-free, high-passed signal. One instance per channel.

    The Olimex shield already band-limits to ~0.16-40 Hz (3rd-order LPF at 40 Hz),
    so the useful EMG band left is roughly hp_fc..40 Hz. hp_fc only needs to remove
    motion artefacts and baseline wander; don't push it much above ~10 Hz."""

    CLIP_LO = 20      # counts; raw values outside [CLIP_LO, CLIP_HI] mean the
    CLIP_HI = 4075    # analog chain is saturating -> turn the gain trimmer (TR1) down

    def __init__(self, fs=500, hp_fc=10.0):
        self.fs = fs
        self.offset = 2048.0   # mid-scale until estimate_offsets() runs
        self.hp = Biquad.highpass(fs, hp_fc)
        self.clips = 0

    def reset(self):
        self.hp.reset()

    def process(self, raw):
        if raw < self.CLIP_LO or raw > self.CLIP_HI:
            self.clips += 1
        return self.hp.process(raw - self.offset)


def estimate_offsets(sampler, procs, discard_ms=2000, measure_ms=1000):
    """Estimate the DC level (the shield's V_REF) of every channel at once.
    EMG is zero-mean, so this works whether or not the muscle is active:
    no user action needed. The discard period lets the shield's 0.16 Hz
    high-pass filters settle after power-up / electrode connection.
    Call right after sampler.start(). procs[i] belongs to channel i."""
    fs = sampler.fs
    n_discard = fs * discard_ms // 1000
    n_meas = fs * measure_ms // 1000
    totals = [0] * sampler.n_ch
    got = 0
    while got < n_discard + n_meas:
        while sampler.available() and got < n_discard + n_meas:
            got += 1
            if got > n_discard:
                for i in range(sampler.n_ch):
                    totals[i] += sampler.read(i)
            sampler.advance()
    for i, p in enumerate(procs):
        p.offset = totals[i] / n_meas
        p.reset()


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
    """Envelope (ADC counts) -> 0..1 command with deadband and sub-maximal full scale."""

    def __init__(self, frac=0.5, k=3.0, min_db=0.1, min_span=1.0):
        self.frac = frac          # fraction of (MVC - rest) that counts as full scale
        self.k = k                # deadband = k * rest std ...
        self.min_db = min_db      # ... or this fraction of the span, whichever is larger
        self.min_span = min_span  # minimum (high - low) in counts, avoids divide-by-~0
        self.low = 0.0
        self.high = 1.0
        self.valid = False

    def set_levels(self, rest_mean, rest_std, mvc_mean):
        span = mvc_mean - rest_mean
        self.low = rest_mean + max(self.k * rest_std, self.min_db * span)
        self.high = rest_mean + self.frac * span
        # Calibration is only usable if the contraction clearly exceeds the deadband
        self.valid = self.high > self.low + self.min_span
        if not self.valid:
            self.high = self.low + self.min_span

    def process(self, e):
        u = (e - self.low) / (self.high - self.low)
        if u < 0.0:
            return 0.0
        if u > 1.0:
            return 1.0
        return u


class SlewLimiter:
    """Limits how fast the 0..1 command may change (rate in units per second)."""

    def __init__(self, fs=1000, rise=2.0, fall=4.0):
        self.up = rise / fs
        self.down = fall / fs
        self.y = 0.0

    def process(self, u):
        d = u - self.y
        if d > self.up:
            d = self.up
        elif d < -self.down:
            d = -self.down
        self.y += d
        return self.y

    def reset(self, value=0.0):
        self.y = value
