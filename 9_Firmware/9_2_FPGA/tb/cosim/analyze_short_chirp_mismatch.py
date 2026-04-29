#!/usr/bin/env python3
# ruff: noqa: T201
"""
analyze_short_chirp_mismatch.py — quantify TX-I matched-filter mismatch loss.

Background
----------
TX path (`plfm_chirp_controller.v:74,118-127`):
  60-sample inline LUT, 8-bit unsigned offset binary (DAC center = 128),
  played at fs_tx = 120 MHz over 0.5 us. Real-valued passband chirp.
  Module comment claims "30 MHz to 10 MHz" downchirp.

RX matched-filter reference (`gen_chirp_mem.py:81-101` -> `short_chirp_{i,q}.mem`):
  50-sample complex baseband, Q15, fs_rx = 100 MHz over 0.5 us.
  Generated as a 0 -> +20 MHz baseband upchirp:
      phi(t) = pi * (BW/T) * t^2,  BW = 20 MHz, T = 0.5 us
      I(n) = cos(phi),  Q(n) = sin(phi),  scaled by 0.9*Q15

These are claimed by the ledger to be ~2-3 dB mismatched. This script
derives the implied baseband chirp from the TX LUT (modeling the IF chain
and DDC by NCO at 120 MHz, decimation 4x to 100 MHz), then computes the
true matched-filter peak power lost to template mismatch by:

  1. Loading the TX LUT, computing the analytic signal (Hilbert),
     verifying instantaneous-frequency trajectory + claimed bandwidth.
  2. Modeling the DDC: mix by 120 MHz NCO at 400 MHz ADC sample rate,
     low-pass + decimate 4x to recover 100 MHz baseband. Since the TX
     LUT is only at 120 MHz, we upsample 120->400 first via zero-stuff +
     filter (the radar's analog chain does this naturally).
  3. Producing the implied 50-sample Q15 baseband reference.
  4. Computing the ambiguity peak between
        a) implied-from-TX reference cross-correlated with itself
        b) implied-from-TX reference cross-correlated with the existing
           short_chirp_{i,q}.mem
     The dB ratio of (b) peak / (a) peak is the mismatch loss.

Output: report only. Does not modify any .mem files.
"""

import os
import re
import sys

import numpy as np
from scipy.signal import hilbert, resample_poly

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
RTL_DIR  = os.path.join(THIS_DIR, "..", "..")

FS_TX = 120e6   # DAC sample rate
FS_RX = 100e6   # post-DDC processing rate
T_CHIRP = 0.5e-6
N_TX = 60       # samples in TX LUT
N_RX = 50       # samples in RX reference

# --- Parse TX LUT inline-coded in plfm_chirp_controller.v ----------------
def read_tx_lut() -> np.ndarray:
    path = os.path.join(RTL_DIR, "plfm_chirp_controller.v")
    with open(path) as f:
        src = f.read()
    # Capture every "short_chirp_lut[<idx>] = 8'd<value>;"
    pairs = re.findall(r"short_chirp_lut\[\s*(\d+)\s*\]\s*=\s*8'd\s*(\d+)\s*;", src)
    if len(pairs) != N_TX:
        sys.exit(f"expected {N_TX} TX LUT entries, got {len(pairs)}")
    arr = np.zeros(N_TX, dtype=np.int32)
    for idx_s, val_s in pairs:
        arr[int(idx_s)] = int(val_s)
    # Convert from 8-bit unsigned offset binary (DAC center = 128) to signed.
    return arr - 128  # int range roughly [-128, +127]


# --- Parse existing RX reference .mem files -------------------------------
def read_q15_mem(name: str) -> np.ndarray:
    path = os.path.join(RTL_DIR, name)
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            v = int(line, 16)
            if v >= 0x8000:
                v -= 0x10000
            out.append(v)
    return np.array(out, dtype=np.int32)


# --- Derive implied 50-sample baseband reference from the TX LUT ---------
def derive_baseband_from_tx(tx: np.ndarray) -> np.ndarray:
    """
    1) Treat tx as fs=120 MHz real samples.
    2) Compute analytic signal (Hilbert) -> single-sided spectrum copy.
    3) Find the chirp's center frequency from the analytic signal's
       mean instantaneous frequency, then mix it down to baseband by
       multiplying by exp(-j*2*pi*fc*t).
    4) Resample 120 -> 100 MHz to get exactly N_RX = 50 samples
       (matching the existing reference grid).
    5) Return as complex float64.
    """
    x = tx.astype(np.float64)
    z = hilbert(x)                          # complex analytic, fs=120 MHz
    n = np.arange(len(z))
    # Instantaneous phase + frequency
    inst_phase = np.unwrap(np.angle(z))
    inst_freq  = np.diff(inst_phase) * FS_TX / (2 * np.pi)
    fc = float(np.mean(inst_freq))          # rough center frequency in Hz
    # Mix to baseband
    bb_120 = z * np.exp(-1j * 2 * np.pi * fc * n / FS_TX)
    # Resample 120 MHz -> 100 MHz: use up=5, down=6 (5/6 = 100/120).
    bb_100 = resample_poly(bb_120, up=5, down=6)
    # Trim/pad to exactly N_RX samples
    if len(bb_100) >= N_RX:
        bb_100 = bb_100[:N_RX]
    else:
        bb_100 = np.concatenate([bb_100, np.zeros(N_RX - len(bb_100), dtype=complex)])
    return bb_100, fc, inst_freq


# --- Mismatch loss in dB --------------------------------------------------
def peak_corr_db(ref: np.ndarray, sig: np.ndarray) -> float:
    """Peak |ref dot conj(sig_shifted)| over all integer shifts, normalised."""
    # Both arrays equal length; cross-correlate.
    c = np.correlate(sig, ref, mode="full")
    return 20 * np.log10(np.max(np.abs(c)) + 1e-30)


def main() -> int:
    tx = read_tx_lut()
    rx_i = read_q15_mem("short_chirp_i.mem")
    rx_q = read_q15_mem("short_chirp_q.mem")
    if len(rx_i) != N_RX or len(rx_q) != N_RX:
        sys.exit(f"RX .mem files expected {N_RX} samples, got I={len(rx_i)} Q={len(rx_q)}")
    rx = (rx_i + 1j * rx_q).astype(complex)

    # Derive implied baseband reference from TX LUT
    bb, fc, inst_freq = derive_baseband_from_tx(tx)

    # Bandwidth check from instantaneous frequency
    f_lo, f_hi = float(np.min(inst_freq)), float(np.max(inst_freq))
    bw = f_hi - f_lo

    print("=== TX LUT analysis ===")
    print(f"  samples: {N_TX} @ {FS_TX/1e6:.0f} MHz, duration {N_TX/FS_TX*1e6:.3f} us")
    print(f"  inst-freq range:  {f_lo/1e6:+7.2f} MHz .. {f_hi/1e6:+7.2f} MHz")
    print(f"  bandwidth swept:  {bw/1e6:6.2f} MHz")
    print(f"  center frequency: {fc/1e6:+7.2f} MHz  (inferred from mean inst freq)")
    sweep_dir = "UP" if inst_freq[-1] > inst_freq[0] else "DOWN"
    print(f"  sweep direction:  {sweep_dir} (start={inst_freq[0]/1e6:+.2f} MHz, "
          f"end={inst_freq[-1]/1e6:+.2f} MHz)")

    print()
    print("=== Existing RX reference (short_chirp_{i,q}.mem) ===")
    rx_phase = np.unwrap(np.angle(rx + 1e-30))
    rx_inst_freq = np.diff(rx_phase) * FS_RX / (2 * np.pi)
    rx_lo, rx_hi = float(np.min(rx_inst_freq)), float(np.max(rx_inst_freq))
    print(f"  samples: {N_RX} @ {FS_RX/1e6:.0f} MHz")
    print(f"  inst-freq range:  {rx_lo/1e6:+7.2f} MHz .. {rx_hi/1e6:+7.2f} MHz")
    print(f"  bandwidth swept:  {(rx_hi - rx_lo)/1e6:6.2f} MHz")
    rx_sweep = "UP" if rx_inst_freq[-1] > rx_inst_freq[0] else "DOWN"
    print(f"  sweep direction:  {rx_sweep}")

    print()
    print("=== Mismatch loss (matched-filter peak: implied-vs-existing) ===")
    # Normalise both to unit energy so the only thing the ratio reflects is shape.
    bb_n = bb / np.sqrt(np.sum(np.abs(bb) ** 2) + 1e-30)
    rx_n = rx / np.sqrt(np.sum(np.abs(rx) ** 2) + 1e-30)
    auto_db = peak_corr_db(bb_n, bb_n)
    cross_db = peak_corr_db(bb_n, rx_n)
    loss_db = auto_db - cross_db
    print(f"  auto-correlation peak (implied vs implied): {auto_db:+6.2f} dB")
    print(f"  cross-corr peak (implied vs existing RX):   {cross_db:+6.2f} dB")
    print(f"  MISMATCH LOSS (matched filter): {loss_db:6.2f} dB")
    print()

    # Decision aid
    if loss_db < 0.5:
        verdict = "AGREEMENT — TX LUT and RX reference are consistent within 0.5 dB."
    elif loss_db < 2.0:
        verdict = ("MILD MISMATCH — within ledger's 2-3 dB note; refresh "
                   "recommended but not blocking.")
    else:
        verdict = "SIGNIFICANT MISMATCH — RX reference should be regenerated from TX LUT."
    print(f"VERDICT: {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
