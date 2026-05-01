#!/usr/bin/env python3
"""
gen_realdata_hex.py — Synthetic stimulus + bit-exact golden for the realdata
co-simulation testbenches (tb_doppler_realdata, tb_fullchain_realdata).

Replaces the legacy ADI CN0566 hardware captures (32-chirp / 2-subframe /
32-bin Doppler) with a synthetic radar scene at production dimensions
(48-chirp / 3-subframe / 48-bin Doppler) so the regression no longer
depends on out-of-tree .npy files.

Outputs (six files, all under tb/cosim/real_data/hex/):
  doppler_input_realdata.hex      48 chirps x 512 range bins, packed {Q,I}
  doppler_ref_i.hex / _q.hex      512 range bins x 48 Doppler bins (signed 16-bit)
  fullchain_range_input.hex       48 chirps x 2048 range bins, packed {Q,I}
  fullchain_doppler_ref_i.hex
  fullchain_doppler_ref_q.hex     same shape as doppler_ref_*

Dimensions match production (radar_params.vh: RP_FFT_SIZE=2048,
RP_DECIMATION_FACTOR=4, RP_NUM_RANGE_BINS=512, RP_NUM_DOPPLER_BINS=48).

Pipeline modeled (bit-exact to RTL):
  doppler-only:  scene -> doppler_processor (3-subframe, 16-pt FFT, Hamming)
  fullchain:     scene -> range_bin_decimator (2048->512 peak, DECIM=4)
                       -> doppler_processor

Usage:  python3 gen_realdata_hex.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fpga_model import DopplerProcessor, RangeBinDecimator
from radar_scene import Target, generate_doppler_frame


# ----------------------------------------------------------------------------
# Production dimensions (radar_params.vh: PR-F + RP_FFT_SIZE/RP_DECIMATION_FACTOR)
# ----------------------------------------------------------------------------
NUM_SUBFRAMES        = 3
DOPPLER_FFT_SIZE     = 16
DOPPLER_TOTAL_BINS   = NUM_SUBFRAMES * DOPPLER_FFT_SIZE   # 48
CHIRPS_PER_SUBFRAME  = 16
CHIRPS_PER_FRAME     = NUM_SUBFRAMES * CHIRPS_PER_SUBFRAME  # 48

# Doppler-only TB: post-decim range bins fed straight into doppler.
# Matches production RP_NUM_RANGE_BINS = RP_FFT_SIZE / RP_DECIMATION_FACTOR.
DOPPLER_RANGE_BINS   = 512

# Fullchain TB: pre-decim 2048-bin range FFT -> range_bin_decimator (DECIM=4) -> doppler.
FULLCHAIN_INPUT_BINS    = 2048
FULLCHAIN_OUTPUT_BINS   = DOPPLER_RANGE_BINS              # 512
FULLCHAIN_DECIM_FACTOR  = FULLCHAIN_INPUT_BINS // FULLCHAIN_OUTPUT_BINS   # 4

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hex")

# ----------------------------------------------------------------------------
# Scene — two distinct targets so both the FFT bin layout and the slow-time
# Doppler axis carry information across all three sub-frames.
# ----------------------------------------------------------------------------
SCENE_TARGETS = [
    Target(range_m=300,  velocity_mps= 10.0, rcs_dbsm=20.0),
    Target(range_m=800,  velocity_mps=-20.0, rcs_dbsm=15.0),
]
SCENE_SEED = 42


def write_hex_32(path, samples_iq):
    """Packed 32-bit {Q[31:16], I[15:0]} per line for $readmemh."""
    with open(path, 'w') as f:
        for (i_val, q_val) in samples_iq:
            packed = ((q_val & 0xFFFF) << 16) | (i_val & 0xFFFF)
            f.write(f"{packed:08X}\n")


def write_hex_16(path, values):
    """One signed 16-bit value per line, two's-complement hex."""
    with open(path, 'w') as f:
        for v in values:
            f.write(f"{v & 0xFFFF:04X}\n")


def make_doppler_processor():
    """DopplerProcessor at production dimensions (3 sub-frames, 512 range bins)."""
    return DopplerProcessor()                        # defaults: NUM_SUBFRAMES=3, RANGE_BINS=512


def flatten_doppler_map(doppler_i, doppler_q):
    """RTL stream order: rbin 0 [dbin 0..47], rbin 1 [...], ..., rbin 63 [...]."""
    flat_i, flat_q = [], []
    for rb in range(DOPPLER_RANGE_BINS):
        for db in range(DOPPLER_TOTAL_BINS):
            flat_i.append(doppler_i[rb][db])
            flat_q.append(doppler_q[rb][db])
    return flat_i, flat_q


def gen_doppler_realdata():
    """tb_doppler_realdata: post-decim 48 x 512 -> doppler 512 x 48."""
    print("[doppler_realdata] generating ...")
    frame_i, frame_q = generate_doppler_frame(
        SCENE_TARGETS,
        n_chirps=CHIRPS_PER_FRAME,
        n_range_bins=DOPPLER_RANGE_BINS,
        seed=SCENE_SEED,
    )

    stim = []
    for c in range(CHIRPS_PER_FRAME):
        for rb in range(DOPPLER_RANGE_BINS):
            stim.append((frame_i[c][rb], frame_q[c][rb]))
    write_hex_32(os.path.join(OUT_DIR, "doppler_input_realdata.hex"), stim)

    dp = make_doppler_processor()
    doppler_i, doppler_q = dp.process_frame(frame_i, frame_q)
    flat_i, flat_q = flatten_doppler_map(doppler_i, doppler_q)
    write_hex_16(os.path.join(OUT_DIR, "doppler_ref_i.hex"), flat_i)
    write_hex_16(os.path.join(OUT_DIR, "doppler_ref_q.hex"), flat_q)

    print(f"  stimulus: {len(stim)} packed lines (expected {CHIRPS_PER_FRAME * DOPPLER_RANGE_BINS})")
    print(f"  golden:   {len(flat_i)} lines i / {len(flat_q)} lines q "
          f"(expected {DOPPLER_RANGE_BINS * DOPPLER_TOTAL_BINS})")


def gen_fullchain_realdata():
    """tb_fullchain_realdata: 48 x 2048 -> RangeBinDecimator (DECIM=4 peak) -> doppler 512 x 48."""
    print("[fullchain_realdata] generating ...")
    frame_i, frame_q = generate_doppler_frame(
        SCENE_TARGETS,
        n_chirps=CHIRPS_PER_FRAME,
        n_range_bins=FULLCHAIN_INPUT_BINS,
        seed=SCENE_SEED,
    )

    stim = []
    for c in range(CHIRPS_PER_FRAME):
        for rb in range(FULLCHAIN_INPUT_BINS):
            stim.append((frame_i[c][rb], frame_q[c][rb]))
    write_hex_32(os.path.join(OUT_DIR, "fullchain_range_input.hex"), stim)

    # fpga_model.RangeBinDecimator is hard-coded to 2048->512, DECIM=4 — production.
    decim_i_2d, decim_q_2d = [], []
    for c in range(CHIRPS_PER_FRAME):
        di, dq = RangeBinDecimator.decimate(frame_i[c], frame_q[c], mode=1, start_bin=0)
        decim_i_2d.append(di)
        decim_q_2d.append(dq)

    dp = make_doppler_processor()
    doppler_i, doppler_q = dp.process_frame(decim_i_2d, decim_q_2d)
    flat_i, flat_q = flatten_doppler_map(doppler_i, doppler_q)
    write_hex_16(os.path.join(OUT_DIR, "fullchain_doppler_ref_i.hex"), flat_i)
    write_hex_16(os.path.join(OUT_DIR, "fullchain_doppler_ref_q.hex"), flat_q)

    print(f"  stimulus: {len(stim)} packed lines "
          f"(expected {CHIRPS_PER_FRAME * FULLCHAIN_INPUT_BINS})")
    print(f"  golden:   {len(flat_i)} lines i / {len(flat_q)} lines q "
          f"(expected {DOPPLER_RANGE_BINS * DOPPLER_TOTAL_BINS})")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    gen_doppler_realdata()
    gen_fullchain_realdata()

    print("\nGenerated files:")
    for f in (
        "doppler_input_realdata.hex",
        "doppler_ref_i.hex",
        "doppler_ref_q.hex",
        "fullchain_range_input.hex",
        "fullchain_doppler_ref_i.hex",
        "fullchain_doppler_ref_q.hex",
    ):
        path = os.path.join(OUT_DIR, f)
        with open(path) as fp:
            n_lines = sum(1 for _ in fp)
        print(f"  {f:40s}  {n_lines:7d} lines  ({os.path.getsize(path):7d} bytes)")


if __name__ == '__main__':
    main()
