#!/usr/bin/env python3
# probe_fed_array_aeris10_v3.py
#
# 4x4 (default; configurable) probe-fed patch array sim for AERIS-10. Built
# on the same single-element design point as probe_fed_aeris10_v3.py but
# placed on the 8x16 Gerber pitch (14.27 mm X / 15.01 mm Y).
#
# Purpose: characterise mutual coupling between elements. Each patch has its
# own probe-via port; only one port is excited per sim run, the other 15 are
# terminated in 50 Ω. From this we read:
#   - S_dd (active S11 of the driven element with array loaded)
#   - S_jd  for all other ports j (coupling driven → j)
# Pattern of |S_jd| dB values across the array tells us nearest-neighbour vs
# diagonal vs skip-one coupling, edge vs interior asymmetry, etc.
#
# Per-element design (matches probe_fed_aeris10_v3.py iter#3):
#   PATCH_W = 7.854 mm   PATCH_L = 6.56 mm   FEED_OFFSET = 2.14 mm
# Substrate: 0.508 mm RO4350B (εr=3.48, tanδ=0.0037)
# Pitch: 14.27 mm × 15.01 mm  (X-pitch ~λ₀/2 at 10.5 GHz, Y-pitch ~1.05·λ₀/2)
#
# Run:
#   cd /tmp && DYLD_LIBRARY_PATH=/Users/ganeshpanth/opt/openEMS/lib \
#     PROFILE=sanity \
#     /Users/ganeshpanth/radar_venv/bin/python \
#     /Users/ganeshpanth/PLFM_RADAR/5_Simulations/Antenna/probe_fed_array_aeris10_v3.py
#
# Env overrides:
#   ARRAY_NX  ARRAY_NY  (default 4, 4)
#   PITCH_X_MM  PITCH_Y_MM   (default 14.27, 15.01 from Gerber)
#   DRIVEN_X  DRIVEN_Y       (0-indexed; default = inner element 1,1)
#   PATCH_W_MM  PATCH_L_MM  FEED_OFFSET_MM  (default v3 design point)
#
# Output (in /tmp/aeris10_array_v3/):
#   S_matrix.csv  — driven-column S parameters (mag dB + phase deg) at 10.5 GHz
#   S11_data.csv  — driven port full sweep
#   coupling_grid.png  — heatmap of |S_jd| dB at 10.5 GHz across array

import os
import sys
import time
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from openEMS import openEMS
from openEMS.physical_constants import C0
from CSXCAD import ContinuousStructure
from CSXCAD.SmoothMeshLines import SmoothMeshLines

# ============================================================================
# PROFILES
# ============================================================================
PROFILE = os.environ.get("PROFILE", "sanity")
profiles = {
    "sanity":   {"mesh_lambda_div": 18, "n_timesteps": 50000, "end_dB": -30},
    "balanced": {"mesh_lambda_div": 25, "n_timesteps": 80000, "end_dB": -40},
}
cfg = profiles[PROFILE]

# ============================================================================
# BAND
# ============================================================================
F0      = 10.5e9
F_SPAN  = 4.0e9
F_START = F0 - F_SPAN/2
F_STOP  = F0 + F_SPAN/2

# ============================================================================
# STACKUP
# ============================================================================
T_CU         = 0.035
H_PATCH_SUB  = 0.508
EPS_RO4350B  = 3.48
TAN_RO4350B  = 0.0037

Z_GND   = 0.0
Z_PATCH = Z_GND + T_CU + H_PATCH_SUB
Z_TOP   = Z_PATCH + T_CU

# ============================================================================
# PATCH (per-element, from v3 iter#3)
# ============================================================================
PATCH_W = float(os.environ.get("PATCH_W_MM", "7.854"))
PATCH_L = float(os.environ.get("PATCH_L_MM", "6.56"))
FEED_OFFSET_MM = float(os.environ.get("FEED_OFFSET_MM", "2.14"))

# ============================================================================
# ARRAY
# ============================================================================
N_X = int(os.environ.get("ARRAY_NX", "4"))
N_Y = int(os.environ.get("ARRAY_NY", "4"))
PITCH_X = float(os.environ.get("PITCH_X_MM", "14.27"))
PITCH_Y = float(os.environ.get("PITCH_Y_MM", "15.01"))
DRIVEN_X = int(os.environ.get("DRIVEN_X", str(N_X // 2 - (N_X+1) % 2)))   # inner element
DRIVEN_Y = int(os.environ.get("DRIVEN_Y", str(N_Y // 2 - (N_Y+1) % 2)))

# Array footprint extent (centre patch on origin)
ARRAY_X_HALF = (N_X-1)/2 * PITCH_X + PATCH_W/2
ARRAY_Y_HALF = (N_Y-1)/2 * PITCH_Y + PATCH_L/2

# Substrate / ground extents (~λ/2 margin around array)
GND_MARGIN = 14.3
GND_X_HALF = ARRAY_X_HALF + GND_MARGIN
GND_Y_HALF = ARRAY_Y_HALF + GND_MARGIN

# Air box
AIR_ABOVE = 14.3
AIR_BELOW = 14.3
AIR_X_HALF = GND_X_HALF + 8.0
AIR_Y_HALF = GND_Y_HALF + 8.0

OUT_DIR = "/tmp/aeris10_array_v3"
os.makedirs(OUT_DIR, exist_ok=True)


def patch_center(i, j):
    """Centre coordinate of patch at array index (i,j), origin at array centre."""
    x = -(N_X-1)/2 * PITCH_X + i * PITCH_X
    y = -(N_Y-1)/2 * PITCH_Y + j * PITCH_Y
    return x, y


# ============================================================================
# Build + run
# ============================================================================
def run_case(sim_path, profile_cfg):
    fdtd = openEMS(NrTS=profile_cfg["n_timesteps"],
                   EndCriteria=10**(profile_cfg["end_dB"]/20.0))
    fdtd.SetGaussExcite(F0, F_SPAN/2.0)
    fdtd.SetBoundaryCond(["MUR"]*6)

    CSX = ContinuousStructure()
    fdtd.SetCSX(CSX)
    mesh = CSX.GetGrid()
    mesh.SetDeltaUnit(1e-3)

    # ---- materials ----
    eps0 = 8.854e-12
    patch_sub = CSX.AddMaterial("RO4350B",
        epsilon=EPS_RO4350B,
        kappa=2*np.pi*F0*EPS_RO4350B*eps0*TAN_RO4350B)
    copper = CSX.AddMetal("Copper")

    # ---- substrate (full board extent) ----
    patch_sub.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND + T_CU],
                      [+GND_X_HALF, +GND_Y_HALF, Z_PATCH], priority=1)

    # ---- L2: full ground plane ----
    copper.AddBox([-GND_X_HALF, -GND_Y_HALF, Z_GND],
                  [+GND_X_HALF, +GND_Y_HALF, Z_GND + T_CU], priority=10)

    # ---- L1: 4x4 patch array + ports ----
    ports = []
    feed_locs = []
    for i in range(N_X):
        for j in range(N_Y):
            cx, cy = patch_center(i, j)
            copper.AddBox([cx - PATCH_W/2, cy - PATCH_L/2, Z_PATCH],
                          [cx + PATCH_W/2, cy + PATCH_L/2, Z_PATCH + T_CU],
                          priority=10)
            feed_x = cx
            feed_y = cy - PATCH_L/2 + FEED_OFFSET_MM
            feed_locs.append((feed_x, feed_y, i, j))

    # ---- mesh ----
    lambda_min_mm = (C0 / F_STOP) * 1000.0
    res = lambda_min_mm / profile_cfg["mesh_lambda_div"]

    # X mesh: array extent + air, plus patch edges + feed locations
    xlines = [-AIR_X_HALF, -GND_X_HALF, +GND_X_HALF, +AIR_X_HALF]
    for i in range(N_X):
        cx, _ = patch_center(i, 0)
        xlines += [cx - PATCH_W/2, cx, cx + PATCH_W/2]
    # Y mesh
    ylines = [-AIR_Y_HALF, -GND_Y_HALF, +GND_Y_HALF, +AIR_Y_HALF]
    for j in range(N_Y):
        _, cy = patch_center(0, j)
        ylines += [cy - PATCH_L/2, cy, cy + PATCH_L/2,
                   cy - PATCH_L/2 + FEED_OFFSET_MM]   # feed y location

    # Z mesh: 6 cells in substrate
    air_below = list(np.arange(Z_GND - T_CU - AIR_BELOW, Z_GND - T_CU, res))
    air_above = list(np.arange(Z_TOP + res, Z_TOP + AIR_ABOVE + res, res))
    sub_interior = list(np.linspace(Z_GND + T_CU, Z_PATCH, 7)[1:-1])
    zlines = sorted(set(air_below + [
        Z_GND - T_CU, Z_GND, Z_GND + T_CU,
        Z_PATCH, Z_PATCH + T_CU,
    ] + sub_interior + air_above))

    xlines = SmoothMeshLines(np.array(xlines), res)
    ylines = SmoothMeshLines(np.array(ylines), res)
    zlines = np.array(zlines)
    mesh.AddLine("x", xlines)
    mesh.AddLine("y", ylines)
    mesh.AddLine("z", zlines)
    n_cells = len(xlines) * len(ylines) * len(zlines)

    # ---- ports (one excited, 15 terminated 50Ω) ----
    # NOTE: each port box must land exactly on existing mesh lines. The seed
    # mesh includes feed_x (= patch centre cx) and feed_y for each (i,j) — so
    # this normally works — but SmoothMeshLines can sub-cell-shift seed lines
    # in some configurations and a port box ends up between two mesh lines,
    # leaving openEMS without an excitation cell ("Unused primitive" warning,
    # zero energy). The DRIVEN=(1,1) inner-element case has been verified to
    # land on the mesh; other driven-port choices are best-effort. If you see
    # NaN/zero results for a different DRIVEN_X/DRIVEN_Y, that's the cause.
    for (feed_x, feed_y, i, j) in feed_locs:
        port_num = i * N_Y + j + 1
        excite_amp = 1.0 if (i == DRIVEN_X and j == DRIVEN_Y) else 0.0
        port = fdtd.AddLumpedPort(port_num, 50,
                                   [feed_x, feed_y, Z_GND + T_CU],
                                   [feed_x, feed_y, Z_PATCH],
                                   'z', excite=excite_amp, priority=5)
        ports.append(((i, j), port))

    # ---- run ----
    print(f"[case] {N_X}x{N_Y} array, driven=({DRIVEN_X},{DRIVEN_Y}), "
          f"cells={n_cells:,}, sub={H_PATCH_SUB}mm, pitch={PITCH_X}x{PITCH_Y}mm")
    t0 = time.time()
    fdtd.Run(sim_path, verbose=0, cleanup=True)
    dt = time.time() - t0

    # ---- post-process ----
    freq = np.linspace(F_START, F_STOP, 401)
    driven_port = next(p for (idx, p) in ports if idx == (DRIVEN_X, DRIVEN_Y))
    for (idx, p) in ports:
        p.CalcPort(sim_path, freq)

    # S_jd (j is each port, d is driven). For driven port, S_dd = uf_ref/uf_inc.
    # For other ports, S_jd = uf_ref_j / uf_inc_d (no incident wave at j from
    # its own source since port j is unexcited).
    S = {}
    for (idx, p) in ports:
        if idx == (DRIVEN_X, DRIVEN_Y):
            S[idx] = p.uf_ref / p.uf_inc
        else:
            S[idx] = p.uf_ref / driven_port.uf_inc

    return freq, S, dt, ports


# ============================================================================
# MAIN
# ============================================================================
sim_path = os.path.join(OUT_DIR, "single")
freq, S, dt, ports = run_case(sim_path, cfg)

# At 10.5 GHz
i_op = int(np.argmin(np.abs(freq - F0)))

# Print coupling grid
print()
print("=" * 70)
print(f"  4x4 probe-fed array — driven port at ({DRIVEN_X},{DRIVEN_Y})")
print(f"  Substrate: {H_PATCH_SUB} mm RO4350B, pitch {PITCH_X}x{PITCH_Y} mm")
print(f"  Sim time: {dt:.1f} s")
print("=" * 70)
print()
print(f"  S parameters at {F0/1e9:.2f} GHz (|S_j,driven| in dB):")
print()
# Layout grid as visual array (i is x-direction, j is y-direction)
# Print y high to low so it matches usual visual orientation
header = "      " + "".join(f"  i={i:1d}  " for i in range(N_X))
print(header)
for j in reversed(range(N_Y)):
    row = f"  j={j:1d}: "
    for i in range(N_X):
        val = abs(S[(i, j)][i_op])
        dB = 20*np.log10(val + 1e-30)
        row += f"{dB:>7.1f}"
    print(row)
print()
# Driven port S11 vs frequency
S_dd = S[(DRIVEN_X, DRIVEN_Y)]
S_dd_dB = 20*np.log10(np.abs(S_dd) + 1e-30)
zin_d = 50.0 * (1 + S_dd) / (1 - S_dd)   # Z = Z0·(1+S)/(1-S)
print(f"  Driven port active S11:")
print(f"    @ 10.5 GHz : {S_dd_dB[i_op]:.2f} dB    Z = {zin_d[i_op].real:.1f} + j{zin_d[i_op].imag:.1f} Ω")
# -10 dB BW around f0
below = S_dd_dB <= -10.0
if below[i_op]:
    lo, hi = i_op, i_op
    while lo > 0 and below[lo-1]:
        lo -= 1
    while hi < len(below)-1 and below[hi+1]:
        hi += 1
    print(f"    -10 dB BW : {(freq[hi]-freq[lo])/1e6:.0f} MHz "
          f"({freq[lo]/1e9:.2f} – {freq[hi]/1e9:.2f} GHz)")
else:
    print(f"    -10 dB BW : <none at 10.5 GHz>")

# Worst-case coupling (excluding driven port itself)
couplings = [(idx, abs(S[idx][i_op])) for idx in S.keys() if idx != (DRIVEN_X, DRIVEN_Y)]
couplings.sort(key=lambda x: -x[1])
print()
print(f"  Top-5 strongest couplings to driven port at 10.5 GHz:")
for idx, val in couplings[:5]:
    di = idx[0] - DRIVEN_X
    dj = idx[1] - DRIVEN_Y
    dB = 20*np.log10(val + 1e-30)
    print(f"    ({idx[0]},{idx[1]})  Δ=({di:+d},{dj:+d})  |S| = {dB:>6.1f} dB")
print("=" * 70)

# Save S matrix CSV (full-band)
with open(os.path.join(OUT_DIR, "S_matrix.csv"), "w", newline="") as f:
    w = csv.writer(f)
    header = ["freq_Hz"]
    keys = sorted(S.keys())
    for idx in keys:
        header += [f"S({idx[0]},{idx[1]})_dB", f"S({idx[0]},{idx[1]})_phase_deg"]
    w.writerow(header)
    for k in range(len(freq)):
        row = [freq[k]]
        for idx in keys:
            mag_dB = 20*np.log10(np.abs(S[idx][k]) + 1e-30)
            phase = np.angle(S[idx][k], deg=True)
            row += [mag_dB, phase]
        w.writerow(row)

# Save driven-port S11 CSV
with open(os.path.join(OUT_DIR, "S11_data.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["freq_Hz", "S11_dB", "Zin_real", "Zin_imag"])
    for k in range(len(freq)):
        w.writerow([freq[k], S_dd_dB[k], zin_d[k].real, zin_d[k].imag])

# Coupling heatmap at 10.5 GHz
fig, ax = plt.subplots(figsize=(6.5, 6))
grid = np.zeros((N_Y, N_X))
for (i, j) in S.keys():
    grid[j, i] = 20*np.log10(abs(S[(i,j)][i_op]) + 1e-30)
# Driven port floor (S11 is just one number, not a coupling) — set to NaN to highlight
grid[DRIVEN_Y, DRIVEN_X] = np.nan
im = ax.imshow(grid, origin='lower', cmap='viridis', aspect='equal')
ax.set_xticks(range(N_X))
ax.set_yticks(range(N_Y))
ax.set_xlabel('i (x-pitch direction)')
ax.set_ylabel('j (y-pitch direction)')
ax.set_title(f'AERIS-10 4x4 array — coupling |S_j,({DRIVEN_X},{DRIVEN_Y})| at {F0/1e9:.2f} GHz')
# Annotate cells
for j in range(N_Y):
    for i in range(N_X):
        if (i, j) == (DRIVEN_X, DRIVEN_Y):
            ax.text(i, j, "DRIVEN", ha='center', va='center', color='red',
                    fontsize=9, fontweight='bold')
        else:
            ax.text(i, j, f"{grid[j,i]:.1f}\ndB", ha='center', va='center',
                    color='white', fontsize=8)
plt.colorbar(im, ax=ax, label='|S| (dB)', shrink=0.7)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "coupling_grid.png"), dpi=140)
plt.close(fig)

print(f"[out] {OUT_DIR}/coupling_grid.png")
print(f"[out] {OUT_DIR}/S_matrix.csv")
print(f"[out] {OUT_DIR}/S11_data.csv")
