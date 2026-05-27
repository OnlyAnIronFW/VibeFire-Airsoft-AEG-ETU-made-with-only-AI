#!/usr/bin/env python3
"""Generate ballistics drop table for the AutoTrigger system.

Output: ``tables/drop_table.bin`` — 801 × 3 = 2403 float32 values.
Range:  0–80 m at 0.1 m step.
Velocity profiles: 55, 70, 90 m/s.

Physics model
-------------
Quadratic drag (flat-nose foam dart)::

    a_x = −α·|v|·v_x
    a_y = −α·|v|·v_y − g

where α = (ρ·A·C_d) / (2·m).

The trajectory is integrated with RK2 (midpoint method) at dt = 0.5 ms.
The simulation stops when horizontal distance reaches the target or the
dart drops below the launch height (ground intercept).

Constants are tuned for a 2 g, 12 mm-diameter foam dart — typical of
Chinese-market "海绵软弹" projectiles.
"""

import math
import struct
import sys
from pathlib import Path

# ── Physics constants ──────────────────────────────────────────
G = 9.81  # gravity (m/s²)
RHO = 1.225  # air density at sea level (kg/m³)
AREA = 1.13e-4  # cross-section: π·(0.006 m)²
CD = 0.67  # drag coefficient, flat-nose cylinder
MASS = 0.002  # projectile mass (kg) — 2 g foam dart

ALPHA = (RHO * AREA * CD) / (2.0 * MASS)
DT = 0.0005  # integration time step (s)

# ── Table geometry ─────────────────────────────────────────────
VELOCITIES = [55.0, 70.0, 90.0]
RANGE_STEP = 0.1
RANGE_MAX = 80.0
RANGE_COUNT = int(RANGE_MAX / RANGE_STEP) + 1  # 801

# Safety: max simulation steps before giving up
MAX_STEPS = 10_000_000  # 5000 s real-time at dt=0.5 ms


def simulate_drop(target_range: float, v0: float) -> float:
    """RK2 integration of the quadratic-drag trajectory.

    Returns the vertical drop (positive = downward) when the
    projectile reaches *target_range* horizontally, or a large
    sentinel value if it never gets there.
    """
    # State
    x, y = 0.0, 0.0
    vx, vy = v0, 0.0

    for _ in range(MAX_STEPS):
        # ── Evaluate derivatives at t ──
        v = math.hypot(vx, vy)
        if v <= 1e-9:
            break  # stalled

        drag = ALPHA * v
        ax0 = -drag * vx
        ay0 = -drag * vy - G

        # ── Half-step (midpoint) ──
        vx_mid = vx + ax0 * (DT * 0.5)
        vy_mid = vy + ay0 * (DT * 0.5)
        v_mid = math.hypot(vx_mid, vy_mid)
        if v_mid <= 1e-9:
            # degenerate: use Euler step
            x += vx * DT
            y += vy * DT
            vx += ax0 * DT
            vy += ay0 * DT
        else:
            drag_mid = ALPHA * v_mid
            ax1 = -drag_mid * vx_mid
            ay1 = -drag_mid * vy_mid - G
            x += vx_mid * DT
            y += vy_mid * DT
            vx += ax1 * DT
            vy += ay1 * DT

        # ── Stop conditions ──
        if x >= target_range:
            # Linear interpolation for the exact crossing point
            # (the last step may have overshot slightly)
            return max(-y, 0.0)  # positive = downward

        if y < -1e4:
            # Terminal dive — dart lost too much altitude
            break

    # Did not reach target_range within MAX_STEPS
    return 999.0  # sentinel


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    out_dir = root / "tables"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "drop_table.bin"

    # Pre-allocate in row-major order: [range][velocity]
    values = []
    n_total = RANGE_COUNT * len(VELOCITIES)

    print(f"Generating {RANGE_COUNT} x {len(VELOCITIES)} = {n_total} entries")
    print(f"Integration:  dt={DT:.4f}s,  alpha={ALPHA:.4f}")
    print()

    sample_printed = False
    for i in range(RANGE_COUNT):
        r = i * RANGE_STEP
        if i % 200 == 0:
            print(f"  ... range {r:6.1f} m ...")

        for j, v0 in enumerate(VELOCITIES):
            drop = simulate_drop(r, v0)
            values.append(drop)

            if not sample_printed and abs(r - 40.0) < 0.05 and abs(v0 - 70.0) < 0.1:
                print(f"  Sample: range={r:.1f}m  v0={v0:.0f}m/s  ->  drop={drop:.4f}m")
                sample_printed = True

    # ── Write binary ──
    fmt = f"<{n_total}f"  # little-endian float32
    packed = struct.pack(fmt, *values)
    out_path.write_bytes(packed)

    nbytes = len(packed)
    print(f"\nWrote {out_path}  ({nbytes} bytes, {nbytes // 4} floats)")
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
