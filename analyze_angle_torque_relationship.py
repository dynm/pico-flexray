#!/usr/bin/env python3
import argparse
import csv
import math
import os
import statistics
from typing import List, Optional, Tuple


def parse_hex_data(data_str: str) -> Optional[bytes]:
    if not data_str:
        return None
    s = data_str.strip()
    if s.startswith("0x") or s.startswith("0X"):
        s = s[2:]
    s = s.replace(" ", "")
    if len(s) % 2 != 0:
        return None
    try:
        return bytes.fromhex(s)
    except ValueError:
        return None


def get_u8(frame: bytes, idx: int) -> int:
    return frame[idx] if 0 <= idx < len(frame) else 0


def get_u16_be(frame: bytes, idx: int) -> int:
    return (get_u8(frame, idx) << 8) | get_u8(frame, idx + 1)


def get_u16_le(frame: bytes, idx: int) -> int:
    return get_u8(frame, idx) | (get_u8(frame, idx + 1) << 8)


def decode_acc_fields(frame: bytes) -> Optional[Tuple[int, float, float, int, int, int]]:
    """
    Decode fields from BO_ 72 ACC (id 72 dec == 0x48 hex), m1 payloads.

    Returns: (cycle_count, angle_req_deg, torque_req_nm, steering_engaged, lane_keeping_triggered, reserve_raw)

    DBC mapping (big-endian @1+):
      - cycle_count: Byte[0] (base=1)
      - steering_angle_req: 24|16 => Byte[3:5], scale 0.04, offset -1000 -> deg
      - steer_torque_req:    40|16 => Byte[5:7], scale 0.006, offset -196.596 -> Nm
      - lane_keeping_triggered: 72|8 => Byte[9]
      - steering_engaged: 116|4 => Byte[14] high nibble
    """
    if len(frame) < 17:
        return None
    cycle = frame[0]
    raw_ang = get_u16_le(frame, 3)
    angle_deg = raw_ang * 0.04 - 1000.0
    raw_tq = get_u16_le(frame, 5)
    torque_nm = raw_tq * 0.006 - 196.596
    lane_trig = get_u8(frame, 9)
    reserve_raw = get_u8(frame, 10)
    b14 = get_u8(frame, 14)
    engaged = (b14 >> 4) & 0xF
    return cycle, angle_deg, torque_nm, engaged, lane_trig, reserve_raw


def get_bit_le(frame: bytes, bit_index: int) -> int:
    """Little-endian bit order as DBC @0+ specifies (Intel)."""
    byte_index = bit_index // 8
    bit_in_byte = bit_index % 8
    if byte_index >= len(frame):
        return 0
    return (frame[byte_index] >> bit_in_byte) & 1


def get_bits_le(frame: bytes, start_bit: int, length: int) -> int:
    """Extract 'length' bits starting at start_bit (Intel), return as unsigned int."""
    val = 0
    for k in range(length):
        b = get_bit_le(frame, start_bit + k)
        val |= (b << k)
    return val


def decode_acc_flags_le(frame: bytes) -> Tuple[int, int, int]:
    """
    Additional ACC flags (Intel):
      - assist_mode: 65|2@0+
      - wayback_en1_lane_keeping_trigger: 71|6@0+ (nonzero => enabled)
      - wayback_en_2: 114|2@0+
    Returns (assist_mode, wayback1_enabled, wayback2_val)
    """
    assist_mode = get_bits_le(frame, 65, 2)
    wayback1_val = get_bits_le(frame, 71, 6)
    wayback1_enabled = 1 if wayback1_val != 0 else 0
    wayback2_val = get_bits_le(frame, 114, 2)
    return assist_mode, wayback1_enabled, wayback2_val

def decode_eps_angle(frame: bytes) -> Optional[float]:
    """
    BO_ 51 EPS_Angle: M at 0|1@0+, steering_angle m0 : 24|16@1+ (0.0439453125,-1440)
    Only decode when mux == 0.
    """
    if len(frame) < 7:
        return None
    mux = get_bit_le(frame, 0)
    if mux != 0:
        return None
    raw = get_u16_le(frame, 3)
    return raw * 0.0439453125 - 1440.0


def decode_target_steering_torque(frame: bytes) -> Optional[float]:
    """
    BO_ 68 target_steering_torque: mux 0|1@0+, field 88|12@1+ (0.005,-10)
    12 bits big-endian starting at byte 11.
    """
    if len(frame) < 13:
        return None
    mux = get_bit_le(frame, 0)
    if mux != 0:
        return None
    b11 = get_u8(frame, 11)
    b12 = get_u8(frame, 12)
    # Intel (little-endian) 12 bits starting at byte 11, then low nibble of byte 12
    raw12 = (b11 | ((b12 & 0x0F) << 8)) & 0x0FFF
    return raw12 * 0.005 - 10.0


def decode_driver_steer_torque(frame: bytes) -> Optional[float]:
    """
    BO_ 49 steer_torque: mux 0|1@0+, driver_steer_torque m0 : 24|12@1+ (0.005,-10)
    12 bits big-endian starting at byte 3.
    """
    if len(frame) < 5:
        return None
    mux = get_bit_le(frame, 0)
    if mux != 0:
        return None
    b3 = get_u8(frame, 3)
    b4 = get_u8(frame, 4)
    # Intel (little-endian) 12 bits starting at byte 3, then low nibble of byte 4
    raw12 = (b3 | ((b4 & 0x0F) << 8)) & 0x0FFF
    return raw12 * 0.005 - 10.0


def decode_vehicle_speed(frame: bytes) -> Optional[float]:
    """
    BO_ 55 NEW_MSG_37: veh_speed m3 : 24|16@1+ (0.015625,0)
    We won't parse mux (m3). We'll decode and later filter plausible ranges.
    """
    if len(frame) < 5:
        return None
    raw = get_u16_le(frame, 3)
    speed = raw * 0.015625  # km/h
    # Plausibility filter
    if speed < 0 or speed > 300:
        return None
    return speed


def pearson_r(xs: List[float], ys: List[float]) -> float:
    if len(xs) != len(ys) or len(xs) < 2:
        return float("nan")
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    num = 0.0
    dx2 = 0.0
    dy2 = 0.0
    for x, y in zip(xs, ys):
        dx = x - mx
        dy = y - my
        num += dx * dy
        dx2 += dx * dx
        dy2 += dy * dy
    if dx2 <= 0 or dy2 <= 0:
        return float("nan")
    return num / math.sqrt(dx2 * dy2)


def ols_two_predictors(y: List[float], x1: List[float], x2: List[float]) -> Tuple[float, float, float]:
    """Centered OLS without intercept. Returns (b1, b2, R2)."""
    n = min(len(y), len(x1), len(x2))
    if n < 3:
        return float('nan'), float('nan'), float('nan')
    y_c = [v - statistics.fmean(y) for v in y]
    x1_c = [v - statistics.fmean(x1) for v in x1]
    x2_c = [v - statistics.fmean(x2) for v in x2]
    S11 = sum(v * v for v in x1_c)
    S22 = sum(v * v for v in x2_c)
    S12 = sum(a * b for a, b in zip(x1_c, x2_c))
    S1y = sum(a * b for a, b in zip(x1_c, y_c))
    S2y = sum(a * b for a, b in zip(x2_c, y_c))
    det = S11 * S22 - S12 * S12
    if abs(det) < 1e-9:
        return float('nan'), float('nan'), float('nan')
    b1 = (S22 * S1y - S12 * S2y) / det
    b2 = (S11 * S2y - S12 * S1y) / det
    # R^2
    y_hat = [b1 * a + b2 * b for a, b in zip(x1_c, x2_c)]
    ss_res = sum((yv - yh) ** 2 for yv, yh in zip(y_c, y_hat))
    ss_tot = sum(yv ** 2 for yv in y_c)
    R2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return b1, b2, R2


def diff(series: List[float], times: List[float]) -> List[float]:
    if len(series) < 2:
        return []
    out: List[float] = []
    for i in range(1, len(series)):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            out.append(0.0)
        else:
            out.append((series[i] - series[i - 1]) / dt)
    return out


def nearest_align(
    t1: List[float], v1: List[float], t2: List[float], v2: List[float], dt_max: float
) -> Tuple[List[float], List[float]]:
    """Align two time series by nearest timestamp within dt_max (seconds)."""
    i = j = 0
    out1: List[float] = []
    out2: List[float] = []
    while i < len(t1) and j < len(t2):
        dt = t1[i] - t2[j]
        if abs(dt) <= dt_max:
            out1.append(v1[i])
            out2.append(v2[j])
            i += 1
            j += 1
        elif dt > 0:
            j += 1
        else:
            i += 1
    return out1, out2


def analyze(csv_path: str, engaged_only: bool, exclude_lda: bool, plot: bool, run_ols: bool = False) -> None:
    times: List[float] = []
    angle_deg: List[float] = []
    torque_nm: List[float] = []
    reserve_raws: List[int] = []
    acc_wayback1: List[int] = []
    acc_wayback2: List[int] = []
    acc_assist_mode: List[int] = []

    # Additional signals storage
    t_eps: List[float] = []
    eps_angle_deg: List[float] = []
    t_tgt_tq: List[float] = []
    target_torque_nm: List[float] = []
    t_drv_tq: List[float] = []
    driver_torque_nm: List[float] = []
    t_speed: List[float] = []
    speed_kmh: List[float] = []

    total = parsed = acc_mux = kept = 0

    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        plt = None
        if plot:
            print("matplotlib not available; plots will be skipped.")

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            total += 1
            try:
                t = float(row.get("time", "0") or 0.0)
            except ValueError:
                continue
            addr_s = (row.get("addr") or "").strip()
            try:
                addr = int(addr_s, 16) if addr_s.lower().startswith("0x") else int(addr_s)
            except ValueError:
                continue
            data_b = parse_hex_data(row.get("data") or "")
            if not data_b:
                continue
            parsed += 1

            # Focus on ACC (id 72 dec == 0x48 hex)
            if addr == 0x48:
                dec = decode_acc_fields(data_b)
                if not dec:
                    continue
                cycle, ang, tq, engaged, lane_trig, reserve_raw = dec
                if cycle % 4 != 1:
                    continue
                acc_mux += 1
                if engaged_only and engaged == 0:
                    continue
                if exclude_lda and lane_trig != 0:
                    continue
                times.append(t)
                angle_deg.append(ang)
                torque_nm.append(tq)
                reserve_raws.append(reserve_raw)
                # flags
                am, wb1, wb2 = decode_acc_flags_le(data_b)
                acc_assist_mode.append(am)
                acc_wayback1.append(wb1)
                acc_wayback2.append(wb2)
                kept += 1
                continue

            # EPS angle
            if addr == 0x33:
                val = decode_eps_angle(data_b)
                if val is not None:
                    t_eps.append(t)
                    eps_angle_deg.append(val)
                continue

            # target steering torque
            if addr == 0x44:
                val = decode_target_steering_torque(data_b)
                if val is not None:
                    t_tgt_tq.append(t)
                    target_torque_nm.append(val)
                continue

            # driver torque
            if addr == 0x31:
                val = decode_driver_steer_torque(data_b)
                if val is not None:
                    t_drv_tq.append(t)
                    driver_torque_nm.append(val)
                continue

            # vehicle speed (km/h)
            if addr == 0x37:
                val = decode_vehicle_speed(data_b)
                if val is not None:
                    t_speed.append(t)
                    speed_kmh.append(val)
                continue

    if kept < 5:
        print("Not enough frames kept for analysis.")
        print(f"Total: {total}, parsed: {parsed}, ACC mux: {acc_mux}, kept: {kept}")
        return

    # Basic correlations
    abs_torque = [abs(x) for x in torque_nm]
    r_ang_tq = pearson_r(angle_deg, torque_nm)
    # Derivative of angle
    d_ang = diff(angle_deg, times)
    # align lengths
    aligned_torque = torque_nm[1:]
    r_dang_tq = pearson_r(d_ang, aligned_torque)

    # Activity fractions and co-activation
    angle_thr = 0.3  # deg
    torque_thr = 1.5  # Nm
    angle_active = sum(1 for a in angle_deg if abs(a) > angle_thr) / len(angle_deg)
    torque_active = sum(1 for v in abs_torque if v > torque_thr) / len(abs_torque)
    co_idx = [i for i, (a, v) in enumerate(zip(angle_deg, abs_torque)) if abs(a) > angle_thr and v > torque_thr]
    co_fraction = len(co_idx) / len(angle_deg)
    if co_idx:
        ang_co = [angle_deg[i] for i in co_idx]
        tq_co = [torque_nm[i] for i in co_idx]
        r_ang_tq_co = pearson_r(ang_co, tq_co)
    else:
        r_ang_tq_co = float('nan')

    print("==== Angle vs Torque Analysis (ACC m1 demux) ====")
    print(f"File: {os.path.basename(csv_path)}")
    print(f"Frames kept: {kept} (from total {total}, parsed {parsed}, ACC mux {acc_mux})")
    print(f"Engaged-only: {engaged_only}, Exclude LDA: {exclude_lda}")
    print("")
    print(f"Corr(angle_req, torque_req): {r_ang_tq:.3f}")
    print(f"Corr(d(angle_req)/dt, torque_req): {r_dang_tq:.3f}")
    print(f"Activity: angle>|{angle_thr}deg| fraction={angle_active:.2%}, torque>|{torque_thr}Nm| fraction={torque_active:.2%}")
    print(f"Co-activation fraction: {co_fraction:.2%}")
    print(f"Corr(angle, torque) on co-active frames: {r_ang_tq_co:.3f}")

    # Cross-signals correlations with alignment
    if t_eps and eps_angle_deg:
        a1, a2 = nearest_align(times, angle_deg, t_eps, eps_angle_deg, dt_max=0.01)
        r_angle_track = pearson_r(a1, a2)
        print(f"Corr(angle_req, EPS_steering_angle) [aligned 10ms]: {r_angle_track:.3f}")
        # Align torque to EPS timeline as well
        _, tq_on_eps = nearest_align(t_eps, eps_angle_deg, times, torque_nm, dt_max=0.01)
        # Angle req on EPS timeline
        _, ang_on_eps = nearest_align(t_eps, eps_angle_deg, times, angle_deg, dt_max=0.01)
        if len(tq_on_eps) == len(eps_angle_deg[: len(tq_on_eps)]) and len(ang_on_eps) == len(tq_on_eps):
            # Compute angle error on EPS time base
            err = [ea - ar for ea, ar in zip(eps_angle_deg[: len(ang_on_eps)], ang_on_eps)]
            r_tq_err = pearson_r(tq_on_eps[: len(err)], err)
            print(f"Corr(torque_req(on EPS), angle_error(EPS-angle_req)) [10ms align]: {r_tq_err:.3f}")
            # EPS angle rate
            eps_rate = diff(eps_angle_deg, t_eps)
            if len(eps_rate) and len(tq_on_eps) > 1:
                r_tq_rate = pearson_r(tq_on_eps[1: 1 + len(eps_rate)], eps_rate[: len(tq_on_eps) - 1])
                print(f"Corr(torque_req(on EPS), d(EPS_angle)/dt) [10ms align]: {r_tq_rate:.3f}")
    if t_tgt_tq and target_torque_nm:
        b1, b2 = nearest_align(times, torque_nm, t_tgt_tq, target_torque_nm, dt_max=0.01)
        r_tq_tgt = pearson_r(b1, b2)
        print(f"Corr(torque_req, target_steering_torque) [aligned 10ms]: {r_tq_tgt:.3f}")
        # Also check ACC angle vs EPS target torque
        a3, b3 = nearest_align(times, angle_deg, t_tgt_tq, target_torque_nm, dt_max=0.01)
        if a3 and b3:
            r_ang_tgt = pearson_r(a3, b3)
            print(f"Corr(angle_req, target_steering_torque) [aligned 10ms]: {r_ang_tgt:.3f}")
    if t_drv_tq and driver_torque_nm and reserve_raws:
        # align driver torque to ACC times for reserve correlation
        c1, c2 = nearest_align(times, reserve_raws, t_drv_tq, [abs(v) for v in driver_torque_nm], dt_max=0.01)
        if c1 and c2:
            r_reserve_vs_driver = pearson_r(c1, [-v for v in c2])
            print(f"Corr(reserve_raw, -|driver_steer_torque|) [aligned 10ms]: {r_reserve_vs_driver:.3f}")
    # Correlations: ACC torque_req vs driver_steer_torque (signed and abs)
    if t_drv_tq and driver_torque_nm:
        d1, d2 = nearest_align(times, torque_nm, t_drv_tq, driver_torque_nm, dt_max=0.01)
        if d1 and d2:
            r_tq_drv = pearson_r(d1, d2)
            r_tq_absdrv = pearson_r(d1, [abs(x) for x in d2])
            print(f"Corr(torque_req, driver_steer_torque) [aligned 10ms]: {r_tq_drv:.3f}")
            print(f"Corr(torque_req, |driver_steer_torque|) [aligned 10ms]: {r_tq_absdrv:.3f}")

    # Per-speed-bin correlations (align speed to ACC time base)
    if t_speed and speed_kmh:
        # Align speed to ACC timeline and capture indices
        i = j = 0
        aligned_speed: List[Optional[float]] = []
        aligned_idx: List[int] = []
        dt_max = 0.05
        while i < len(times) and j < len(t_speed):
            dt = times[i] - t_speed[j]
            if abs(dt) <= dt_max:
                aligned_speed.append(speed_kmh[j])
                aligned_idx.append(i)
                i += 1
                j += 1
            elif dt > 0:
                j += 1
            else:
                i += 1
        # Align EPS angle to ACC timeline as well (for EPS vs inputs per-speed)
        i2 = j2 = 0
        eps_on_acc: List[float] = []
        eps_idx_on_acc: List[int] = []
        dt_eps = 0.01
        while i2 < len(times) and j2 < len(t_eps):
            dt = times[i2] - t_eps[j2]
            if abs(dt) <= dt_eps:
                eps_on_acc.append(eps_angle_deg[j2])
                eps_idx_on_acc.append(i2)
                i2 += 1
                j2 += 1
            elif dt > 0:
                j2 += 1
            else:
                i2 += 1

        if aligned_speed:
            bins = [(0, 10), (10, 30), (30, 60), (60, 200)]
            print("Per-speed-bin correlations (km/h):")
            for lo, hi in bins:
                idx_local = [aligned_idx[k] for k, v in enumerate(aligned_speed) if v is not None and v >= lo and v < hi]
                if len(idx_local) >= 50:
                    ang_series = [angle_deg[i2] for i2 in idx_local]
                    tq_series = [torque_nm[i2] for i2 in idx_local]
                    t_series = [times[i2] for i2 in idx_local]
                    r1 = pearson_r(ang_series, tq_series)
                    r2 = pearson_r(diff(ang_series, t_series), tq_series[1:]) if len(ang_series) > 1 else float('nan')
                    # Intersect with indices where EPS angle is available
                    idx_set = set(idx_local)
                    eps_pairs = [
                        (angle_deg[i3], eps_on_acc[k])
                        for k, i3 in enumerate(eps_idx_on_acc)
                        if i3 in idx_set and k < len(eps_on_acc)
                    ]
                    tq_eps_pairs = [
                        (torque_nm[i3], eps_on_acc[k])
                        for k, i3 in enumerate(eps_idx_on_acc)
                        if i3 in idx_set and k < len(eps_on_acc)
                    ]
                    if len(eps_pairs) >= 30:
                        r_ang_eps = pearson_r([p[0] for p in eps_pairs], [p[1] for p in eps_pairs])
                    else:
                        r_ang_eps = float('nan')
                    if len(tq_eps_pairs) >= 30:
                        r_tq_eps = pearson_r([p[0] for p in tq_eps_pairs], [p[1] for p in tq_eps_pairs])
                    else:
                        r_tq_eps = float('nan')

                    print(
                        f"  {lo:>3}-{hi:<3} km/h: r(angle, torque)={r1:.3f}, r(d(angle)/dt, torque)={r2:.3f}, "
                        f"r(angle, EPS)={r_ang_eps:.3f}, r(torque, EPS)={r_tq_eps:.3f} (n={len(idx_local)})"
                    )
                else:
                    print(f"  {lo:>3}-{hi:<3} km/h: insufficient samples (n={len(idx_local)})")

            # Standstill grouping by wayback flags (speed < 1 km/h)
            standstill_idx = [aligned_idx[k] for k, v in enumerate(aligned_speed) if v is not None and v < 1.0]
            if standstill_idx:
                idx_set = set(standstill_idx)
                # Group A: any wayback flag enabled
                groupA = [i for i in standstill_idx if (acc_wayback1[i] != 0 or acc_wayback2[i] != 0)]
                groupB = [i for i in standstill_idx if (acc_wayback1[i] == 0 and acc_wayback2[i] == 0)]
                def r_angle_eps_for(indices: List[int]) -> float:
                    if not indices:
                        return float('nan')
                    idx_local_set = set(indices)
                    pairs = [
                        (angle_deg[i3], eps_on_acc[k])
                        for k, i3 in enumerate(eps_idx_on_acc)
                        if i3 in idx_local_set and k < len(eps_on_acc)
                    ]
                    if len(pairs) < 30:
                        return float('nan')
                    return pearson_r([p[0] for p in pairs], [p[1] for p in pairs])
                rA = r_angle_eps_for(groupA)
                rB = r_angle_eps_for(groupB)
                print(f"Standstill (<1 km/h) r(angle, EPS) with wayback_on: {rA:.3f} (n={len(groupA)}), wayback_off: {rB:.3f} (n={len(groupB)})")

        if run_ols:
            # Lag scan + OLS on EPS time-base (engaged-only already applied)
            # Build joint series on EPS time base by aligning ACC inputs to EPS timestamps
            dt_align = 0.02
            i_acc = 0
            x1_on_eps: List[float] = []
            x2_on_eps: List[float] = []
            y_eps: List[float] = []
            sp_on_eps: List[float] = []
            # map speed on ACC index for quick lookup (reuse aligned_speed mapping)
            speed_map = {}
            for k, idx_acc in enumerate(aligned_idx):
                speed_map[idx_acc] = aligned_speed[k]
            for j, t_e in enumerate(t_eps):
                # find nearest ACC index i where |t_acc - t_eps| < dt_align
                while i_acc + 1 < len(times) and abs(times[i_acc + 1] - t_e) <= abs(times[i_acc] - t_e):
                    i_acc += 1
                if abs(times[i_acc] - t_e) <= dt_align:
                    y_eps.append(eps_angle_deg[j])
                    x1_on_eps.append(angle_deg[i_acc])
                    x2_on_eps.append(torque_nm[i_acc])
                    sp_on_eps.append(speed_map.get(i_acc, float('nan')))

            def run_bins_with_lag(title: str, idxs: List[int]):
                if not idxs:
                    print(f"{title}: no samples")
                    return
                best = None
                for lag in range(-3, 4):
                    xs1 = []
                    xs2 = []
                    ys = []
                    for i0 in idxs:
                        i = i0 + lag
                        if i < 0 or i >= len(y_eps):
                            continue
                        xs1.append(x1_on_eps[i])
                        xs2.append(x2_on_eps[i])
                        ys.append(y_eps[i])
                    if len(ys) < 50:
                        continue
                    b1, b2, R2 = ols_two_predictors(ys, xs1, xs2)
                    if not (math.isnan(R2)):
                        if best is None or R2 > best[0]:
                            best = (R2, lag, b1, b2, len(ys))
                if best is None:
                    print(f"{title}: insufficient samples")
                else:
                    R2, lag, b1, b2, n = best
                    print(f"{title}: best_lag={lag} samples, OLS R2={R2:.3f}, betas: angle={b1:.4f}, torque={b2:.4f} (n={n})")

            if y_eps:
                # Build indices per speed bin on EPS base where speed is known
                idx_all = [i for i, v in enumerate(sp_on_eps) if not math.isnan(v)]
                print("Lag-scan OLS (EPS base, engaged-only):")
                run_bins_with_lag("  ALL", idx_all)
                bins = [(0, 10), (10, 30), (30, 60), (60, 200)]
                for lo, hi in bins:
                    idx_bin = [i for i in idx_all if sp_on_eps[i] >= lo and sp_on_eps[i] < hi]
                    run_bins_with_lag(f"  {lo:>3}-{hi:<3} km/h", idx_bin)

    # Heuristic classification
    mode = "torque-dominant (LKA style)" if torque_active > angle_active else "angle-dominant"
    print(f"Inferred mode (heuristic): {mode}")

    if plot and plt is not None:
        out_dir = os.path.join(os.path.dirname(csv_path), "plots")
        os.makedirs(out_dir, exist_ok=True)

        import matplotlib.pyplot as plt2  # type: ignore

        fig, ax = plt2.subplots(4, 1, figsize=(12, 11), sharex=True)
        ax[0].plot(times, angle_deg, lw=0.6)
        ax[0].set_ylabel("angle req (deg)")
        ax[1].plot(times, torque_nm, lw=0.6)
        ax[1].set_ylabel("torque req (Nm)")
        # simple normalized view
        max_tq = max(1e-6, max(abs(v) for v in torque_nm))
        norm_tq = [v / max_tq for v in torque_nm]
        ax[2].plot(times, angle_deg, lw=0.5, label="angle (deg)")
        ax[2].plot(times, norm_tq, lw=0.5, label="torque (norm)")
        ax[2].legend()
        ax[2].set_ylabel("angle & torque (norm)")
        # reserve
        if reserve_raws:
            ax[3].plot(times, reserve_raws, lw=0.5, label="reserve (raw)")
            ax[3].legend()
        ax[3].set_xlabel("time (s)")
        fig.tight_layout()
        pth = os.path.join(out_dir, "angle_vs_torque_timeseries.png")
        fig.savefig(pth, dpi=150)
        plt2.close(fig)
        print(f"Saved plot: {pth}")


def main():
    ap = argparse.ArgumentParser(description="Analyze relationship between angle_req and torque_req (ACC demux)")
    ap.add_argument("csv_path", help="CSV with time,addr,bus,data")
    ap.add_argument("--engaged-only", action="store_true", help="Keep steering_engaged != 0")
    ap.add_argument("--exclude-lda", action="store_true", help="Exclude lane_keeping_triggered != 0")
    ap.add_argument("--plot", action="store_true", help="Generate plot")
    ap.add_argument("--ols", action="store_true", help="Run heavy lag-scan OLS (slower)")
    args = ap.parse_args()

    analyze(args.csv_path, engaged_only=args.engaged_only, exclude_lda=args.exclude_lda, plot=args.plot, run_ols=args.ols)


if __name__ == "__main__":
    main()


