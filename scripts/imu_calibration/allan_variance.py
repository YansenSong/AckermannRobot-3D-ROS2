#!/usr/bin/env python3
"""Overlapping Allan-variance analysis of a static IMU CSV recording.

Reads the CSV produced by record_imu.py and prints the noise parameters that
LIO-SAM consumes:

    imuGyrNoise  = gyroscope white-noise density       (rad/s/sqrt(Hz))
    imuGyrBiasN  = gyroscope bias random walk          (rad/s/sqrt(s))
    imuAccNoise  = accelerometer white-noise density   (m/s^2/sqrt(Hz))
    imuAccBiasN  = accelerometer bias random walk      (m/s^2/sqrt(s))

Reference for the mapping: LIO-SAM imuPreintegration.cpp builds
gtsam PreintegrationParams with accelerometerCovariance = imuAccNoise^2,
gyroscopeCovariance = imuGyrNoise^2 and the between-bias noise vector
(imuAccBiasN, ..., imuGyrBiasN, ...).

Usage:

    python3 scripts/imu_calibration/allan_variance.py imu_static.csv [--plot out.png]

Standard IEEE 952 relations used below (sigma = Allan deviation):
    white noise:        sigma(tau) = N / sqrt(tau)          -> N = sigma*sqrt(tau)
    rate random walk:   sigma(tau) = K * sqrt(tau/3)        -> K = sigma*sqrt(3/tau)
"""

import argparse
import math
import sys

import numpy as np


def load_csv(path, drop_first_s=10.0):
    """Load the recording, estimate dt, drop the warm-up transient."""
    raw = np.loadtxt(path, delimiter=',', skiprows=1)
    secs = raw[:, 0] + raw[:, 1] * 1e-9
    acc = raw[:, 2:5]
    gyr = raw[:, 5:8]
    # Remove warm-up transient (sensor settling right after startup).
    if len(secs) > 1 and secs[-1] > secs[0] + drop_first_s:
        mask = secs >= secs[0] + drop_first_s
        secs, acc, gyr = secs[mask], acc[mask], gyr[mask]
    dt = float(np.median(np.diff(secs)))
    return secs, acc, gyr, dt


def allan_deviation(y, dt, min_m=1, max_m=None):
    """Overlapping Allan deviation of a 1-D rate/accel series.

    Returns (taus, adev). O(N) per cluster length via cumulative sums.
    """
    y = np.asarray(y, dtype=float)
    n = len(y)
    if max_m is None:
        max_m = n // 4
    cumsum = np.concatenate(([0.0], np.cumsum(y)))

    ms = []
    m = min_m
    while m <= max_m:
        ms.append(m)
        m = m + 1 if m < 10 else int(m * 1.1) + 1

    taus = np.empty(len(ms))
    adev = np.empty(len(ms))
    for i, m in enumerate(ms):
        # windowed sums over m consecutive samples, s[k] = sum(y[k..k+m-1])
        win = cumsum[m:] - cumsum[:-m]
        n_terms = n - 2 * m
        if n_terms <= 0:
            taus[i] = m * dt
            adev[i] = np.nan
            continue
        diff = win[m:] - win[:-m]
        var = float(np.sum(diff * diff)) / (2.0 * n_terms * m * m)
        taus[i] = m * dt
        adev[i] = math.sqrt(var) if var > 0 else np.nan
    ok = ~np.isnan(adev)
    return taus[ok], adev[ok]


def _slope_region(x, y, target_slope, tol=0.25):
    """Longest contiguous run of points whose local log-log slope is near target_slope."""
    slopes = np.gradient(y, x)
    good = np.abs(slopes - target_slope) <= tol
    best_start, best_len = -1, 0
    cur_start, cur_len = -1, 0
    for i, g in enumerate(good):
        if g:
            if cur_len == 0:
                cur_start = i
            cur_len += 1
            if cur_len > best_len:
                best_start, best_len = cur_start, cur_len
        else:
            cur_len = 0
    if best_len == 0:
        return None
    return best_start, best_start + best_len


def extract(taus, adev, dt, name, unit_sqrt_hz, unit_rw):
    """Extract noise density (white), bias instability, and random walk."""
    if len(taus) < 5:
        return None

    # --- White noise density: sigma(tau) = N/sqrt(tau) -> N = sigma*sqrt(tau)
    N = None
    region = _slope_region(np.log(taus), np.log(adev), -0.5, tol=0.35)
    if region is not None:
        a, b = region
        # Prefer the lowest-tau part of the -1/2 run; ignore the very first point
        # (often quantization), cap the region to avoid the flat/Bi bend.
        start = a
        while start + 1 < b and (adev[start + 1] * math.sqrt(taus[start + 1]) < adev[start] * math.sqrt(taus[start])):
            start += 1
        seg = slice(start, b)
        N = float(np.mean(adev[seg] * np.sqrt(taus[seg])))
    if N is None:
        # Fallback: if tau = 1 s lies in the curve, read it directly.
        i1 = int(np.argmin(np.abs(taus - 1.0)))
        if np.isfinite(adev[i1]):
            N = float(adev[i1] * math.sqrt(taus[i1]))

    # --- Bias instability: flat region -> minimum of adev (reference only)
    B = float(np.nanmin(adev))

    # --- Random walk: sigma(tau) = K*sqrt(tau/3) -> K = sigma*sqrt(3/tau)
    K = None
    region = _slope_region(np.log(taus), np.log(adev), 0.5, tol=0.35)
    if region is not None:
        a, b = region
        K = float(np.mean(adev[a:b] * np.sqrt(3.0 / taus[a:b])))
    if K is None:
        # Fallback: use the largest-tau third of the curve.
        n = len(taus)
        seg = slice(max(2 * n // 3, 1), n)
        K = float(np.mean(adev[seg] * np.sqrt(3.0 / taus[seg])))

    print(f'--- {name} ---')
    print(f'  white-noise density : {N:.6e} {unit_sqrt_hz}')
    print(f'  bias instability    : {B:.6e} (reference, not used by LIO-SAM)')
    print(f'  bias random walk    : {K:.6e} {unit_rw}')
    return N, B, K


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', help='CSV produced by record_imu.py')
    ap.add_argument('--plot', default=None, help='optional PNG output path for the Allan plot')
    args = ap.parse_args()

    secs, acc, gyr, dt = load_csv(args.csv)
    duration = float(secs[-1] - secs[0])
    n = len(secs)
    print(f'Loaded {n} samples, {duration:.0f}s, dt={dt*1e3:.2f}ms '
          f'(rate~{1/dt:.1f}Hz)')

    # Static sanity check.
    print('\nStatic means / std (after warm-up cut):')
    for name, col in (('acc (m/s^2)', acc), ('gyr (rad/s)', gyr)):
        mean = col.mean(axis=0)
        std = col.std(axis=0)
        print(f'  {name}: mean=[{mean[0]:+.4f} {mean[1]:+.4f} {mean[2]:+.4f}] '
              f'std=[{std[0]:.2e} {std[1]:.2e} {std[2]:.2e}]')
    norm = np.sqrt(np.sum(np.mean(acc, axis=0) ** 2))
    print(f'  |acc| mean norm = {norm:.4f} m/s^2')

    print('\nAllan deviation per axis:')
    gyro_N, gyro_B, gyro_K = [], [], []
    acc_N, acc_B, acc_K = [], [], []
    for i, ax in enumerate(('X', 'Y', 'Z')):
        g_t, g_a = allan_deviation(gyr[:, i], dt)
        a_t, a_a = allan_deviation(acc[:, i], dt)
        r_g = extract(g_t, g_a, dt, f'gyr-{ax}', 'rad/s/sqrt(Hz)', 'rad/s/sqrt(s)')
        r_a = extract(a_t, a_a, dt, f'acc-{ax}', 'm/s^2/sqrt(Hz)', 'm/s^2/sqrt(s)')
        if r_g:
            gyro_N.append(r_g[0]); gyro_K.append(r_g[2])
        if r_a:
            acc_N.append(r_a[0]); acc_K.append(r_a[2])

    def agg(name, vals, unit):
        if not vals:
            print(f'  {name}: n/a')
            return
        arr = np.array(vals)
        print(f'  {name} = {arr.mean():.6e} {unit}  (per-axis [{arr[0]:.3e}, {arr[1]:.3e}, {arr[2]:.3e}], median {np.median(arr):.3e})')

    print('\nAggregated LIO-SAM parameters:')
    agg('imuGyrNoise', gyro_N, 'rad/s/sqrt(Hz)')
    agg('imuGyrBiasN', gyro_K, 'rad/s/sqrt(s)')
    agg('imuAccNoise', acc_N, 'm/s^2/sqrt(Hz)')
    agg('imuAccBiasN', acc_K, 'm/s^2/sqrt(s)')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))
        for i, ax in enumerate(('X', 'Y', 'Z')):
            a_t, a_a = allan_deviation(acc[:, i], dt)
            g_t, g_a = allan_deviation(gyr[:, i], dt)
            axs[0].loglog(a_t, a_a, label=f'acc-{ax}')
            axs[1].loglog(g_t, g_a, label=f'gyr-{ax}')
        axs[0].set_title('Accelerometer Allan deviation')
        axs[0].set_xlabel('tau (s)'); axs[0].set_ylabel('m/s^2')
        axs[0].legend()
        axs[1].set_title('Gyroscope Allan deviation')
        axs[1].set_xlabel('tau (s)'); axs[1].set_ylabel('rad/s')
        axs[1].legend()
        plt.tight_layout()
        plt.savefig(args.plot, dpi=120)
        print(f'\nPlot saved to {args.plot}')


if __name__ == '__main__':
    main()
