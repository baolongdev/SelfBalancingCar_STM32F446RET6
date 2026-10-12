"""Phan tich log [T] (BAL LOG <ms>) de tune balance.

Dung: python tune_analyze.py capture.log [capture2.log ...]

Dong log: [   12345 ms][INF][T] a=<err cdeg> g=<gyro cdps> o=<out c%> l=<pwmL> r=<pwmR> f=<fault hex>
  a = target - angle (deg*100), g = gyro dung cho balance (dps*100).
"""
import math
import re
import sys

LINE = re.compile(r"\[\s*(\d+) ms\]\[INF\]\[T\] a=(-?\d+) g=(-?\d+) o=(-?\d+) l=(-?\d+) r=(-?\d+) f=([0-9a-fA-F]+)(?: s=(-?\d+))?")


def load(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = LINE.search(line)
            if m:
                t, a, g, o, l, r, f, s = m.groups()
                rows.append((int(t) / 1000.0, int(a) / 100.0, int(g) / 100.0,
                             int(o) / 100.0, int(l), int(r), int(f, 16),
                             int(s) / 100.0 if s is not None else 0.0))
    return rows


def analyze(path):
    rows = load(path)
    print(f"== {path}: {len(rows)} mau")
    if len(rows) < 10:
        print("  qua it mau [T] (BAL LOG chua bat?)")
        return
    active = [x for x in rows if x[6] == 0]
    span = rows[-1][0] - rows[0][0]
    print(f"  thoi gian {span:.2f} s, dt tb {1000 * span / (len(rows) - 1):.1f} ms, active {100 * len(active) / len(rows):.0f}%")
    falls = sum(1 for p, c in zip(rows, rows[1:]) if (c[6] & 0x4) and not (p[6] & 0x4))
    print(f"  so lan nga (vao TILT): {falls}")
    if len(active) < 10:
        return
    errs = [x[1] for x in active]
    mean = sum(errs) / len(errs)
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    sat = sum(1 for x in active if abs(x[3]) >= 99.0) / len(active)
    print(f"  err: mean {mean:+.2f} deg, rms {rms:.2f} deg, min {min(errs):+.2f}, max {max(errs):+.2f}")
    print(f"  out bao hoa (|o|>=99%): {100 * sat:.0f}%")
    outs = [x[3] for x in active]
    shifts = [x[7] for x in active]
    print(f"  out mean {sum(outs) / len(outs):+.1f}% (lech ve 1 phia = xe dang troi), "
          f"target shift mean {sum(shifts) / len(shifts):+.2f} deg")
    gy = [x[2] for x in active]
    print(f"  gyro rms {math.sqrt(sum(v * v for v in gy) / len(gy)):.1f} dps")

    # Tan so dao dong tu so lan doi dau cua (err - mean).
    crossings = sum(1 for p, c in zip(errs, errs[1:]) if (p - mean) * (c - mean) < 0)
    act_span = active[-1][0] - active[0][0]
    if act_span > 0:
        print(f"  tan so dao dong ~{crossings / 2 / act_span:.1f} Hz")

    # Kiem tra dau gyro: err = target - angle => d(err)/dt phai nguoc dau gyro.
    num = den_a = den_b = 0.0
    for p, c in zip(active, active[1:]):
        dt = c[0] - p[0]
        if dt <= 0 or dt > 0.05:
            continue
        derr = (c[1] - p[1]) / dt
        gmid = 0.5 * (c[2] + p[2])
        num += derr * gmid
        den_a += derr * derr
        den_b += gmid * gmid
    if den_a > 0 and den_b > 0:
        corr = num / math.sqrt(den_a * den_b)
        verdict = "OK (gyro cung chieu goc)" if corr < -0.3 else ("SAI DAU gyro!" if corr > 0.3 else "khong ro")
        print(f"  corr(d err/dt, gyro) = {corr:+.2f} -> {verdict}")


for p in sys.argv[1:]:
    analyze(p)
