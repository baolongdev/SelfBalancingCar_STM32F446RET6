"""Phan tich luc dung xe len (TILT -> active) trong log [T].

Dung: python standup_events.py capture.log
In 0.6 s dau sau moi lan xe bat dau tu can bang: goc luc bat, PWM, xe co
giu duoc hay nga lai.
"""
import re
import sys

LINE = re.compile(r"\[\s*(\d+) ms\]\[INF\]\[T\] a=(-?\d+) g=(-?\d+) o=(-?\d+) l=(-?\d+) r=(-?\d+) f=([0-9a-fA-F]+)")

for path in sys.argv[1:]:
    rows = [(int(m[1]), int(m[2]) / 100, int(m[3]) / 100, int(m[4]) / 100, int(m[5]), int(m[6]), int(m[7], 16))
            for m in map(LINE.search, open(path, encoding="utf-8", errors="replace")) if m]
    print(f"== {path}: {len(rows)} mau")
    for i in range(1, len(rows)):
        if (rows[i - 1][6] & 0x4) and rows[i][6] == 0:
            t0 = rows[i][0]
            seg = [r for r in rows[i:i + 400] if r[0] - t0 <= 2000]
            fall = next((r for r in seg if r[6] & 0x4), None)
            print(f"-- bat dau tu can bang luc t={(t0 - rows[0][0]) / 1000:.2f}s, goc lech {rows[i][1]:+.1f} deg, "
                  f"{'NGA sau ' + str(fall[0] - t0) + ' ms' if fall else 'giu duoc >= 2 s'}")
            for r in seg[:75:5]:
                print(f"   {r[0] - t0:+5d}ms err={r[1]:+6.1f} gyro={r[2]:+7.1f} out={r[3]:+7.1f} L={r[4]:+4d} R={r[5]:+4d}")
