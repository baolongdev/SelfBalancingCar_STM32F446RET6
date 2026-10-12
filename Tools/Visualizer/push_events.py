"""Tim cac lan bi day trong log [T] va do xe hoi phuc ra sao.

Dung: python push_events.py capture.log
Mot "su kien" bat dau khi |err| > 5 deg va ket thuc khi |err| < 2 deg.
"""
import re
import sys

LINE = re.compile(r"\[\s*(\d+) ms\]\[INF\]\[T\] a=(-?\d+) g=(-?\d+) o=(-?\d+) l=(-?\d+) r=(-?\d+) f=([0-9a-fA-F]+)")

for path in sys.argv[1:]:
    rows = [(int(m[1]), int(m[2]) / 100, int(m[4]) / 100, int(m[7], 16))
            for m in map(LINE.search, open(path, encoding="utf-8", errors="replace")) if m]
    print(f"== {path}: {len(rows)} mau")
    i = 0
    while i < len(rows):
        if abs(rows[i][1]) > 5:
            j = i
            while j < len(rows) and abs(rows[j][1]) > 2 and not (rows[j][3] & 4):
                j += 1
            seg = rows[i:j] or rows[i:i + 1]
            peak = max(seg, key=lambda r: abs(r[1]))
            sat = sum(1 for r in seg if abs(r[2]) >= 99)
            fell = any(r[3] & 4 for r in rows[i:j + 3])
            end = rows[min(j, len(rows) - 1)][0]
            print(f"  t={(rows[i][0] - rows[0][0]) / 1000:6.2f}s peak {peak[1]:+6.1f} deg, "
                  f"hoi ve |err|<2 sau {end - rows[i][0]} ms, bao hoa {sat} mau, "
                  f"{'NGA' if fell else 'hoi phuc'}")
            i = j + 1
        else:
            i += 1
