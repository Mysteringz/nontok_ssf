"""A4 landscape checkerboard for the calibrate_camera.py defaults (nontok_localization): 9 x 6 inner
corners (10 x 7 squares), 25 mm squares, as an exact vector PDF that asks for no print scaling.
  python3 tools/make_checkerboard.py docs/checkerboard_9x6_25mm_A4.pdf
Print at 100 % (Actual size) and check the 100 mm bar with a ruler."""
import sys

MM = 72 / 25.4
COLS, ROWS, SQ = 10, 7, 25.0
W, H = 297.0, 210.0
X0 = (W - COLS * SQ) / 2          # 23.5 mm side margins
Y0 = 22.5                          # bottom margin holds the scale bar; top margin 12.5 mm

ops = ['0 g']
for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:
            ops.append('%.3f %.3f %.3f %.3f re f' % ((X0 + c * SQ) * MM, (Y0 + r * SQ) * MM, SQ * MM, SQ * MM))
bx, by = X0, 9.0                   # 100 mm bar with end ticks
ops.append('0.6 w %.3f %.3f m %.3f %.3f l S' % (bx * MM, by * MM, (bx + 100) * MM, by * MM))
for x in (bx, bx + 100):
    ops.append('%.3f %.3f m %.3f %.3f l S' % (x * MM, (by - 2) * MM, x * MM, (by + 2) * MM))
text = ('Checkerboard 9 x 6 inner corners, 25 mm squares. Print at 100 % (Actual size). '
        'The bar must be 100 mm and 10 squares 250 mm. Glue flat on rigid board.')
ops.append('BT /F1 7 Tf %.3f %.3f Td (%s) Tj ET' % (bx * MM, 3.5 * MM, text))
content = '\n'.join(ops).encode()

objs = [
    b'<< /Type /Catalog /Pages 2 0 R /ViewerPreferences << /PrintScaling /None >> >>',
    b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
    ('<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.2f %.2f] /Contents 4 0 R '
     '/Resources << /Font << /F1 5 0 R >> >> >>' % (W * MM, H * MM)).encode(),
    b'<< /Length %d >>\nstream\n' % len(content) + content + b'\nendstream',
    b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
]
out = bytearray(b'%PDF-1.4\n')
offsets = []
for i, body in enumerate(objs, 1):
    offsets.append(len(out))
    out += b'%d 0 obj\n' % i + body + b'\nendobj\n'
xref = len(out)
out += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objs) + 1)
out += b''.join(b'%010d 00000 n \n' % o for o in offsets)
out += b'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (len(objs) + 1, xref)
open(sys.argv[1], 'wb').write(out)
print('wrote', sys.argv[1])
