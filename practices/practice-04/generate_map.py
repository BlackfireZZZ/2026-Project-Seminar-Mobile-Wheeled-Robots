from pathlib import Path
import re

import numpy as np


def main():
    folder = Path(__file__).resolve().parent
    resolution = 0.01
    origin = -4.1
    side = 820
    coords = origin + (np.arange(side) + 0.5) * resolution
    x, y = np.meshgrid(coords, coords[::-1])
    picture = np.full((side, side), 254, dtype=np.uint8)
    wall = ((abs(x) >= 4) & (abs(x) <= 4.05) & (abs(y) <= 4))
    wall |= ((abs(y) >= 4) & (abs(y) <= 4.05) & (abs(x) <= 4.05))
    picture[wall] = 0
    for digit, shift in [('0', -1), ('2', 1)]:
        proto = (folder / 'protos' / f'Digit{digit}.proto').read_text()
        proto = proto.split('boundingObject')[1]
        blocks = re.findall(r'translation\s+([\d.\-]+)\s+([\d.\-]+)\s+[\d.\-]+\s+children\s*\[\s*Box\s*\{\s*size\s+([\d.]+)\s+([\d.]+)', proto)
        if not blocks:
            raise ValueError(f'No boxes in Digit{digit}')
        for px, py, width, height in blocks:
            px, py, width, height = map(float, (px, py, width, height))
            inside = (abs(x - px - shift) <= width / 2) & (abs(y - py) <= height / 2)
            picture[inside] = 0
    target = folder / 'maps' / 'numbers.pgm'
    target.write_bytes(f'P5\n{side} {side}\n255\n'.encode() + picture.tobytes())
    (folder / 'maps' / 'numbers.yaml').write_text(
        'image: numbers.pgm\nmode: trinary\nresolution: 0.01\n'
        'origin: [-4.1, -4.1, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
    )


if __name__ == '__main__':
    main()
