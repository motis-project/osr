#!/usr/bin/env python3
#-*- coding: utf8 -*-

import sys

def main():
    (
        Tile(tl=Point(8.656339, 49.883796, ), br=Point(8.657444, 49.882826, ), size=Pos(3, 2), default=-100)
        .set(Pos(2, 1), -95)
        .print()
        .save('elevations_1')
    )

class Point(object):

    x: float
    y: float

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

class Pos(object):

    x: int
    y: int

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def __str__(self):
        return f'({self.x}, {self.y})'

class Grid(object):

    rows: list[list[int]]

    def __init__(self, width: int, height: int, default: int):
        if not (0 < width and 0 < height):
            print(f'Error: Invalid grid size {pos}', file=sys.stderr)
            sys.exit(1)
        self.rows = [[default for w in range(width)] for h in range(height)]

    def set(self, pos: Pos, value: int):
        if not (0 <= pos.x <= self.width() and 0 <= pos.y <= self.height()):
            print(f'Error: Position {pos} is out of bound', file=sys.stderr)
            sys.exit(1)
        self.rows[pos.y][pos.x] = value

    def width(self):
        return len(self.rows[0]) - 1

    def height(self):
        return len(self.rows) - 1

    def save(self, path: str):
        with open(path, 'wb') as f:
            for row in self.rows:
                for value in row:
                    f.write(value.to_bytes(length=2, byteorder='little', signed=True))

    def print(self):
        for row in self.rows:
            for entry in row:
                print(f'{entry:>5} ', end='')
            print()

class Tile(object):

    tl: Point
    br: Point
    grid: Grid

    def __init__(self, tl: Point, br: Point, size: Pos, default: int = 0):
        self.tl = tl
        self.br = br
        self.grid = Grid(width=size.x, height=size.y, default=default)

    def set(self, pos: Pos, value: int):
        self.grid.set(pos, value)
        return self

    def save(self, path: str):
        width = self.grid.width()
        height = self.grid.height()
        with open(path + '.hdr', 'w') as f:
            f.write(f'''
BYTEORDER      I
LAYOUT         BIL
NROWS          {height + 1}
NCOLS          {width + 1}
NBANDS         1
NBITS          16
BANDROWBYTES   {2 * (width + 1)}
TOTALROWBYTES  {2 * (width + 1)}
PIXELTYPE      SIGNEDINT
ULXMAP         {self.tl.x}
ULYMAP         {self.tl.y}
XDIM           {(self.br.x - self.tl.x) / width}
YDIM           {(self.tl.y - self.br.y) / height}
NODATA         -32767
''')
        self.grid.save(path + '.bil')
        return self

    def print(self):
        self.grid.print()
        return self

if __name__ == '__main__':
    main()
