#! /usr/bin/python

# The MIT License (MIT)

# Copyright (c) 2015 Luke Gaynor

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

class AABB():
    def __init__(self, min_x=None, min_y=None, max_x=None, max_y=None):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y

        self.to_tile = False # TODO: remove?

    def add(self, x,y):
        self.min_x = min(self.min_x, x) if self.min_x is not None else x
        self.min_y = min(self.min_y, y) if self.min_y is not None else y
        self.max_x = max(self.max_x, x) if self.max_x is not None else x
        self.max_y = max(self.max_y, y) if self.max_y is not None else y

    def uv_wrap(self):
        return (self.max_x - self.min_x, self.max_y - self.min_y)

    def tiling(self):
        if self.min_x and self.max_x and self.min_y and self.max_y:
            if self.min_x < 0 or self.min_y < 0 or self.max_x > 1 or self.max_y > 1:
                return (self.max_x - self.min_x, self.max_y - self.min_y)
        return None

    def __repr__(self):
        return "({},{}) ({},{})".format(
            self.min_x,
            self.min_y,
            self.max_x,
            self.max_y
        )