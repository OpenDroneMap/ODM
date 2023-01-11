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

import rasterio
import numpy as np
#from PIL import Image, ImageDraw
import math

# Based off of the great writeup, demo and code at:
# http://codeincomplete.com/posts/2011/5/7/bin_packing/

class Block():
    """A rectangular block, to be packed"""
    def __init__(self, w, h, data=None, padding=0):
        self.w = w
        self.h = h
        self.x = None
        self.y = None
        self.fit = None
        self.data = data
        self.padding = padding # not implemented yet

    def __str__(self):
        return "({x},{y}) ({w}x{h}): {data}".format(
            x=self.x,y=self.y, w=self.w,h=self.h, data=self.data)


class _BlockNode():
    """A BlockPacker node"""
    def __init__(self, x, y, w, h, used=False, right=None, down=None):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.used = used
        self.right = right
        self.down = down

    def __repr__(self):
        return "({x},{y}) ({w}x{h})".format(x=self.x,y=self.y,w=self.w,h=self.h)


class BlockPacker():
    """Packs blocks of varying sizes into a single, larger block"""
    def __init__(self):
        self.root = None

    def fit(self, blocks):
        nblocks = len(blocks)
        w = blocks[0].w# if nblocks > 0 else 0
        h = blocks[0].h# if nblocks > 0 else 0

        self.root = _BlockNode(0,0, w,h)

        for block in blocks:
            node = self.find_node(self.root, block.w, block.h)
            if node:
                # print("split")
                node_fit = self.split_node(node, block.w, block.h)
                block.x = node_fit.x
                block.y = node_fit.y
            else:
                # print("grow")
                node_fit = self.grow_node(block.w, block.h)
                block.x = node_fit.x
                block.y = node_fit.y

    def find_node(self, root, w, h):
        if root.used:
            # raise Exception("used")
            node = self.find_node(root.right, w, h)
            if node:
                return node
            return self.find_node(root.down, w, h)
        elif w <= root.w and h <= root.h:
            return root
        else:
            return None

    def split_node(self, node, w, h):
        node.used = True
        node.down = _BlockNode(
            node.x, node.y + h,
            node.w, node.h - h
        )
        node.right = _BlockNode(
            node.x + w, node.y,
            node.w - w, h
        )
        return node

    def grow_node(self, w, h):
        can_grow_down = w <= self.root.w
        can_grow_right = h <= self.root.h

        # try to keep the packing square
        should_grow_right = can_grow_right and self.root.h >= (self.root.w + w)
        should_grow_down = can_grow_down and self.root.w >= (self.root.h + h)

        if should_grow_right:
            return self.grow_right(w, h)
        elif should_grow_down:
            return self.grow_down(w, h)
        elif can_grow_right:
            return self.grow_right(w, h)
        elif can_grow_down:
            return self.grow_down(w, h)
        else:
            raise Exception("no valid expansion avaliable!")

    def grow_right(self, w, h):
        old_root = self.root
        self.root = _BlockNode(
            0, 0,
            old_root.w + w, old_root.h,
            down=old_root,
            right=_BlockNode(self.root.w, 0, w, self.root.h),
            used=True
        )

        node = self.find_node(self.root, w, h)
        if node:
            return self.split_node(node, w, h)
        else:
            return None

    def grow_down(self, w, h):
        old_root = self.root
        self.root = _BlockNode(
            0, 0,
            old_root.w, old_root.h + h,
            down=_BlockNode(0, self.root.h, self.root.w, h),
            right=old_root,
            used=True
        )

        node = self.find_node(self.root, w, h)
        if node:
            return self.split_node(node, w, h)
        else:
            return None


def crop_by_extents(image, extent):
    if min(extent.min_x,extent.min_y) < 0 or max(extent.max_x,extent.max_y) > 1:
        print("\tWARNING! UV Coordinates lying outside of [0:1] space!")
    
    _, h, w = image.shape
    minx = max(math.floor(extent.min_x*w), 0)
    miny = max(math.floor(extent.min_y*h), 0)
    maxx = min(math.ceil(extent.max_x*w), w)
    maxy = min(math.ceil(extent.max_y*h), h)

    image = image[:, miny:maxy, minx:maxx]
    delta_w = maxx - minx
    delta_h = maxy - miny

    # offset from origin x, y, horizontal scale, vertical scale
    changes = (minx, miny, delta_w / w, delta_h / h)

    return (image, changes)

def pack(obj, background=(0,0,0,0), format="PNG", extents=None):
    blocks = []
    image_name_map = {}
    profile = None

    for mat in obj['materials']:
        filename = obj['materials'][mat]

        with rasterio.open(filename, 'r') as f:
            profile = f.profile
            image = f.read()

        image = np.flip(image, axis=1)

        changes = None
        if extents and extents[mat]:
            image, changes = crop_by_extents(image, extents[mat])
            
        image_name_map[filename] = image
        _, h, w = image.shape

        # using filename so we can pass back UV info without storing it in image
        blocks.append(Block(w, h, data=(filename, mat, changes)))

    # sort by width, descending (widest first)
    blocks.sort(key=lambda block: -block.w)

    packer = BlockPacker()
    packer.fit(blocks)

    # output_image = Image.new("RGBA", (packer.root.w, packer.root.h))
    output_image = np.zeros((profile['count'], packer.root.h, packer.root.w), dtype=profile['dtype'])

    uv_changes = {}
    for block in blocks:
        fname, mat, changes = block.data
        image = image_name_map[fname]
        _, im_h, im_w = image.shape

        uv_changes[mat] = {
            "offset": (
                # should be in [0, 1] range
                (block.x - (changes[0] if changes else 0))/output_image.shape[2],
                # UV origin is bottom left, PIL assumes top left!
                (block.y - (changes[1] if changes else 0))/output_image.shape[1]
            ),

            "aspect": (
                ((1/changes[2]) if changes else 1) * (im_w/output_image.shape[2]),
                ((1/changes[3]) if changes else 1) * (im_h/output_image.shape[1])
            ),
        }

        output_image[:, block.y:block.y + im_h, block.x:block.x + im_w] = image
    output_image = np.flip(output_image, axis=1)

    return output_image, uv_changes, profile
