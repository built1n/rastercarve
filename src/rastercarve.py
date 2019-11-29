#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) 2019 Franklin Wei
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
# KIND, either express or implied.

"""
rastercarve: a raster engraving G-code generator

Usage: rastercarve.py IMAGE

This program outputs G-code to engrave a bitmap image on a 3-axis
milling machine.
"""

import cv2
import math
import numpy as np
import sys

#### Machine configuration
FEEDRATE = 100 # in / min
PLUNGE_RATE = 10 # in / min
SAFE_Z = .2 # tool will start/end this high from material
TRAVERSE_Z = 1
MAX_DEPTH = .080 # full black is this many inches deep
TOOL_ANGLE = 60 # included angle of tool (we assume a V-bit). change if needed

#### Image size
DESIRED_WIDTH = 10 # desired width in inches (change this to scale image)

#### Cutting Parameters
LINE_SPACING_FACTOR = 1.1 # Vectric recommends 10-20% for wood
LINE_ANGLE = 22.5 # angle of lines across image, [0-90) degrees
LINEAR_RESOLUTION = .01 # spacing between image samples along a line (inches)

#### Image interpolation
SUPERSAMPLE = 5 # interpolate the image by this factor (caution: this scales the image by the square of its value)

#### G-Code options
LINE_NOS = True # Generate line "N"umbers

#### Internal stuff - don't mess with this
DEG2RAD = math.pi / 180
DEPTH_TO_WIDTH = 2 * math.tan(TOOL_ANGLE / 2 * DEG2RAD) # multiply by this to get the width of a cut
LINE_WIDTH = MAX_DEPTH * DEPTH_TO_WIDTH
LINE_SPACING = LINE_SPACING_FACTOR * LINE_WIDTH # orthogonal distance between lines

# floating-point range
def frange(x, y, jump):
    while x < y:
        yield x
        x += jump

def eprint(s):
    print(s, file=sys.stderr)

line = 1
def gcode(s):
    global line
    print(("N%d %s" % (line, s)) if LINE_NOS else s)
    line += 1

pathlen = 0
lastpos = None

def updatePos(pos):
    global pathlen, lastpos
    if lastpos is None:
        lastpos = pos
        return
    pathlen += np.linalg.norm(pos - lastpos)
    lastpos = pos

# reflect as needed
def transform(x, y):
    return x, -y

# we will negate the Y axis in all these
def move(x, y, z, f = FEEDRATE):
    x, y = transform(x, y)
    gcode("G1 F%d X%f Y%f Z%f" % (f, x, y, z))
    updatePos(np.array([x, y, z]))

def moveRapid(x, y, z):
    x, y = transform(x, y)
    gcode("G0 X%f Y%f Z%f" % (x, y, z))
    updatePos(np.array([x, y, z]))

def moveSlow(x, y, z):
    # we don't want to transform X, Y here
    move(x, y, z, PLUNGE_RATE)
    updatePos(np.array([x, y, z]))

def moveRapidXY(x, y):
    x, y = transform(x, y)
    gcode("G0 X%f Y%f" % (x, y))
    updatePos(np.array([x, y, lastpos[2]]))

def moveZ(z, f = PLUNGE_RATE):
    gcode("G1 F%d Z%f" % (f, z))
    newpos = lastpos
    newpos[2] = z
    updatePos(newpos)

def getPix(image, x, y):
    # clamp
    x = max(0, min(int(x), image.shape[1]-1))
    y = max(0, min(int(y), image.shape[0]-1))

    return image[y, x]

# return how deep to cut given a pixel value
def getDepth(pix):
    # may want to do gamma mapping
    return -float(pix) / 256 * MAX_DEPTH

def inBounds(img_size, x):
    return 0 <= x[0] and x[0] < img_size[0] and 0 <= x[1] and x[1] < img_size[1]

# Engrave one line across the image. start and d are vectors in the
# output space representing the start point and direction of
# machining, respectively. start should be on the border of the image,
# and d should point INTO the image.
def engraveLine(img_interp, img_size, ppi, start, d, step = LINEAR_RESOLUTION):
    v = start
    d = d / np.linalg.norm(d)

    if not inBounds(img_size, v):
        print("NOT IN BOUNDS (PROGRAMMING ERROR): ", img_size, v, file=sys.stderr)

    moveZ(SAFE_Z)
    moveRapidXY(v[0], v[1])

    first = True

    while inBounds(img_size, v):
        img_x = int(round(v[0] * ppi))
        img_y = int(round(v[1] * ppi))
        x, y = v
        depth = getDepth(getPix(img_interp, img_x, img_y))
        if not first:
            move(x, y, depth)
        else:
            first = False
            moveSlow(x, y, depth)

        v += step * d
    # return last engraved point
    return v - step * d

def doEngrave(filename):
    # check parameter sanity
    if ( not(0 <= LINE_ANGLE < 90) or
         not(0 < TOOL_ANGLE < 180) or
         not(0 < FEEDRATE) or
         not(0 < PLUNGE_RATE) or
         not(0 < SAFE_Z) or
         not(0 < TRAVERSE_Z) or
         not(0 < MAX_DEPTH) or
         not(0 < DESIRED_WIDTH) or
         not(1 <= LINE_SPACING_FACTOR) or
         not(0 < LINEAR_RESOLUTION) or
         not(1 <= SUPERSAMPLE) ):
        eprint("WARNING: Invalid parameter(s).")

    # invert and convert to grayscale
    img = ~cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2GRAY)

    orig_h, orig_w = img.shape[:2]

    img_w, img_h = img_size = DESIRED_WIDTH, DESIRED_WIDTH * (orig_h / orig_w)
    img_ppi = orig_w / img_w # should be the same for X and Y directions

    # scale up the image with interpolation
    img_interp = cv2.resize(img, None, fx = SUPERSAMPLE, fy = SUPERSAMPLE)
    interp_ppi = img_ppi * SUPERSAMPLE

    # preamble: https://www.instructables.com/id/How-to-write-G-code-basics/
    print("( Generated by rastercarve: github.com/built1n/rastercarve )")
    print("( Image name: %s )" % (filename))
    gcode("G00 G90 G80 G28 G17 G20 G40 G49\n")

    d = np.array([math.cos(LINE_ANGLE * DEG2RAD),
                  -math.sin(LINE_ANGLE * DEG2RAD)])

    max_y = img_h + img_w * -d[1] / d[0] # highest Y we'll loop to
    yspace = LINE_SPACING / math.cos(LINE_ANGLE * DEG2RAD) # vertical spacing between lines
    xspace = LINE_SPACING / math.sin(LINE_ANGLE * DEG2RAD) if LINE_ANGLE != 0 else 0 # horizontal space

    nlines = round(max_y / yspace)

    ### Generate toolpath
    moveRapid(0, 0, SAFE_Z)
    end = None

    for y in frange(0, max_y - yspace, yspace * 2):
        start = np.array([0, y]).astype('float64')

        # start some vectors on the bottom edge of the image
        if d[1] != 0:
            c = (img_h - y) / d[1] # solve (start + cd)_y = h for c
            if c >= 0:
                start += (c + LINEAR_RESOLUTION) * d

        start = engraveLine(img_interp, img_size, interp_ppi, start, d)

        # now engrave the other direction
        # we just need to flip d and move start over

        # see which side of the image the last line ran out on (either top or right side)
        if (start + LINEAR_RESOLUTION * d)[1] < 0:
            start[0] += xspace
        else:
            start[1] += yspace

        end = engraveLine(img_interp, img_size, interp_ppi, start, -d)

    moveSlow(end[0], end[1], TRAVERSE_Z)
    moveRapid(0, 0, TRAVERSE_Z)

    ### Dump stats
    eprint("=== Statistics ===")
    eprint("Image dimensions: %.2f\" wide by %.2f\" tall = %.1f in^2 (%.1f PPI)" % (img_w, img_h, img_w * img_h, img_ppi))
    eprint("Max line depth: %.3f in" % (MAX_DEPTH))
    eprint("Max line width: %.3f in (%.1f deg V-bit)" % (LINE_WIDTH, TOOL_ANGLE))
    eprint("Line spacing: %.3f in (%d%%)" % (LINE_SPACING, int(round(100 * LINE_SPACING_FACTOR))))
    eprint("Line angle: %.1f deg" % (LINE_ANGLE))
    eprint("Number of lines: %d" % (nlines))
    eprint("Interpolated image by f=%.1f (%.1f PPI)" % (SUPERSAMPLE, interp_ppi))
    eprint("Toolpath length: %.1f in" % (pathlen))
    eprint("Feed rate: %.1f in/min" % (FEEDRATE))
    eprint("Plunge rate: %.1f in/min" % (PLUNGE_RATE))
    eprint("Approximate machining time: %.1f sec" % (pathlen / (FEEDRATE / 60)))

def main():
    if len(sys.argv) != 2:
        eprint("Usage: rastercarve.py IMAGE")
        return
    doEngrave(sys.argv[1])

if __name__=="__main__":
    main()
