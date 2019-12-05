#!/usr/bin/env python
# -*- coding: utf-8 -*-
##############################################################################
# Rastercarve v0.0
#
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
FEED_RATE = 80 # in / min
PLUNGE_RATE = 30 # in / min
RAPID_RATE = 180 # in / min (used only for time estimation)
SAFE_Z = .1 # tool will start/end this high from material
TRAVERSE_Z = 2 # ending height (in)
MAX_DEPTH = .080 # full black is this many inches deep
TOOL_ANGLE = 30 # included angle of tool (we assume a V-bit). change if needed

#### Image size
DESIRED_WIDTH = 4 # desired width in inches (change this to scale image)

#### Cutting Parameters
LINE_SPACING_FACTOR = 1.0 # Vectric recommends 10-20% for wood
LINE_ANGLE = 22.5 # angle of lines across image, [0-90) degrees
LINEAR_RESOLUTION = .01 # spacing between image samples along a line (inches)

#### Image interpolation
SUPERSAMPLE = 2 # scale heightmap by this factor before cutting

#### G-Code options
LINE_NOS = True # Generate line "N"umbers (required for ShopBot)

#### Internal stuff - don't mess with this
DEG2RAD = math.pi / 180
DEPTH_TO_WIDTH = 2 * math.tan(TOOL_ANGLE / 2 * DEG2RAD) # multiply by this to get the width of a cut
LINE_WIDTH = MAX_DEPTH * DEPTH_TO_WIDTH
LINE_SPACING = LINE_SPACING_FACTOR * LINE_WIDTH # orthogonal distance between lines
OUTPUT_PPI = 1 / LINEAR_RESOLUTION # linear PPI of engraved image

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

pathlen = 0 # in
rapidlen = 0 # in
plungelen = 0 # in
movelen = 0 # in
pathtime = 0 # sec
lastpos = None

def updatePos(pos, feedrate):
    global pathlen, rapidlen, plungelen, movelen, lastpos, pathtime
    if lastpos is None:
        lastpos = pos
        return
    d = np.linalg.norm(pos - lastpos)
    pathlen += d

    # account for different types of moves separately
    if feedrate == FEED_RATE:
        movelen += d
    elif feedrate == RAPID_RATE:
        rapidlen += d
    elif feedrate == PLUNGE_RATE:
        plungelen += d

    pathtime += d / feedrate
    lastpos = pos

# reflect as needed
def transform(x, y):
    return x, -y

# we will negate the Y axis in all these
def move(x, y, z, f = FEED_RATE):
    x, y = transform(x, y)
    gcode("G1 F%d X%f Y%f Z%f" % (f, x, y, z))
    updatePos(np.array([x, y, z]), f)

def moveRapid(x, y, z):
    x, y = transform(x, y)
    gcode("G0 X%f Y%f Z%f" % (x, y, z))
    updatePos(np.array([x, y, z]), RAPID_RATE)

def moveSlow(x, y, z):
    # we don't want to transform X, Y here
    move(x, y, z, PLUNGE_RATE)
    # we also don't update position (handled by move)

def moveRapidXY(x, y):
    x, y = transform(x, y)
    gcode("G0 X%f Y%f" % (x, y))
    updatePos(np.array([x, y, lastpos[2]]), RAPID_RATE)

def moveZ(z, f = PLUNGE_RATE):
    gcode("G1 F%d Z%f" % (f, z))
    newpos = lastpos
    newpos[2] = z
    updatePos(newpos, f)

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
        print("WARNING: Engraving out of bounds! (Possible programming error, you idiot!): ", img_size, v, file=sys.stderr)

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
         not(0 < FEED_RATE) or
         not(0 < PLUNGE_RATE) or
         not(0 < SAFE_Z) or
         not(0 < TRAVERSE_Z) or
         not(0 < MAX_DEPTH) or
         not(0 < DESIRED_WIDTH) or
         not(1 <= LINE_SPACING_FACTOR) or
         not(0 < LINEAR_RESOLUTION) ):
        eprint("WARNING: Invalid parameter(s).")

    # invert and convert to grayscale
    img = ~cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2GRAY)

    orig_h, orig_w = img.shape[:2]

    img_w, img_h = img_size = DESIRED_WIDTH, DESIRED_WIDTH * (orig_h / orig_w)
    img_ppi = orig_w / img_w # should be the same for X and Y directions

    # scale up the image with interpolation
    # we want the image DPI to match our engraving DPI (which is LINEAR_RESOLUTION)
    scale_factor = SUPERSAMPLE * OUTPUT_PPI / img_ppi
    img_interp = cv2.resize(img, None, fx = scale_factor, fy = scale_factor)
    interp_ppi = img_ppi * scale_factor

    # preamble: https://www.instructables.com/id/How-to-write-G-code-basics/
    print("( Generated by rastercarve: github.com/built1n/rastercarve )")
    print("( Image name: %s )" % (filename))
    gcode("G00 G90 G80 G28 G17 G20 G40 G49\n")
    gcode("M03") # start spindle

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
        last = start + LINEAR_RESOLUTION * d

        if last[1] < 0:
            start[0] += xspace
        else:
            start[1] += yspace

        # check if we ran out the top-right corner (this needs special treatment)
        if start[0] >= img_w:
            eprint("Special case TRIGGERED")
            c = (start[0] - img_w) / d[0]
            start -= (c + LINEAR_RESOLUTION * .01) * d

        end = engraveLine(img_interp, img_size, interp_ppi, start, -d)

    moveSlow(end[0], end[1], TRAVERSE_Z)
    moveRapid(0, 0, TRAVERSE_Z)

    gcode("M05") # stop spindle

    ### Dump stats
    eprint("=== Statistics ===")
    eprint("Input resolution: %dx%dpx" % (orig_w, orig_h))
    eprint("Output dimensions: %.2f\" wide by %.2f\" tall = %.1f in^2" % (img_w, img_h, img_w * img_h))
    eprint("Max line depth: %.3f in" % (MAX_DEPTH))
    eprint("Max line width: %.3f in (%.1f deg V-bit)" % (LINE_WIDTH, TOOL_ANGLE))
    eprint("Line spacing: %.3f in (%d%% stepover)" % (LINE_SPACING, int(round(100 * LINE_SPACING_FACTOR))))
    eprint("Line angle: %.1f deg" % (LINE_ANGLE))
    eprint("Number of lines: %d" % (nlines))
    eprint("Input resolution:  %.1f PPI" % (img_ppi))
    eprint("Output resolution: %.1f PPI" % (OUTPUT_PPI))
    eprint("Scaled image by f=%.2f (%.1f PPI)" % (scale_factor, interp_ppi))
    eprint("Total toolpath length: %.1f in" % (pathlen))
    eprint(" - Rapids:  %.1f in (%.1f s)" % (rapidlen, rapidlen / (RAPID_RATE / 60)))
    eprint(" - Plunges: %.1f in (%.1f s)" % (plungelen, plungelen / (PLUNGE_RATE / 60)))
    eprint(" - Moves:   %.1f in (%.1f s)" % (movelen, movelen / (FEED_RATE / 60)))
    eprint("Feed rate: %.1f in/min" % (FEED_RATE))
    eprint("Plunge rate: %.1f in/min" % (PLUNGE_RATE))
    eprint("Total machining time: %.1f sec" % (pathlen / (FEED_RATE / 60)))

def main():
    if len(sys.argv) != 2:
        eprint("Usage: rastercarve.py IMAGE")
        return
    doEngrave(sys.argv[1])

if __name__=="__main__":
    main()
