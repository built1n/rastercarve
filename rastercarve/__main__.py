#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Rastercarve
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

import math
import sys

from rastercarve import __version__

import argparse
import cv2 # image scaling
import numpy as np # a little vector stuff
from tqdm import tqdm # progress bar

glob_args = None

##### Default parameters
#### Machine configuration
DEF_FEED_RATE = 100 # in / min
DEF_PLUNGE_RATE = 30 # in / min
DEF_RAPID_RATE = 240 # in / min (used only for time estimation)
DEF_SAFE_Z = .1 # tool will start/end this high from material
DEF_TRAVERSE_Z = 2 # ending height (in)
DEF_MAX_DEPTH = .080 # full black is this many inches deep
DEF_TOOL_ANGLE = 30 # included angle of tool (we assume a V-bit). change if needed

#### Cutting parameters
DEF_STEPOVER = 110
DEF_LINE_ANGLE = 22.5 # angle of lines across image, [0-90) degrees
DEF_LINEAR_RESOLUTION = .01 # spacing between image samples along a line (inches)

#### Image interpolation
SUPERSAMPLE = 2 # scale heightmap by this factor before cutting

#### Internal stuff - don't mess with this
DEG2RAD = math.pi / 180

#### Parameter constraints
CONSTRAINTS = [
    '0 <= glob_args.line_angle < 90',
    '0 < glob_args.tool_angle < 180',
    '0 < glob_args.feed_rate',
    '0 < glob_args.plunge_rate',
    '0 < glob_args.safe_z',
    '0 < glob_args.traverse_z',
    '0 < glob_args.max_depth',
    '100 <= glob_args.stepover',
    '0 < glob_args.linear_resolution',
    '(hasattr(glob_args, "height") and glob_args.height > 0) or (hasattr(glob_args, "width") and glob_args.width > 0)'
]

# floating-point range
def frange(x, y, jump):
    # this is hugely inefficient but gives us nice progress bar
    # stats... what matters more?
    ret = []
    while x < y:
        ret.append(x)
        x += jump
    return ret

def eprint(s):
    if not hasattr(glob_args, 'quiet'):
        print(s, file=sys.stderr)

debug_msgs = 0
def debug(str):
    global debug_msgs
    if hasattr(glob_args, 'debug'):
        eprint(str)
    debug_msgs += 1

line = 1
gcodebuf = ""
def gcode(s):
    global line
    print((s if hasattr(glob_args, 'suppress_linenos') else "N%d %s" % (line, s)) )
    line += 1

pathlen = 0 # in
rapidlen = 0 # in
plungelen = 0 # in
movelen = 0 # in
pathtime = 0 # sec
lastpos = None

# movetype: 1 = feed, 2 = rapid, 3 = plunge
def updatePos(pos, feedrate, movetype):
    global pathlen, rapidlen, plungelen, movelen, lastpos, pathtime
    if lastpos is None:
        lastpos = pos
        return
    d = np.linalg.norm(pos - lastpos)
    pathlen += d

    # account for different types of moves separately
    if movetype == 1:
        movelen += d
    elif movetype == 2:
        rapidlen += d
    elif movetype == 3:
        plungelen += d

    pathtime += d / (feedrate / 60)
    lastpos = pos

# reflect as needed
def transform(x, y):
    return x, -y

# we will negate the Y axis in all these
def move(x, y, z, f):
    x, y = transform(x, y)
    gcode("G1 F%d X%f Y%f Z%f" % (f, x, y, z))
    updatePos(np.array([x, y, z]), f, 1)

def moveRapid(x, y, z):
    x, y = transform(x, y)
    gcode("G0 X%f Y%f Z%f" % (x, y, z))
    updatePos(np.array([x, y, z]), glob_args.rapid_rate, 2)

def moveSlow(x, y, z):
    x, y = transform(x, y)
    f = glob_args.plunge_rate
    gcode("G1 F%d X%f Y%f Z%f" % (f, x, y, z))
    updatePos(np.array([x, y, z]), f, 3)

def moveRapidXY(x, y):
    x, y = transform(x, y)
    gcode("G0 X%f Y%f" % (x, y))
    updatePos(np.array([x, y, lastpos[2]]), glob_args.rapid_rate, 2)

def moveZ(z, f):
    gcode("G1 F%d Z%f" % (f, z))
    newpos = lastpos
    newpos[2] = z
    updatePos(newpos, f, 2)

def getPix(image, x, y):
    # clamp
    x = max(0, min(int(x), image.shape[1]-1))
    y = max(0, min(int(y), image.shape[0]-1))

    return image[y, x]

# return how deep to cut given a pixel value
def getDepth(pix):
    # may want to do gamma mapping
    return -float(pix) / 256 * glob_args.max_depth

def inBounds(img_size, x):
    return 0 <= x[0] and x[0] < img_size[0] and 0 <= x[1] and x[1] < img_size[1]

# Engrave one line across the image. start and d are vectors in the
# output space representing the start point and direction of
# machining, respectively. start should be on the border of the image,
# and d should point INTO the image.
def engraveLine(img_interp, img_size, ppi, start, d, step):
    v = start
    d = d / np.linalg.norm(d)

    if not inBounds(img_size, v):
        debug("Refusing to engrave out of bounds. (Possible programming error, you idiot!): %s, %s" % (v, img_size))
        return start

    moveZ(glob_args.safe_z, glob_args.plunge_rate)
    moveRapidXY(v[0], v[1])

    first = True

    while inBounds(img_size, v):
        img_x = int(round(v[0] * ppi))
        img_y = int(round(v[1] * ppi))
        x, y = v
        depth = getDepth(getPix(img_interp, img_x, img_y))
        if not first:
            move(x, y, depth, glob_args.feed_rate)
        else:
            first = False
            moveSlow(x, y, depth)

        v += step * d
    # return last engraved point
    return v - step * d

def checkCondition(cond):
    success = eval(cond)
    if not success:
        eprint("ERROR: Invalid parameter: %s" % cond)
    return success

def doEngrave():
    # check parameter sanity
    for c in CONSTRAINTS:
        if not checkCondition(c):
            eprint("Refusing to generate G-code.")
            return

    # invert and convert to grayscale
    img = ~cv2.cvtColor(cv2.imread(glob_args.filename), cv2.COLOR_BGR2GRAY)

    orig_h, orig_w = img.shape[:2]

    img_w, img_h = img_size = (glob_args.width, glob_args.width * (orig_h / orig_w)) if hasattr(glob_args, 'width') else (glob_args.height * (orig_w / orig_h), glob_args.height)
    img_ppi = orig_w / img_w # should be the same for X and Y directions

    depth2width = 2 * math.tan(glob_args.tool_angle / 2 * DEG2RAD) # multiply by this to get the width of a cut
    line_width = glob_args.max_depth * depth2width
    line_spacing = glob_args.stepover * line_width / 100.0 # orthogonal distance between lines
    output_ppi = 1 / glob_args.linear_resolution # linear PPI of engraved image

    # scale up the image with interpolation
    # we want the image DPI to match our engraving DPI (which is glob_args.linear_resolution)
    scale_factor = SUPERSAMPLE * output_ppi / img_ppi
    img_interp = cv2.resize(img, None, fx = scale_factor, fy = scale_factor)
    interp_ppi = img_ppi * scale_factor

    # preamble: https://www.instructables.com/id/How-to-write-G-code-basics/
    print("( Generated by rastercarve: github.com/built1n/rastercarve )")
    print("( Image name: %s )" % (glob_args.filename))
    gcode("G00 G90 G80 G28 G17 G20 G40 G49\n")
    gcode("M03") # start spindle

    d = np.array([math.cos(glob_args.line_angle * DEG2RAD),
                  -math.sin(glob_args.line_angle * DEG2RAD)])

    max_y = img_h + img_w * -d[1] / d[0] # highest Y we'll loop to
    yspace = line_spacing / math.cos(glob_args.line_angle * DEG2RAD) # vertical spacing between lines
    xspace = line_spacing / math.sin(glob_args.line_angle * DEG2RAD) if glob_args.line_angle != 0 else 0 # horizontal space

    nlines = round(max_y / yspace)

    ### Generate toolpath
    moveRapid(0, 0, glob_args.safe_z)
    end = None

    for y in tqdm(frange(0, max_y - yspace, yspace * 2),
                  desc='Generating G-code',
                  unit=' lines',
                  unit_scale = 2,
                  disable = hasattr(glob_args, 'quiet')): # we engrave two lines per loop
        start = np.array([0, y]).astype('float64')

        # start some vectors on the bottom edge of the image
        if d[1] != 0:
            c = (img_h - y) / d[1] # solve (start + cd)_y = h for c
            if c >= 0:
                start += (c + glob_args.linear_resolution) * d

        start = engraveLine(img_interp, img_size, interp_ppi, start, d, glob_args.linear_resolution)

        # now engrave the other direction
        # we just need to flip d and move start over

        # see which side of the image the last line ran out on (either top or right side)
        last = start + glob_args.linear_resolution * d

        if last[1] < 0:
            start[0] += xspace
        else:
            start[1] += yspace

        # check if we ran out the top-right corner (this needs special treatment)
        if start[0] >= img_w:
            debug("Special case TRIGGERED")
            c = (start[0] - img_w) / d[0]
            start -= (c + glob_args.linear_resolution * .01) * d

        end = engraveLine(img_interp, img_size, interp_ppi, start, -d, glob_args.linear_resolution)

    moveSlow(end[0], end[1], glob_args.traverse_z)
    moveRapid(0, 0, glob_args.traverse_z)

    gcode("M05") # stop spindle

    ### Dump stats
    eprint("=== Statistics ===")
    eprint("Input resolution: %dx%d px" % (orig_w, orig_h))
    eprint("Output dimensions: %.2f\" wide by %.2f\" tall = %.1f in^2" % (img_w, img_h, img_w * img_h))
    eprint("Max line depth: %.3f in" % (glob_args.max_depth))
    eprint("Max line width: %.3f in (%.1f deg V-bit)" % (line_width, glob_args.tool_angle))
    eprint("Line spacing: %.3f in (%d%% stepover)" % (line_spacing, int(round(glob_args.stepover))))
    eprint("Line angle: %.1f deg" % (glob_args.line_angle))
    eprint("Number of lines: %d" % (nlines))
    eprint("Input resolution:  %.1f PPI" % (img_ppi))
    eprint("Output resolution: %.1f PPI" % (output_ppi))
    eprint("Scaled image by f=%.2f (%.1f PPI)" % (scale_factor, interp_ppi))
    eprint("Total toolpath length: %.1f in" % (pathlen))
    eprint(" - Rapids:  %.1f in (%.1f s)" % (rapidlen, rapidlen / (glob_args.rapid_rate / 60)))
    eprint(" - Plunges: %.1f in (%.1f s)" % (plungelen, plungelen / (glob_args.plunge_rate / 60)))
    eprint(" - Moves:   %.1f in (%.1f s)" % (movelen, movelen / (glob_args.feed_rate / 60)))
    eprint("Feed rate: %.1f in/min" % (glob_args.feed_rate))
    eprint("Plunge rate: %.1f in/min" % (glob_args.plunge_rate))
    eprint("Estimated machining time: %.1f sec" % (pathtime))

    if not hasattr(glob_args, 'debug') and debug_msgs > 0:
        eprint("%d suppressed debug message(s)." % (debug_msgs))

def main():
    parser = argparse.ArgumentParser(prog='rastercarve',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     description='Generate G-code to engrave raster images.',
                                     epilog='Defaults are usually safe to leave unchanged.')
    parser.add_argument('filename', help='input image (any OpenCV-supported format)')

    dim_group = parser.add_argument_group('output dimensions', 'Exactly one required.')
    mutex_group = dim_group.add_mutually_exclusive_group(required=True)
    mutex_group.add_argument('--width', help='output width (in)', action='store', dest='width', type=float, default=argparse.SUPPRESS)
    mutex_group.add_argument('--height', help='output height (in)', action='store', dest='height', type=float, default=argparse.SUPPRESS)

    mach_group = parser.add_argument_group('machine configuration')
    mach_group.add_argument('-f', help='engraving feed rate (in/min)', action='store', dest='feed_rate', default=DEF_FEED_RATE, type=float)
    mach_group.add_argument('-p', help='engraving plunge rate (in/min)', action='store', dest='plunge_rate', default=DEF_PLUNGE_RATE, type=float)
    mach_group.add_argument('--rapid', help='rapid traverse rate (for time estimation only)', action='store', dest='rapid_rate', default=DEF_RAPID_RATE, type=float)
    mach_group.add_argument('-z', help='rapid Z traverse height (in)', action='store', dest='safe_z', default=DEF_SAFE_Z, type=float)
    mach_group.add_argument('--end-z', help='Z height of final traverse (in)', action='store', dest='traverse_z', default=DEF_TRAVERSE_Z, type=float)
    mach_group.add_argument('-d', help='maximum engraving depth (in)', action='store', dest='max_depth', default=DEF_MAX_DEPTH, type=float)
    mach_group.add_argument('-t', help='included angle of tool (deg)', action='store', dest='tool_angle', default=DEF_TOOL_ANGLE, type=float)

    cut_group = parser.add_argument_group('engraving parameters')
    cut_group.add_argument('-a', help='angle of grooves from horizontal (deg)', action='store', dest='line_angle', default=DEF_LINE_ANGLE, type=float)
    cut_group.add_argument('-s', help='stepover percentage (affects spacing between lines)', action='store', dest='stepover', default=DEF_STEPOVER, type=float)
    cut_group.add_argument('-r', help='distance between successive G-code points (in)', action='store', dest='linear_resolution', default=DEF_LINEAR_RESOLUTION, type=float)

    gcode_group = parser.add_argument_group('G-code parameters')
    gcode_group.add_argument('--no-line-numbers', help='suppress G-code line numbers (dangerous on ShopBot!)', action='store_true', dest='suppress_linenos', default=argparse.SUPPRESS)

    parser.add_argument('--debug', help='print debug messages', action='store_true', dest='debug', default=argparse.SUPPRESS)
    parser.add_argument('-q', help='disable progress and statistics', action='store_true', dest='quiet', default=argparse.SUPPRESS)
    parser.add_argument('--version', help="show program's version number and exit", action='version', version=__version__)

    global glob_args
    glob_args = parser.parse_args()
    debug(glob_args)

    doEngrave()

if __name__=="__main__":
    main()
