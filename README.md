# RasterCarve

[![PyPI version](https://badge.fury.io/py/rastercarve.svg)](https://badge.fury.io/py/rastercarve) [![PyPI license](https://img.shields.io/pypi/l/rastercarve.svg)](https://pypi.python.org/pypi/rastercarve/) [![PyPI status](https://img.shields.io/pypi/status/rastercarve.svg)](https://pypi.python.org/pypi/rastercarve/)

This is a little Python script I wrote to generate G-code toolpaths to
engrave raster images.

A hosted version of the script is available at
https://rastercarve.live
([Github](https://github.com/built1n/rastercarve-live)). There is also
a standalone custom G-code previewer available at
https://github.com/built1n/rastercarve-preview.

It takes bitmap images and produces commands (G-code) for a CNC
machine to engrave that image onto a piece of material. For the
uninitiated, a CNC machine is essentially a robotic carving machine --
think *robot drill*: you 1) put in a piece of wood/foam/aluminum
stock; 2) program the machine; and 3) out comes a finished piece with
the right patterns cut into it.

This program comes in during step 2 -- it takes an image and outputs
the right sequence of commands for your machine to engrave it. This is
not the first program that can do this, but existing solutions are
unsuitable due to their high cost.

Examples below:

<img src="https://www.fwei.tk/blog/d-day.jpg" width="100%">
<img src="https://www.fwei.tk/blog/baby-yoda.png" width="100%">

# Installation

`$ pip install rastercarve`

Running straight from the source tree works fine, too:

`$ python -m rastercarve -h`

# Usage

```
$ rastercarve --width 10 examples/test.png > out.nc
Generating G-code: 100%|██████████████████| 278/278 [00:04<00:00, 57.10 lines/s]
=== Statistics ===
Input resolution: 512x512 px
Output dimensions: 10.00" wide by 10.00" tall = 100.0 in^2
Max line depth: 0.080 in
Max line width: 0.043 in (30.0 deg V-bit)
Line spacing: 0.047 in (110% stepover)
Line angle: 22.5 deg
Number of lines: 277
Input resolution:  51.2 PPI
Output resolution: 100.0 PPI
Scaled image by f=3.91 (200.0 PPI)
Total toolpath length: 2202.6 in
 - Rapids:  34.6 in (8.6 s)
 - Plunges: 29.8 in (59.6 s)
 - Moves:   2138.2 in (1282.9 s)
Feed rate: 100.0 in/min
Plunge rate: 30.0 in/min
Estimated machining time: 1351.2 sec
1 suppressed debug message(s).
```

This command generates G-code to engrave `examples/test.png` into an
piece of material 10 inches wide. Exactly one of the `--width` or
`--height` parameters must be specified on the command line; the other
will be calculated automatically.

The engraving parameters can be safely left at their defaults, though
fine-tuning is possible depending on material and machine
characteristics.

The output G-code will be piped to `out.nc`, which any CNC machine
should accept as input.

# Machining Process

With the toolpath generated, it is time to run the job. Presumably you
know the specifics of your particular machine, so I'll only outline
the high-level steps here:

1. Load the right tool. An engraving bit is best, though ordinary
V-bits give acceptable results. Make sure that the tool angle matches
that used to generate the toolpath (30 degrees is the default --
change this if needed).

2. Load the material. MDF seems to work best; plywood and ordinary
lumber are too prone to chipping. Plastics have a tendency to melt and
stick to the bit.

3. Zero X and Y axes at the top left corner of the eventual image
location. Double check that the bottom right corner is in bounds.

4. Zero the Z axis to the top surface of the material.

5. Load and run the toolpath. The engraving will begin in the top
right corner and work its way down to the bottom right in a serpentine
fashion.

The program's output has been tested on a ShopBot Desktop MAX, which
produced the results shown earlier.

# Advanced

```
usage: rastercarve [-h] (--width WIDTH | --height HEIGHT) [-f FEED_RATE]
                   [-p PLUNGE_RATE] [--rapid RAPID_RATE] [-z SAFE_Z]
                   [--end-z TRAVERSE_Z] [-t TOOL_ANGLE] [-d MAX_DEPTH]
                   [-a LINE_ANGLE] [-s STEPOVER] [-r LINEAR_RESOLUTION]
                   [--dots] [--no-line-numbers]
                   [--preamble PREAMBLE | --preamble-file PREAMBLE_FILE]
                   [--epilogue EPILOGUE | --epilogue-file EPILOGUE_FILE]
                   [--json JSON_DEST] [--debug] [-q] [--version]
                   filename

Generate G-code to engrave raster images.

positional arguments:
  filename              input image (any OpenCV-supported format)

optional arguments:
  -h, --help            show this help message and exit
  --json JSON_DEST      dump statistics in JSON format
  --debug               print debug messages
  -q                    disable progress and statistics
  --version             show program's version number and exit

output dimensions:
  Exactly one required. Image will be scaled while maintaining aspect ratio.

  --width WIDTH         output width (in)
  --height HEIGHT       output height (in)

machine configuration:
  -f FEED_RATE          engraving feed rate (in/min) (default: 100)
  -p PLUNGE_RATE        engraving plunge rate (in/min) (default: 30)
  --rapid RAPID_RATE    rapid traverse rate (for time estimation only)
                        (default: 240)
  -z SAFE_Z             rapid traverse height (in) (default: 0.1)
  --end-z TRAVERSE_Z    Z height of final traverse (in) (default: 2)
  -t TOOL_ANGLE         included angle of tool (deg) (default: 30)

engraving parameters:
  -d MAX_DEPTH          maximum engraving depth (in) (default: 0.08)
  -a LINE_ANGLE         angle of grooves from horizontal (deg) (default: 22.5)
  -s STEPOVER           stepover percentage (affects spacing between lines)
                        (default: 110)
  -r LINEAR_RESOLUTION  distance between successive G-code points (in)
                        (default: 0.01)
  --dots                engrave using dots instead of lines (experimental)

G-code parameters:
  --no-line-numbers     suppress G-code line numbers (dangerous on ShopBot!)
  --preamble PREAMBLE   override the default G-code preamble; to specify
                        multiple lines on the command line, use $'' strings
                        with \n; each line of the preamble will be prepended
                        with a line number, except when used with --no-line-
                        numbers
  --preamble-file PREAMBLE_FILE
                        like --preamble, but read from a file
  --epilogue EPILOGUE   override the default G-code epilogue; see above notes
                        for --preamble
  --epilogue-file EPILOGUE_FILE
                        like --epilogue, but read from a file

The default feeds have been found to be safe values for medium-density
fiberboard (MDF). Experimenting with the STEPOVER, LINE_ANGLE, and
LINEAR_RESOLUTION may yield improvements in engraving quality at the cost of
increased machining time. On ShopBot machines, the --no-line-numbers flag must
not be used, since the spindle will fail to start and damage the material. Use
this flag with caution on other machines.
```

# Related

[Vectric PhotoVCarve](https://www.vectric.com/products/photovcarve) -
a similar commercial solution. This program is not derived from
PhotoVCarve.

[My blog post](https://www.fwei.tk/blog/opening-black-boxes.html) -
writeup on the development process.
