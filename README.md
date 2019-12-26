# RasterCarve: Generate G-code to engrave raster images

This is a little Python script I wrote to generate 3-axis toolpaths to
engrave raster images.

## Installation

`pip install rastercarve`

## Usage

```
usage: rastercarve [-h] (--width WIDTH | --height HEIGHT) [-f FEED_RATE]
                   [-p PLUNGE_RATE] [--rapid RAPID_RATE] [-z SAFE_Z]
                   [--end-z TRAVERSE_Z] [-d MAX_DEPTH] [-t TOOL_ANGLE]
                   [-a LINE_ANGLE] [-s STEPOVER] [-r LINEAR_RESOLUTION]
                   [--no-line-numbers] [--debug] [-q] [--version]
                   filename

Generate G-code to engrave raster images.

positional arguments:
  filename              input image (any OpenCV-supported format)

optional arguments:
  -h, --help            show this help message and exit
  --debug               print debug messages
  -q                    disable progress and statistics
  --version             show program's version number and exit

output dimensions:
  Exactly one required.

  --width WIDTH         output width (in)
  --height HEIGHT       output height (in)

machine configuration:
  -f FEED_RATE          engraving feed rate (in/min) (default: 100)
  -p PLUNGE_RATE        engraving plunge rate (in/min) (default: 30)
  --rapid RAPID_RATE    rapid traverse rate (for time estimation only)
                        (default: 240)
  -z SAFE_Z             rapid Z traverse height (in) (default: 0.1)
  --end-z TRAVERSE_Z    Z height of final traverse (in) (default: 2)
  -d MAX_DEPTH          maximum engraving depth (in) (default: 0.08)
  -t TOOL_ANGLE         included angle of tool (deg) (default: 30)

engraving parameters:
  -a LINE_ANGLE         angle of grooves from horizontal (deg) (default: 22.5)
  -s STEPOVER           stepover percentage (affects spacing between lines)
                        (default: 110)
  -r LINEAR_RESOLUTION  distance between successive G-code points (in)
                        (default: 0.01)

G-code parameters:
  --no-line-numbers     suppress G-code line numbers (dangerous on ShopBot!)

Defaults are usually safe to leave unchanged.
```

### Examples

```
rastercarve --width 10 examples/test.png > out.nc
```

Generate G-code to engrave `examples/test.png` into an image 10 inches
wide. Output will be piped from stdout to `out.nc`.
