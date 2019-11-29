# RasterCarve: Generate G-code to engrave raster images

This is a little Python script I wrote to generate 3-axis toolpaths to
engrave raster images.

## Getting Started

You just need Python 3, OpenCV, and NumPy (i.e. `pip install ...`).

Then, just run `python src/rastercarve.py IMAGE`, where `IMAGE` is a
bitmap image in any format supported by OpenCV. G-code is output to
standard output.

## Configuration

Edit `rastercarve.py` to change the material and engraving settings.
