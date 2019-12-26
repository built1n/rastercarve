#!/usr/bin/env python
import setuptools
from rastercarve import __version__

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="rastercarve", # Replace with your own username
    version=__version__,
    author="Franklin Wei",
    author_email="franklin@rockbox.org",
    description="Generate G-code to engrave raster images",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/built1n/rastercarve",
    packages=setuptools.find_packages(),
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Environment :: Console",
        "License :: OSI Approved :: GNU General Public License v2 or later (GPLv2+)",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering :: Electronic Design Automation (EDA)",
        "Topic :: Utilities",
    ],
    python_requires='>=3.6',
    install_requires=["opencv-python", "numpy", "tqdm", "argparse"],
    entry_points={
        "console_scripts": [
            "rastercarve=rastercarve.__main__:main",
        ]
    }
)
