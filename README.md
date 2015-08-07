# Introduction
Fovis is a visual odometry library that estimates the 3D motion of a camera
using a source of depth information for each pixel. It's designed for sensors
such as calibrated stereo cameras and RGB-D cameras like the Microsoft Kinect.
Its original implementation is described in the following paper:

Visual Odometry and Mapping for Autonomous Flight Using an RGB-D Camera. Albert
S. Huang, Abraham Bachrach, Peter Henry, Michael Krainin, Daniel Maturana,
  Dieter Fox, and Nicholas Roy. Int. Symposium on Robotics Research (ISRR),
  Flagstaff, Arizona, USA, Aug. 2011 PDF.

# Build requirements
Fovis is intended to be relatively portable. The requirements for building and
using the software are:

- CMake
- Eigen 3

Fovis was developed and tested on Ubuntu. It may work on other platforms, but
no guarantees.

# Build instructions

See libfovis/README for instructions on building libfovis.

The fovis-bot2 and fovis-analysis directories are included for completeness,
but you probably won't want to build them.

# Usage requirements
For portability reasons, the actual library itself is sensor agnostic and
provides no data acquisition capabilities. To use fovis, your program must
acquire data on its own and pass it through to the fovis API. Some examples are
provided with the source code.

Effective use of fovis for visual odometry requires the following:

- A source of 8-bit grayscale camera images.
- A camera calibration for the images that provides an accurate mapping between
image pixel coordinates and 3D rays in the camera's Cartesian coordinate frame.
- A depth source for each image. A depth source must be able to provide a
metric depth estimate for as many pixels in the camera image as possible.

Fovis provides built-in support for the following types of depth sources:

- An RGB-D camera such as the Microsoft Kinect.
- Calibrated stereo cameras.
- You can also create your own depth sources using the Fovis API and adapt it
to other sensor types.

# Getting started
The best way to get started is to look through the examples provided with the
source code in the examples/ directory.

Next, look through the Fovis C++ API. The primary class of interest is
fovis::VisualOdometry.

# License
fovis is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

fovis is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

A copy of the GNU General Public License is provided with the fovis source
code.

Contact the authors for other licensing options.
