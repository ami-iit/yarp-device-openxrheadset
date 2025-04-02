# yarp-device-openxrheadset


`openxrheadset` YARP Device Driver for OpenXR-based headsets.

# Installation

## Using conda

The `yarp-device-openxrheadset` binary package is available from the `robotology` channel, to create a new environment that contains it run:

~~~
conda create -n yarpopenxrdev -c conda-forge -c robotology yarp-device-openxrheadset
~~~

## From source

Make sure to install the required dependencies:
- C++ compiler
- CMake
- ycm-cmake-modules
- YARP
- OpenXR
- GLEW
- GLM
- Eigen3

Then install as any other CMake project:

~~~
git clone https://github.com/ami-iit/yarp-device-openxrheadset
cd yarp-device-openxrheadset
cmake -Bbuild -S.
cmake --build build
cmake --install build
~~~
