# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-2-Clause license. See the accompanying LICENSE file for details.

find_package(iDynTree 4.0.0 QUIET)

option(COMPILE_FRAME_VIZ "Compile the frame visualization utility" ${iDynTree_FOUND})

if (COMPILE_FRAME_VIZ)
    add_subdirectory(OpenXrFrameViz)
endif()
