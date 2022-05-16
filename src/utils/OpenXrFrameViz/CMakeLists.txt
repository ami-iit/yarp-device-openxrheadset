# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-2-Clause license. See the accompanying LICENSE file for details.


find_package(iDynTree 4.0.0 REQUIRED)

add_executable(OpenXrFrameViz main.cpp)

target_link_libraries(OpenXrFrameViz PRIVATE  YARP::YARP_os YARP::YARP_sig
                                              YARP::YARP_dev YARP::YARP_math
                                              iDynTree::idyntree-visualization iDynTree::idyntree-yarp)

install(TARGETS OpenXrFrameViz DESTINATION bin)
