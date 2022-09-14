# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-2-Clause license. See the accompanying LICENSE file for details.

add_library(stb_image stb_image.h stb_image.cpp)
target_include_directories(stb_image PUBLIC ${CMAKE_CURRENT_LIST_DIR})
