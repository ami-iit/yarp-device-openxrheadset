# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-2-Clause license. See the accompanying LICENSE file for details.

add_library(stb_image stb_image.h stb_image.cpp)

target_include_directories(stb_image PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>"
                                            "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/stb_image>")

set_target_properties(stb_image PROPERTIES
                                OUTPUT_NAME stb_image
                                PUBLIC_HEADER stb_image.h)

## Specify installation targets, typology and destination folders.
install(TARGETS          stb_image
        EXPORT           stb_image
        COMPONENT        runtime
        LIBRARY          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                      COMPONENT shlib
        ARCHIVE          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                      COMPONENT lib
        RUNTIME          DESTINATION "${CMAKE_INSTALL_BINDIR}"                      COMPONENT bin
        PUBLIC_HEADER    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/stb_image/" COMPONENT dev)
