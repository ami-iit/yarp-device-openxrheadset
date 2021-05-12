# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-2-Clause license. See the accompanying LICENSE file for details.

set(DEPENDS_STRING "OpenXR_FOUND;glfw3_FOUND;OpenGL_FOUND; Eigen3_FOUND")

if (NOT WIN32)
    string(APPEND DEPENDS_STRING ";X11_FOUND")
endif()

yarp_prepare_plugin(openxrheadset
  CATEGORY device
  TYPE yarp::dev::OpenXrHeadset
  INCLUDE OpenXrHeadset.h
  DEPENDS ${DEPENDS_STRING}
  INTERNAL
  QUIET
)

if(NOT ENABLE_openxrheadset)
  return()
endif()

yarp_add_plugin(yarp_openxrheadset)

set(yarp_openxrheadset_SRCS
  OpenXrHeadset.cpp
  OpenXrHeadsetLogComponent.cpp
  OpenXrInterface.cpp
  OpenXrQuadLayer.cpp
)

set(yarp_openxrheadset_HDRS
  OpenXrConfig.h
  OpenGLConfig.h
  OpenXrHeadset.h
  OpenXrHeadsetLogComponent.h
  PortToQuadLayer.h
  OpenXrInterface.h
  OpenXrQuadLayer.h
  )

target_sources(yarp_openxrheadset
  PRIVATE
    ${yarp_openxrheadset_SRCS}
    ${yarp_openxrheadset_HDRS}
)

target_link_libraries(yarp_openxrheadset
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    YARP::YARP_math
    OpenXR::openxr_loader
    glfw
    OpenGL::GL
    Eigen3::Eigen
)

if (NOT WIN32)
    target_link_libraries(yarp_openxrheadset PRIVATE ${X11_LIBRARIES})
endif()

yarp_install(
  TARGETS yarp_openxrheadset
  EXPORT yarp-device-openxrheadset
  COMPONENT yarp-device-openxrheadset
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)
