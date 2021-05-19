/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRCONFIG_H
#define YARP_DEV_OPENXRCONFIG_H

#define XR_USE_GRAPHICS_API_OPENGL

#include "OpenGLConfig.h"

#if defined(_WIN32)
 #define XR_USE_PLATFORM_WIN32
#elif defined(__APPLE__)
 #define XR_USE_PLATFORM_XLIB
#elif defined(__linux__)
 #define XR_USE_PLATFORM_XLIB
#endif

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#endif // YARP_DEV_OPENXRCONFIG_H
