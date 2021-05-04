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
#define GL_GLEXT_PROTOTYPES
#define GL3_PROTOTYPES

#include <GL/gl.h>
#include <GL/glext.h>

#if defined(_WIN32)
 #define XR_USE_PLATFORM_WIN32
 #define GLFW_EXPOSE_NATIVE_WIN32
 #define GLFW_EXPOSE_NATIVE_WGL
#elif defined(__APPLE__)
 #define XR_USE_PLATFORM_XLIB
 #define GLFW_EXPOSE_NATIVE_COCOA
 #define GLFW_EXPOSE_NATIVE_NSGL
 #include <GL/glx.h>
#elif defined(__linux__)
 #define XR_USE_PLATFORM_XLIB
 #define GLFW_EXPOSE_NATIVE_X11
 #define GLFW_EXPOSE_NATIVE_GLX
 #include <GL/glx.h>
#endif

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#endif // YARP_DEV_OPENXRCONFIG_H
