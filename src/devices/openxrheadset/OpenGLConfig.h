/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENGLCONFIG_H
#define YARP_DEV_OPENGLCONFIG_H

#define GL_GLEXT_PROTOTYPES
#define GL3_PROTOTYPES
#define NOMINMAX //On windows this avoids issues when using min and max

#include <GL/glew.h>
#include <GL/gl.h>

#if defined(_WIN32)
 #define GLFW_EXPOSE_NATIVE_WIN32
 #define GLFW_EXPOSE_NATIVE_WGL
#elif defined(__APPLE__)
 #define GLFW_EXPOSE_NATIVE_COCOA
 #define GLFW_EXPOSE_NATIVE_NSGL
 #include <GL/glx.h>
#elif defined(__linux__)
 #define GLFW_EXPOSE_NATIVE_X11
 #define GLFW_EXPOSE_NATIVE_GLX
 #include <GL/glx.h>
#endif

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#ifdef Success
  #undef Success
#endif //See https://gitlab.com/libeigen/eigen/-/issues/253


#endif // YARP_DEV_OPENGLCONFIG_H
