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

#include <GL/gl.h>
#include <GL/glext.h>

#if defined(__APPLE__)
 #include <GL/glx.h>
#elif defined(__linux__)
 #include <GL/glx.h>
#endif

#ifdef Success
  #undef Success
#endif //See https://gitlab.com/libeigen/eigen/-/issues/253


#endif // YARP_DEV_OPENGLCONFIG_H
