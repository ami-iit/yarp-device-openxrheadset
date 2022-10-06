/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_FRAMEBUFFER_H
#define YARP_DEV_FRAMEBUFFER_H

#include <Renderer.h>

class FrameBuffer
{
    GLuint m_bufferId{ 0 };

public:

    FrameBuffer();

    ~FrameBuffer();

    FrameBuffer(const FrameBuffer& other) = delete;
    FrameBuffer(FrameBuffer&& other) = delete;

    FrameBuffer& operator=(const FrameBuffer&) = delete;
    FrameBuffer& operator=(FrameBuffer&&) = delete;

    void bind();

    void unbind();

    GLuint id();
};

#endif //YARP_DEV_FRAMEBUFFER_H
