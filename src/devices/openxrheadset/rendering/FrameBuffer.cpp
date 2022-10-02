/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <FrameBuffer.h>

FrameBuffer::FrameBuffer()
{
    glGenFramebuffers(1, &(m_bufferId));
}

FrameBuffer::~FrameBuffer()
{
    if (m_bufferId != 0)
    {
        unbind();
        glDeleteFramebuffers(1, &m_bufferId);
        m_bufferId = 0;
    }
}

void FrameBuffer::bind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_bufferId);
}

void FrameBuffer::unbind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GLuint FrameBuffer::id()
{
    return m_bufferId;
}
