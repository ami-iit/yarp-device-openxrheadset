#pragma once

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

    void Bind();

    void Unbind();

    GLuint ID();
};
