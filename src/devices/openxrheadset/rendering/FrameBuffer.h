#pragma once

#include <Renderer.h>

class FrameBuffer
{
	GLuint m_bufferId{ 0 };
	GLenum m_target{ GL_FRAMEBUFFER };

public:

	FrameBuffer();

	~FrameBuffer();

	FrameBuffer(const FrameBuffer& other) = delete;
	FrameBuffer(FrameBuffer&& other) = delete;

	FrameBuffer& operator=(const FrameBuffer&) = delete;
	FrameBuffer& operator=(FrameBuffer&&) = delete;

	void Bind(GLenum target);

	void Unbind();
};