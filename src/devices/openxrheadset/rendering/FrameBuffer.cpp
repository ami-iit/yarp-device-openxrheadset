#include <FrameBuffer.h>

FrameBuffer::FrameBuffer()
{
    glGenFramebuffers(1, &(m_bufferId));
}

FrameBuffer::~FrameBuffer()
{
    if (m_bufferId != 0)
    {
        Unbind();
        glDeleteFramebuffers(1, &m_bufferId);
        m_bufferId = 0;
    }
}

void FrameBuffer::Bind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_bufferId);
}

void FrameBuffer::Unbind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GLuint FrameBuffer::ID()
{
    return m_bufferId;
}
