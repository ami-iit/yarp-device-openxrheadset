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

void FrameBuffer::Bind(GLenum target)
{
    m_target = target;
    glBindFramebuffer(target, m_bufferId);
}

void FrameBuffer::Unbind()
{
    glBindFramebuffer(m_target, 0);
}