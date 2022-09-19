#include "IndexBuffer.h"

#include "Renderer.h"

IndexBuffer::IndexBuffer()
{
    ASSERT(sizeof(unsigned int) == sizeof(GLuint));

    GLCall(glGenBuffers(1, &m_RendererID));

}

IndexBuffer::~IndexBuffer()
{
    GLCall(glDeleteBuffers(1, &m_RendererID));
}

void IndexBuffer::setIndices(const std::vector<unsigned int>& data)
{
    m_data = data;
    GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RendererID));
    GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_data.size() * sizeof(unsigned int), m_data.data(), GL_STATIC_DRAW));
}

void IndexBuffer::Bind() const
{
    GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RendererID));

}

void IndexBuffer::Unbind() const
{
    GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

}

unsigned int IndexBuffer::GetCount() const 
{ 
    return m_data.size(); 
}
