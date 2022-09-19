#include "VertexBuffer.h"

#include "Renderer.h"

VertexBuffer::VertexBuffer()
{
    GLCall(glGenBuffers(1, &m_RendererID));
}

VertexBuffer::~VertexBuffer()
{
    GLCall(glDeleteBuffers(1, &m_RendererID));
}

void VertexBuffer::setVertices(const std::vector<float>& positions)
{
    m_positions = positions;
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RendererID));
    GLCall(glBufferData(GL_ARRAY_BUFFER, m_positions.size() * sizeof(float), m_positions.data(), GL_STATIC_DRAW));
}

void VertexBuffer::Bind() const
{
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RendererID));

}

void VertexBuffer::Unbind() const
{
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));

}
