#include <VertexBufferLayout.h>


VertexBufferLayout::VertexBufferLayout()
    : m_Stride(0) {}

void VertexBufferLayout::Push(unsigned int count)
{
    m_Elements.push_back({ GL_FLOAT, count, GL_FALSE });
    m_Stride += count * VertexBufferElement::GetSizeOfType(GL_FLOAT);   // size of the thing that we are pushing back (4 bytes)
}

const std::vector<VertexBufferElement> VertexBufferLayout::GetElements() const
{
    return m_Elements;
}

unsigned int VertexBufferLayout::GetStride() const {
    return m_Stride;
}
