#pragma once

#include <vector>

class VertexBuffer
{
private:
    unsigned int m_RendererID{ 0 };
    std::vector<float> m_positions;

public:

    VertexBuffer();

    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer(VertexBuffer&&) = delete;

    VertexBuffer& operator=(const VertexBuffer&) = delete;
    VertexBuffer& operator=(VertexBuffer&&) = delete;

    ~VertexBuffer();

    void setVertices(const std::vector<float>& positions);

    void Bind() const;
    void Unbind() const;
};