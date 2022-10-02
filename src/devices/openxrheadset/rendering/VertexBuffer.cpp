/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <OpenGLConfig.h>
#include <VertexBuffer.h>

VertexBuffer::VertexBuffer()
{
    glGenBuffers(1, &m_rendererID);
}

VertexBuffer::~VertexBuffer()
{
    glDeleteBuffers(1, &m_rendererID);
}

void VertexBuffer::setVertices(const std::vector<float>& positions)
{
    m_positions = positions;
    glBindBuffer(GL_ARRAY_BUFFER, m_rendererID);
    glBufferData(GL_ARRAY_BUFFER, m_positions.size() * sizeof(float), m_positions.data(), GL_STATIC_DRAW);
}

void VertexBuffer::bind() const
{
    glBindBuffer(GL_ARRAY_BUFFER, m_rendererID);

}

void VertexBuffer::unbind() const
{
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
