/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <OpenGLConfig.h>
#include <VertexArray.h>

VertexArray::VertexArray()
{
    glGenVertexArrays(1, &m_rendererID);
}

VertexArray::~VertexArray()
{
    glDeleteVertexArrays(1, &m_rendererID);
}

void VertexArray::addBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout)
{
    bind();                                                                                                                                //bind vertex array
    vb.bind();
    const auto& elements = layout.getElements();
    unsigned int offset = 0;
    for (unsigned int i = 0; i < elements.size(); i++)
    {
        const auto& element = elements[i];
        glEnableVertexAttribArray(i);                                                                                            // I need to enable each attribute before or after its definition
        glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.getStride(), (const void*)(uintptr_t)offset);     // the first argument means that index 0 of the vao is binded to the currently-bound VERTEX BUFFER.
        offset += element.count * VertexBufferElement::GetSizeOfType(element.type);
    }
}

void VertexArray::bind() const
{
    glBindVertexArray(m_rendererID);
}

void VertexArray::unbind() const
{
    glBindVertexArray(0);
}
