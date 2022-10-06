/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_VERTEXBUFFERLAYOUT_H
#define YARP_DEV_VERTEXBUFFERLAYOUT_H

#include <OpenGLConfig.h>
#include <vector>
#include <cassert>

struct VertexBufferElement
{
    unsigned int type;
    unsigned int count;
    unsigned char normalized;

    static unsigned int GetSizeOfType(unsigned int type)
    {
        switch (type)
        {
            case GL_FLOAT:          return 4;
            case GL_UNSIGNED_INT:   return 4;
            case GL_UNSIGNED_BYTE:  return 1;
        }
        assert(false);
        return 0;

    }
};

class VertexBufferLayout
{
private:
    std::vector<VertexBufferElement> m_elements;
    unsigned int m_stride{0};
public:

    template <typename T>
    inline void push(unsigned int count) = delete;

    const std::vector<VertexBufferElement> getElements() const;
    unsigned int getStride() const;
};

template<>
inline void VertexBufferLayout::push<float>(unsigned int count)
{
    m_elements.push_back({ GL_FLOAT, count, GL_FALSE });
    m_stride += count * VertexBufferElement::GetSizeOfType(GL_FLOAT); // size of the thing that we are pushing back (4 bytes)
};

template<>
inline void VertexBufferLayout::push<unsigned int>(unsigned int count)
{
    m_elements.push_back({ GL_UNSIGNED_INT, count, GL_FALSE });
    m_stride += count * VertexBufferElement::GetSizeOfType(GL_UNSIGNED_INT); // size of the thing that we are pushing back
}

template<>
inline void VertexBufferLayout::push<unsigned char>(unsigned int count)
{
    m_elements.push_back({ GL_UNSIGNED_BYTE, count, GL_TRUE });
    m_stride += count * VertexBufferElement::GetSizeOfType(GL_UNSIGNED_BYTE); // size of the thing that we are pushing back
}

#endif //YARP_DEV_VERTEXBUFFERLAYOUT_H
