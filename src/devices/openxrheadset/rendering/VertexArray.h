/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_VERTEXARRAY_H
#define YARP_DEV_VERTEXARRAY_H

#include <VertexBuffer.h>
#include <VertexBufferLayout.h>

class VertexArray
{
private:
    unsigned int m_rendererID;
public:
    VertexArray();
    ~VertexArray();

    void addBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout);

    void bind() const;
    void unbind() const;
};

#endif //YARP_DEV_VERTEXARRAY_H
