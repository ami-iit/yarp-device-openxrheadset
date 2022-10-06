/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_VERTEXBUFFER_H
#define YARP_DEV_VERTEXBUFFER_H

#include <vector>

class VertexBuffer
{
private:
    unsigned int m_rendererID{ 0 };
    std::vector<float> m_positions;

public:

    VertexBuffer();

    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer(VertexBuffer&&) = delete;

    VertexBuffer& operator=(const VertexBuffer&) = delete;
    VertexBuffer& operator=(VertexBuffer&&) = delete;

    ~VertexBuffer();

    void setVertices(const std::vector<float>& positions);

    void bind() const;
    void unbind() const;
};

#endif //YARP_DEV_VERTEXBUFFER_H
