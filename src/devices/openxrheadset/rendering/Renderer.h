/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_RENDERER_H
#define YARP_DEV_RENDERER_H

#include <OpenGLConfig.h>
#include <VertexArray.h>
#include <IndexBuffer.h>
#include <Shader.h>

class Renderer
{
public:

    void clear() const;
    void draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader) const;

};

#endif //YARP_DEV_RENDERER_H
