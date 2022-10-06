/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */


#include <VertexBufferLayout.h>

const std::vector<VertexBufferElement> VertexBufferLayout::getElements() const
{
    return m_elements;
}

unsigned int VertexBufferLayout::getStride() const
{
    return m_stride;
}
