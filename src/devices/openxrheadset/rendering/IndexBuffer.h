/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_INDEXBUFFER_H
#define YARP_DEV_INDEXBUFFER_H

#include <vector>

class IndexBuffer
{
private:
    unsigned int m_RendererID{ 0 };
    std::vector<unsigned int> m_data;

public:
    IndexBuffer();
    ~IndexBuffer();

    void setIndices(const std::vector<unsigned int>& data);

    void bind() const;
    void unbind() const;

    unsigned int getCount() const;
};

#endif //YARP_DEV_INDEXBUFFER_H
