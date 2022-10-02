/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_TEXTURE_H
#define YARP_DEV_TEXTURE_H

#include <OpenGLConfig.h>
#include <FrameBuffer.h>

class Texture
{
private:
    unsigned int m_rendererID{ 0 };
    std::string m_filePath;
    unsigned char* m_localBuffer;
    int m_width, m_height;
public:
    Texture();

    Texture(const Texture&) = delete;
    Texture(Texture&&) = delete;

    Texture& operator=(const Texture&) = delete;
    Texture& operator=(Texture&&) = delete;

    ~Texture();

    void setTextureFromPath(const std::string& path);

    void allocateTexture(int32_t imageMaxWidth, int32_t imageMaxHeight);

    unsigned int getTextureID() const;

    unsigned int bind(unsigned int slot = 0) const;            // usually you have 32 texture slots to bind muliple Textures
    unsigned int bindToFrameBuffer(FrameBuffer& frameBuffer, unsigned int slot = 0);
    void unbind() const;

    int width() const;
    int height() const;

};

#endif //YARP_DEV_TEXTURE_H
