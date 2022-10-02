/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <Texture.h>
#include <stb_image.h>

Texture::Texture()
    : m_rendererID(0), m_localBuffer(nullptr), m_width(0), m_height(0)
{
    glGenTextures(1, &m_rendererID);
}

Texture::~Texture()
{
    glDeleteTextures(1, &m_rendererID);
}

void Texture::setTextureFromPath(const std::string& path)
{
    m_filePath = path;

    stbi_set_flip_vertically_on_load(1);

    int bytesPerPixel;

    m_localBuffer = stbi_load(path.c_str(), &m_width, &m_height, &bytesPerPixel, 4);

    glBindTexture(GL_TEXTURE_2D, m_rendererID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);            // MANDATORY
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);            // MANDATORY
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);        // MANDATORY - horizontal clamp
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);        // MANDATORY - vertical clamp

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_width, m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_localBuffer);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (m_localBuffer)            // if the buffer contains data
        stbi_image_free(m_localBuffer);
}

void Texture::allocateTexture(int32_t imageMaxWidth, int32_t imageMaxHeight)
{
    bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTextureStorage2D(m_rendererID, 1, GL_RGBA8, imageMaxWidth, imageMaxHeight);
}

unsigned int Texture::getTextureID() const
{
    return m_rendererID;
}

unsigned int Texture::bind(unsigned int slot) const
{
    glActiveTexture(GL_TEXTURE0 + slot);                    // selecting the first slot to be the active one (if you call bind after this line it will bind slot 1)
    glBindTexture(GL_TEXTURE_2D, m_rendererID);

    return m_rendererID;
}

unsigned int Texture::bindToFrameBuffer(FrameBuffer &frameBuffer, unsigned int slot)
{
    frameBuffer.bind();
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, bind(slot), slot);
    return m_rendererID;
}

void Texture::unbind() const
{
    glBindTexture(GL_TEXTURE_2D, 0);
}

int Texture::width() const
{
    return m_width;
}

int Texture::height() const
{
    return m_height;
}
