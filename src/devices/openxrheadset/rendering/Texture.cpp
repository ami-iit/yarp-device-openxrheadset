#include "Texture.h"

#include <stb_image.h>

Texture::Texture()
    : m_RendererID(0), m_LocalBuffer(nullptr), m_Width(0), m_Height(0), m_BPP(0)
{
    GLCall(glGenTextures(1, &m_RendererID));
}

Texture::~Texture()
{
    GLCall(glDeleteTextures(1, &m_RendererID));
}

void Texture::setTextureFromPath(const std::string& path)
{
    m_FilePath = path;

    stbi_set_flip_vertically_on_load(1);
    m_LocalBuffer = stbi_load(path.c_str(), &m_Width, &m_Height, &m_BPP, 4);

    GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID));

    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));            // MANDATORY
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));            // MANDATORY
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));        // MANDATORY - horizontal clamp
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));        // MANDATORY - vertical clamp

    GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_Width, m_Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_LocalBuffer));
    GLCall(glBindTexture(GL_TEXTURE_2D, 0));

    if (m_LocalBuffer)            // if the buffer contains data
        stbi_image_free(m_LocalBuffer);
}

void Texture::allocateTexture(int32_t imageMaxWidth, int32_t imageMaxHeight)
{
    Bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTextureStorage2D(m_RendererID, 1, GL_RGBA8, imageMaxWidth, imageMaxHeight);
}

unsigned int Texture::GetTextureID() const
{
    return m_RendererID;
}

unsigned int Texture::Bind(unsigned int slot) const
{
    GLCall(glActiveTexture(GL_TEXTURE0 + slot));                    // selecting the first slot to be the active one (if you call bind after this line it will bind slot 1)
    GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID));

    return m_RendererID;
}

unsigned int Texture::BindToFrameBuffer(FrameBuffer &frameBuffer, unsigned int slot)
{
    frameBuffer.Bind();
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, Bind(slot), slot);
    return m_RendererID;
}

void Texture::Unbind() const
{
    GLCall(glBindTexture(GL_TEXTURE_2D, 0));
}
