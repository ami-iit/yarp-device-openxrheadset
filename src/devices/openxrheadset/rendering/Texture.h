#pragma once

#include <Renderer.h>
#include <FrameBuffer.h>

class Texture
{
private:
    unsigned int m_RendererID{ 0 };
    std::string m_FilePath;
    unsigned char* m_LocalBuffer;
    int m_Width, m_Height, m_BPP;                // Bits Per Pixel of the texture
public:
    Texture();

    Texture(const Texture&) = delete;
    Texture(Texture&&) = delete;

    Texture& operator=(const Texture&) = delete;
    Texture& operator=(Texture&&) = delete;

    ~Texture();

    void setTextureFromPath(const std::string& path);

    void allocateTexture(int32_t imageMaxWidth, int32_t imageMaxHeight);

    unsigned int GetTextureID() const;

    unsigned int Bind(unsigned int slot = 0) const;            // usually you have 32 texture slots to bind muliple Textures
    unsigned int BindToFrameBuffer(FrameBuffer& frameBuffer, unsigned int slot = 0);
    void Unbind() const;

    inline int GetWidth() const { return m_Width; }
    inline int GetHeight() const { return m_Height; }

};
