#pragma once

#include "Renderer.h"

class Texture
{
private:
	unsigned int m_RendererID;
	std::string m_FilePath;
	unsigned char* m_LocalBuffer;
	int m_Width, m_Height, m_BPP;				// Bits Per Pixel of the texture
public:
	Texture(const std::string& path);
	~Texture();

	void Bind(unsigned int slot = 0) const;			// usually you have 32 texture slots to bind muliple Textures
	void Unbind() const;

	inline int GetWidth() const { return m_Width; }
	inline int GetHeight() const { return m_Height; }

};