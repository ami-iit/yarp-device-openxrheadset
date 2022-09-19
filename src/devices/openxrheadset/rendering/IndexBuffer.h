#pragma once

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

	void Bind() const;
	void Unbind() const;

	unsigned int GetCount() const;
};