#pragma once

#include <GL/glew.h>

#include "VertexArray.h"
#include "IndexBuffer.h"
#include "Shader.h"

#define ASSERT(x) if (!(x)) __debugbreak();                                                            // ASSERTION
#define GLCall(x) GLClearError();\
    x;\
    ASSERT(GLLogCall(#x, __FILE__, __LINE__))                                       // #x turns x into a string. C++ macros

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);             // name of the function that we are trying to call, cpp file that is causing the error, line of the code

class Renderer
{
public:
    void Clear() const;
    void Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader) const;
};