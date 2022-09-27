#pragma once

#include <OpenGLConfig.h>

#include <VertexArray.h>
#include <IndexBuffer.h>
#include <Shader.h>

#define GLCall(x) GLClearError();\
    x;\
    GLLogCall(#x, __FILE__, __LINE__)

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);             // name of the function that we are trying to call, cpp file that is causing the error, line of the code

class Renderer
{
public:
    void Clear() const;
    void Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader) const;
};
