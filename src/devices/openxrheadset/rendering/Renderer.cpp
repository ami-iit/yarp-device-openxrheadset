#include "Renderer.h"

#include <iostream>

void GLClearError()
{
    while (glGetError() != GL_NO_ERROR);
}

bool GLLogCall(const char* function, const char* file, int line)                                                           // name of the function that we are trying to call, cpp file that is causing the error, line of the code
{
    while (GLenum error = glGetError())
    {
        std::cout << "[OpenGL Error] (" << error << "): " << function << " " << file << ":" << line << std::endl;
        return false;
    }

    return true;
}

void Renderer::Clear() const
{
    GLCall(glClear(GL_COLOR_BUFFER_BIT));
    GLCall(glClear(GL_DEPTH_BUFFER_BIT));
}

void Renderer::Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader) const
{
    shader.Bind();                                                                                                         // bind shader
    va.Bind();
    ib.Bind();
    GLCall(glDrawElements(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, nullptr));                                         // Used with an index buffer. Without index buffer you may use "glDrawArrays". They both draw what has been previously binded through glBindBuffer. We are drawing six indices for a square. GLCall checks for errors on its line.
}
