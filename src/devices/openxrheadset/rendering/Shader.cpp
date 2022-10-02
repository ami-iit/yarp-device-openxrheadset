/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <fstream> // to be able to provide the shaders as a file
#include <string>
#include <sstream>
#include <Shader.h>
#include <yarp/os/LogStream.h>
#include <OpenXrHeadsetLogComponent.h>

Shader::Shader()
    : m_rendererID(0)
{ }

Shader::~Shader()
{
    if (!m_initialized)
        return;

    glDeleteProgram(m_rendererID);
}

void Shader::initializeFromPath(const std::string &shaderPath)
{
    std::ifstream shaderFile(shaderPath);
    ShaderProgramSource source = parseShader(shaderFile);
    m_rendererID = createShader(source.vertexSource, source.fragmentSource);
    m_initialized = true;
}

void Shader::initializeFromString(const std::string &shaderValue)
{
    std::stringstream shaderStream(shaderValue);
    ShaderProgramSource source = parseShader(shaderStream);
    m_rendererID = createShader(source.vertexSource, source.fragmentSource);
    m_initialized = true;
}

ShaderProgramSource Shader::parseShader(std::istream& stream)
{
    enum class ShaderType
    {
        NONE = -1, VERTEX = 0, FRAGMENT = 1
    };

    std::string line;
    std::stringstream ss[2];
    ShaderType type = ShaderType::NONE;
    while (getline(stream, line, '\n'))
    {
        if (line.find("#shader") != std::string::npos)
        {
            if (line.find("vertex") != std::string::npos)
                type = ShaderType::VERTEX;
            else if (line.find("fragment") != std::string::npos)
                type = ShaderType::FRAGMENT;
        }
        else
        {
            ss[(int)type] << line << '\n';
        }
    }

    return { ss[0].str(), ss[1].str() };
}


unsigned int Shader::compileShader(unsigned int type, const std::string& source)         // string& --> passing the item by reference
{
    unsigned int id = glCreateShader(type);
    const char* src = source.c_str();                            // OpenGL is expecting a Raw String, not a std::string. This line returns a pointer to the beginning of our data
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    // Error handling
    int result;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE)
    {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        glGetShaderInfoLog(id, length, &length, message);
        yCError(OPENXRHEADSET) << "Failed to compile" << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << "shader!";
        yCError(OPENXRHEADSET) << message;
        glDeleteShader(id);
        return 0;
    }

    return id;
}

unsigned int Shader::createShader(const std::string& vertexShader, const std::string& fragmentShader)         //shaders can be provided to OpenGL as strings (most basic approach)
{
    /* inside this scope, the code needed to compile the 2 shaders */
    unsigned int program = glCreateProgram();
    unsigned int vs = compileShader(GL_VERTEX_SHADER, vertexShader);
    unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragmentShader);

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}


void Shader::bind() const
{
    glUseProgram(m_rendererID);
}

void Shader::unbind() const
{
    glUseProgram(0);
}

void Shader::setUniform1i(const std::string& name, int value)
{
    glUniform1i(getUniformLocation(name), value);
}

void Shader::setUniform1f(const std::string& name, float value)
{
    glUniform1f(getUniformLocation(name), value);
}

void Shader::setUniform4f(const std::string& name, float v0, float v1, float v2, float v3)
{
    glUniform4f(getUniformLocation(name), v0, v1, v2, v3);
}

void Shader::setUniformMat4f(const std::string& name, const glm::mat4& matrix)
{
    glUniformMatrix4fv(getUniformLocation(name), 1, GL_FALSE, &matrix[0][0]);
}

int Shader::getUniformLocation(const std::string& name)
{
    if (m_uniformLocationCache.find(name) != m_uniformLocationCache.end())
        return m_uniformLocationCache[name];

    int location = glGetUniformLocation(m_rendererID, name.c_str());
    if (location == -1)
        yCWarning(OPENXRHEADSET) << "Uniform '" << name << "'doesn't exist!";

    m_uniformLocationCache[name] = location;
    return location;
}
