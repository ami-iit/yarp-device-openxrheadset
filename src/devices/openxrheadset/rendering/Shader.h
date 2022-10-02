/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_SHADER_H
#define YARP_DEV_SHADER_H

#include <OpenGLConfig.h>
#include <string>
#include <unordered_map>

struct ShaderProgramSource
{
    std::string vertexSource;
    std::string fragmentSource;
};

class Shader
{
private:
    unsigned int m_rendererID;
    std::unordered_map<std::string, int> m_uniformLocationCache;
    bool m_initialized{ false };

public:
    Shader();
    ~Shader();

    Shader(const Shader&) = delete;
    Shader(Shader&&) = delete;

    Shader& operator=(const Shader&) = delete;
    Shader& operator=(Shader&&) = delete;


    void initialize(const std::string& shaderPath);

    void bind() const;                    // it would be "use program" for shaders, but let's use "bind" for consistency
    void unbind() const;

    // Set uniforms
    void setUniform1i(const std::string& name, int value);
    void setUniform1f(const std::string& name, float value);
    void setUniform4f(const std::string& name, float v0, float v1, float v2, float v3);                // v standing for value
    void setUniformMat4f(const std::string& name, const glm::mat4& matrix);

private:
    ShaderProgramSource parseShader(const std::string& filepath);
    unsigned int compileShader(unsigned int type, const std::string& source);
    unsigned int createShader(const std::string& vertexShader, const std::string& fragmentShader);
    int getUniformLocation(const std::string& name);
};

#endif //YARP_DEV_SHADER_H
