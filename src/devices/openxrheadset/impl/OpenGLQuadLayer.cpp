/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/LogStream.h>
#include <impl/OpenGLQuadLayer.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrEigenConversions.h>

#include <Resources.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


bool OpenGLQuadLayer::initialize()
{
    m_positions = {
        // vertex coords   // texture coords
        -0.5f, -0.5f, 0.0, 0.0f, 0.0f, // 0 (the first 3 numbers of the row are the vertex coordinates, the second 2 numbers are the texture coordinate for that vertex (the bottom left corner of the rectangle is also the bottom left corner of the picture))
         0.5f, -0.5f, 0.0, 1.0f, 0.0f, // 1
         0.5f,  0.5f, 0.0, 1.0f, 1.0f, // 2
        -0.5f,  0.5f, 0.0, 0.0f, 1.0f  // 3
    };

    m_indices = {                                                                         // creating an index buffer to save GPU memory
        0, 1, 2,
        2, 3, 0
    };

    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glDepthFunc(GL_LEQUAL));

    GLCall(glEnable(GL_BLEND));
    GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));                          // src (output color of the fragment shader) and dest (color already in the buffer) factors respectively (sfactor, dfactor)

    m_vb.setVertices(m_positions);

    VertexBufferLayout layout;
    layout.Push<float>(3);                          // 3 floats for each vertex position
    layout.Push<float>(2);                          // 2 floats for texture coordinates to be mapped on each vertex
    m_va.AddBuffer(m_vb, layout);

    m_ib.setIndices(m_indices);

    m_shader.initialize(resourcesPath() + "/shaders/Basic.shader");
    m_shader.Bind();
    m_shader.SetUniform4f("u_Color", 0.0f, 1.0f, 0.0f, 1.0f);

    m_texture.setTextureFromPath(resourcesPath() + "/testImages/skeleton.jpg");
    m_texture.Bind();                                 // default input is 0 (first slot)
    m_shader.SetUniform1i("u_Texture", 0);            // the second argument must match the input of texture.Bind(input)

    /* unbinding everything */
    m_va.Unbind();
    m_vb.Unbind();
    m_ib.Unbind();
    m_shader.Unbind();
    m_texture.Unbind();

    return true;
}

bool OpenGLQuadLayer::setRenderAspectRatio(float aspectRatio)
{
    if (aspectRatio <= 0.4f || aspectRatio >= 2.4f)
        return false;
    else
    {
        m_aspectRatio = aspectRatio;
        return true;
    }
}

bool OpenGLQuadLayer::setRenderFov(float fov)
{
    if (fov <= 0.0f || fov >= 180.0f)
        return false;
    else
    {
        m_fov = fov;
        return true;
    }
}

bool OpenGLQuadLayer::setRenderDepth(float zNear, float zFar)
{
    if (zNear <= 0.0f || zNear >= 1000.0f || zFar <= 0.0f || zFar >= 1000.0f)
        return false;
    else
    {
        m_zNear = zNear;
        m_zFar = zFar;
        return true;
    }
}

bool OpenGLQuadLayer::setRenderPose(glm::vec3 modelTranslation, glm::vec3 modelRotation, glm::vec3 modelScale)
{
    m_modelTranslation = modelTranslation;
    m_modelRotation = modelRotation;
    m_modelScale = modelScale;
    return true;
}

bool OpenGLQuadLayer::setRenderColor(float r, float g, float b, float alpha)
{
    if (r < 0.0f || r > 1.0f || g < 0.0f || g > 1.0f || b < 0.0f || b > 1.0f || alpha < 0.0f || alpha > 1.0f)
        return false;
    else
    {
        m_r = r;
        m_g = g;
        m_b = b;
        m_alpha = alpha;
        return true;
    }
}

unsigned int OpenGLQuadLayer::render()
{
    Renderer renderer;

    {
        m_texID = m_texture.Bind();

        glm::mat4 model = glm::translate(glm::mat4(1.0f), m_modelTranslation);
        model = glm::rotate(model, glm::radians(m_modelRotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        model = glm::rotate(model, glm::radians(m_modelRotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::rotate(model, glm::radians(m_modelRotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
        model = glm::scale(model, m_modelScale);
        glm::mat4 view = glm::mat4(1.0f);
        glm::mat4 proj = glm::perspective(glm::radians(m_fov), m_aspectRatio, m_zNear, m_zFar);                           // 3D alternative to "ortho" proj type. It allows to define the view frustum by inserting the y FOV, the aspect ratio of the window, where are placed the near and far clipping planes

        m_shader.Bind();                                                                                             // bind shader
        m_shader.SetUniform4f("u_Color", m_r, m_g, m_b, m_alpha);                                                       // setup uniform
        m_shader.SetUniformMat4f("u_M", model);
        m_shader.SetUniformMat4f("u_V", view);
        m_shader.SetUniformMat4f("u_P", proj);

        renderer.Draw(m_va, m_ib, m_shader);
    }

    if (m_r >= 1.0f)
        m_increment = -0.05f;
    else if (m_r <= 0.0f)
        m_increment = 0.05f;

    m_r += m_increment;
    
    return m_texID;
}

void OpenGLQuadLayer::setPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &quaternion)
{
    setPosition(position);
    setQuaternion(quaternion);
}

void OpenGLQuadLayer::setPosition(const Eigen::Vector3f &position)
{
    //layer.pose.position = toXr(position);
}

void OpenGLQuadLayer::setQuaternion(const Eigen::Quaternionf &quaternion)
{
    //layer.pose.orientation = toXr(quaternion);
}

void OpenGLQuadLayer::setDimensions(float widthInMeters, float heightInMeters)
{
    //layer.size.height = heightInMeters;
    //layer.size.width = widthInMeters;
}

void OpenGLQuadLayer::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    //switch (visibility)
    //{
    //case IOpenXrQuadLayer::Visibility::LEFT_EYE:
    //    layer.eyeVisibility = XR_EYE_VISIBILITY_LEFT;
    //    hasVisibility = true;
    //    break;

    //case IOpenXrQuadLayer::Visibility::RIGHT_EYE:
    //    layer.eyeVisibility = XR_EYE_VISIBILITY_RIGHT;
    //    hasVisibility = true;
    //    break;

    //case IOpenXrQuadLayer::Visibility::BOTH_EYES:
    //    layer.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    //    hasVisibility = true;
    //    break;

    //case IOpenXrQuadLayer::Visibility::NONE:
    //    hasVisibility = false;
    //    break;
    //}
}

void OpenGLQuadLayer::useAlphaChannel(bool useAlphaChannel)
{
    //layer.layerFlags = 0;

    //if (useAlphaChannel)
    //{
    //    layer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT | XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
    //}
}

bool OpenGLQuadLayer::getImage(uint32_t &glImage)
{
    return false;
}

bool OpenGLQuadLayer::submitImage()
{
    return submitImage(0, 0, imageMaxWidth(), imageMaxHeight());
}

bool OpenGLQuadLayer::submitImage(int32_t xOffset, int32_t yOffset, int32_t imageWidth, int32_t imageHeight)
{


    return false;

}

int32_t OpenGLQuadLayer::imageMaxHeight() const
{
    //return swapchainHeight;
    return 0;
}

int32_t OpenGLQuadLayer::imageMaxWidth() const
{
    //return swapchainWidth;
    return 0;
}

float OpenGLQuadLayer::layerWidth() const
{
    //return layer.size.width;
    return 0.0;
}

float OpenGLQuadLayer::layerHeight() const
{
    //return layer.size.height;
    return 0.0;
}

Eigen::Vector3f OpenGLQuadLayer::layerPosition() const
{
    //return toEigen(layer.pose.position);
    return Eigen::Vector3f();
}

Eigen::Quaternionf OpenGLQuadLayer::layerQuaternion() const
{
    //return toEigen(layer.pose.orientation);
    return Eigen::Quaternionf();
}

void OpenGLQuadLayer::setEnabled(bool enabled)
{
    //this->isEnabled = enabled;
}

OpenGLQuadLayer::OpenGLQuadLayer()
{
}

OpenGLQuadLayer::~OpenGLQuadLayer()
{

}
