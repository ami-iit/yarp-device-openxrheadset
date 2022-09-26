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
#include <glm/gtc/quaternion.hpp>


bool OpenGLQuadLayer::initialize(int32_t imageMaxWidth, int32_t imageMaxHeight)
{
    m_imageMaxWidth = imageMaxWidth;
    m_imageMaxHeight = imageMaxHeight;

    m_positions = {
        // vertex coords         // texture coords
        -0.5f, -0.5f, 0.0, 1.0f, 0.0f, 0.0f, // 0 (the first 3 numbers of the row are the vertex coordinates, the second 2 numbers are the texture coordinate for that vertex (the bottom left corner of the rectangle is also the bottom left corner of the picture))
         0.5f, -0.5f, 0.0, 1.0f, 1.0f, 0.0f, // 1
         0.5f,  0.5f, 0.0, 1.0f, 1.0f, 1.0f, // 2
        -0.5f,  0.5f, 0.0, 1.0f, 0.0f, 1.0f  // 3
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
    layout.Push(4);                          // 3 floats for each vertex position
    layout.Push(2);                          // 2 floats for texture coordinates to be mapped on each vertex
    m_va.AddBuffer(m_vb, layout);

    m_ib.setIndices(m_indices);

    m_shader.initialize(resourcesPath() + "/shaders/Basic.shader");
    m_shader.Bind();
    m_shader.SetUniform4f("u_Color", 0.0f, 1.0f, 0.0f, 1.0f);               // for debug purposes

    m_texture.Bind();                                                                   // default input is 0 (first slot)
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));			// MANDATORY
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));			// MANDATORY
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));		// MANDATORY - horizontal clamp
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));		// MANDATORY - vertical clamp
    m_shader.SetUniform1i("u_Texture", 0);                                              // the second argument must match the input of texture.Bind(input)

    /* unbinding everything */
    m_va.Unbind();
    m_vb.Unbind();
    m_ib.Unbind();
    m_shader.Unbind();
    m_texture.Unbind();

    return true;
}

bool OpenGLQuadLayer::setAspectRatio(float aspectRatio)
{
    if (aspectRatio <= 0.4f || aspectRatio >= 2.4f)
        return false;
    else
    {
        m_aspectRatio = aspectRatio;
        return true;
    }
}

bool OpenGLQuadLayer::setFov(float fov)
{
    if (fov <= 0.0f || fov >= 180.0f)
        return false;
    else
    {
        m_fov = fov;
        return true;
    }
}

bool OpenGLQuadLayer::setDepthLimits(float zNear, float zFar)
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

bool OpenGLQuadLayer::setColor(float r, float g, float b, float alpha)
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

        glm::mat4 offsetPose = m_offsetTra * m_offsetRot;        
        glm::mat4 modelPose = m_modelTra * m_modelRot;
        glm::mat4 sca = glm::scale(glm::mat4(1.0f), m_modelScale);

        glm::mat4 model = offsetPose * modelPose * sca;
        glm::mat4 view = glm::mat4(1.0f);                                                                                 // set the camera view: not used
        glm::mat4 proj = glm::perspective(glm::radians(m_fov), m_aspectRatio, m_zNear, m_zFar);                           // 3D alternative to "ortho" proj type. It allows to define the view frustum by inserting the y FOV, the aspect ratio of the window, where are placed the near and far clipping planes

        m_shader.Bind();                                                                                                  // bind shader
        m_shader.SetUniform4f("u_Color", m_r, m_g, m_b, m_alpha);                                                         // setup uniform
        m_shader.SetUniformMat4f("u_M", model);
        m_shader.SetUniformMat4f("u_V", view);
        m_shader.SetUniformMat4f("u_P", proj);

        renderer.Draw(m_va, m_ib, m_shader);
    }
    
    return m_texID;
}

void OpenGLQuadLayer::setOffsetPose(const Eigen::Vector3f& offset, const Eigen::Quaternionf& offsetQuat)
{
    setOffsetPosition(offset);
    setOffsetQuaternion(offsetQuat);
}

void OpenGLQuadLayer::setOffsetPosition(const Eigen::Vector3f& offset)                                                    // the offset vector must represent the position of the headset frame wrt the Left or Right Screen Frame. Both the Screen Frames are right-handed, have the origin at the center of the screen, the x to the right and the y pointing up.
{
    m_offsetTra[3][0] = offset(1);
    m_offsetTra[3][1] = offset(2);
    m_offsetTra[3][2] = offset(3);
}

void OpenGLQuadLayer::setOffsetQuaternion(const Eigen::Quaternionf& offsetQuat)
{
    glm::fquat qIn(1.0, 0.0, 0.0, 0.0);
    qIn.w = offsetQuat.w();
    qIn.x = offsetQuat.x();
    qIn.y = offsetQuat.y();
    qIn.z = offsetQuat.z();

    m_offsetRot = glm::mat4_cast(qIn);
}

void OpenGLQuadLayer::setPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &quaternion)
{
    setPosition(position);
    setQuaternion(quaternion);
}

void OpenGLQuadLayer::setPosition(const Eigen::Vector3f &position)
{
    m_modelTraEig = position;

    m_modelTra[3][0] = position(1);
    m_modelTra[3][1] = position(2);
    m_modelTra[3][2] = position(3);
}

void OpenGLQuadLayer::setQuaternion(const Eigen::Quaternionf &quaternion)
{
    m_modelRotEig = quaternion;

    glm::fquat qInput(1.0, 0.0, 0.0, 0.0);
    qInput.w = quaternion.w();
    qInput.x = quaternion.x();
    qInput.y = quaternion.y();
    qInput.z = quaternion.z();
    m_modelRot = glm::mat4_cast(qInput);
}

void OpenGLQuadLayer::setDimensions(float widthInMeters, float heightInMeters)
{
    m_modelScale.x = widthInMeters;
    m_modelScale.y = heightInMeters;
}

void OpenGLQuadLayer::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    m_visibility = visibility;
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
    m_texID = m_texture.GetTextureID();

    return true;
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
    return m_imageMaxHeight;
}

int32_t OpenGLQuadLayer::imageMaxWidth() const
{
    return m_imageMaxWidth;
}

float OpenGLQuadLayer::layerWidth() const
{
    return m_modelScale.x;
}

float OpenGLQuadLayer::layerHeight() const
{
    return m_modelScale.y;
}

Eigen::Vector3f OpenGLQuadLayer::layerPosition() const
{
    return m_modelTraEig;
}

Eigen::Quaternionf OpenGLQuadLayer::layerQuaternion() const
{
    return m_modelRotEig;
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
