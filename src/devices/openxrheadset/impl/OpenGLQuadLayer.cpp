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
#include <QuadLayerShader.h>
#include <string>

bool OpenGLQuadLayer::initialize(int32_t imageMaxWidth, int32_t imageMaxHeight)
{
    m_imageMaxWidth = imageMaxWidth;
    m_imageMaxHeight = imageMaxHeight;

    m_positions = {
        // vertex coords         // texture coords
        -0.5f, -0.5f, 0.0, 1.0f, 0.0f, 0.0f, // 0 (the first 4 numbers of the row are the vertex coordinates, the second 2 numbers are the texture coordinate for that vertex (the bottom left corner of the rectangle is also the bottom left corner of the picture))
         0.5f, -0.5f, 0.0, 1.0f, 1.0f, 0.0f, // 1
         0.5f,  0.5f, 0.0, 1.0f, 1.0f, 1.0f, // 2
        -0.5f,  0.5f, 0.0, 1.0f, 0.0f, 1.0f  // 3
    };

    m_indices = { // creating an index buffer to save GPU memory
        0, 1, 2,
        2, 3, 0
    };

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // src (output color of the fragment shader) and dest (color already in the buffer) factors respectively (sfactor, dfactor)

    m_vb.setVertices(m_positions);

    VertexBufferLayout layout;
    layout.push<float>(4); // 4 floats for each vertex position
    layout.push<float>(2); // 2 floats for texture coordinates to be mapped on each vertex
    m_va.addBuffer(m_vb, layout);

    m_ib.setIndices(m_indices);

    m_shader.initializeFromString(QuadLayerShader::Content());
    m_shader.bind();

    m_userTexture.bindToFrameBuffer(m_userBuffer);
    m_userTexture.allocateTexture(imageMaxWidth, imageMaxHeight);
    m_userTexture.unbind();
    m_userBuffer.unbind();

    m_internalTexture.bindToFrameBuffer(m_internalBuffer);
    m_internalTexture.allocateTexture(imageMaxWidth, imageMaxHeight);

    m_shader.setUniform1i("u_Texture", 0); // the second argument must match the input of texture.Bind(input)
    m_shader.setUniform1i("u_UseAlpha", m_useAlpha);

    m_internalTexture.unbind();
    m_internalBuffer.unbind();

    /* unbinding everything */
    m_va.unbind();
    m_vb.unbind();
    m_ib.unbind();
    m_shader.unbind();

    return true;
}

void OpenGLQuadLayer::setFOVs(float fovX, float fovY)
{
    float tan_fovY_2 = std::tan(fovY/2);

    if (std::abs(tan_fovY_2) < 1e-15)
        return;

    m_fovY = fovY;
    m_aspectRatio = std::tan(fovX/2) / tan_fovY_2; //See https://en.wikipedia.org/wiki/Field_of_view_in_video_games
}

void OpenGLQuadLayer::setDepthLimits(float zNear, float zFar)
{
    m_zNear = zNear;
    m_zFar = zFar;
}

void OpenGLQuadLayer::render()
{
    Renderer renderer;

    m_internalTexture.bind();

    glm::mat4 modelPose = m_modelTra * m_modelRot;
    glm::mat4 sca = glm::scale(glm::mat4(1.0f), m_modelScale);

    glm::mat4 model = m_offsetTra * modelPose * sca;
    glm::mat4 proj = glm::perspective(m_fovY, m_aspectRatio, m_zNear, m_zFar);                           // 3D alternative to "ortho" proj type. It allows to define the view frustum by inserting the y FOV, the aspect ratio of the window, where are placed the near and far clipping planes

    glm::mat4 layerTransform = proj * model;

    m_shader.bind();                                                                                                  // bind shader
    m_shader.setUniformMat4f("u_H", layerTransform);
    m_shader.setUniform1i("u_UseAlpha", m_useAlpha);

    renderer.draw(m_va, m_ib, m_shader);
}

void OpenGLQuadLayer::setOffsetPosition(const Eigen::Vector3f& offset) // the offset vector must represent the position of the screen wrt the headset. Both the Screen Frames are right-handed, have the origin at the center of the screen, the x to the right and the y pointing up.
{
    //The sintax for glm::mat4 is [col][row]
    m_offsetTra[3][0] = -offset(0);
    m_offsetTra[3][1] = -offset(1);
    m_offsetTra[3][2] = -offset(2);
    m_offsetIsSet = true;
}

bool OpenGLQuadLayer::offsetIsSet() const
{
    return m_offsetIsSet;
}

Texture& OpenGLQuadLayer::getUserTexture()
{
    return m_userTexture;
}

const  IOpenXrQuadLayer::Visibility& OpenGLQuadLayer::visibility() const
{
    return m_visibility;
}

bool OpenGLQuadLayer::shouldRender() const
{
    return m_isEnabled && m_isReleased;
}

void OpenGLQuadLayer::setPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &quaternion)
{
    setPosition(position);
    setQuaternion(quaternion);
}

void OpenGLQuadLayer::setPosition(const Eigen::Vector3f &position)
{
    m_modelTraEig = position;
    //The sintax for glm::mat4 is [col][row]
    m_modelTra[3][0] = position(0);
    m_modelTra[3][1] = position(1);
    m_modelTra[3][2] = position(2);
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
    m_useAlpha = useAlphaChannel;
}

bool OpenGLQuadLayer::getImage(uint32_t &glImage)
{
    glImage = m_userTexture.getTextureID();

    return true;
}

bool OpenGLQuadLayer::submitImage()
{
    return submitImage(0, 0, imageMaxWidth(), imageMaxHeight());
}

bool OpenGLQuadLayer::submitImage(int32_t xOffset, int32_t yOffset, int32_t imageWidth, int32_t imageHeight)
{

    m_userTexture.bindToFrameBuffer(m_userBuffer);
    m_internalTexture.bindToFrameBuffer(m_internalBuffer);

    //Copy from the read framebuffer to the draw framebuffer
    glBlitNamedFramebuffer(m_userBuffer.id(), m_internalBuffer.id(), xOffset, yOffset, xOffset + imageWidth, yOffset + imageHeight,
        0, 0, imageMaxWidth(), imageMaxHeight(),
        GL_COLOR_BUFFER_BIT, GL_NEAREST);

//Resetting read and draw framebuffers
    m_userBuffer.unbind();
    m_internalBuffer.unbind();

    m_isReleased = true;

    return true;
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
    m_isEnabled = enabled;
}

OpenGLQuadLayer::OpenGLQuadLayer()
{
}

OpenGLQuadLayer::~OpenGLQuadLayer()
{
}
