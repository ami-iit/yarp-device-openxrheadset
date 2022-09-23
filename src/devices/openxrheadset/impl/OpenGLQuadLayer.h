/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENGLQUADLAYER_H
#define YARP_DEV_OPENGLQUADLAYER_H

#include <vector>

#include <OpenXrConfig.h>
#include <OpenXrInterface.h>

#include <Renderer.h>
#include <VertexBuffer.h>
#include <VertexBufferLayout.h>
#include <IndexBuffer.h>
#include <VertexArray.h>
#include <Shader.h>
#include <Texture.h>


class OpenGLQuadLayer : public IOpenXrQuadLayer
{
    std::vector<float> m_positions;
    std::vector<unsigned int> m_indices;
    VertexArray m_va;
    VertexBuffer m_vb;
    IndexBuffer m_ib;
    Shader m_shader;
    Texture m_texture;
    unsigned int m_texID{ 0 };

    float m_r = 0.0f;
    float m_g = 0.0f;
    float m_b = 0.0f;
    float m_alpha = 1.0f;
    float m_increment = 0.05f;

    glm::vec3 m_modelTranslation = glm::vec3( 0.0f, 0.0f,-3.0f);
    glm::vec3 m_modelRotation    = glm::vec3( 0.0f, 0.0f, 0.0f);              // rotation angles: degrees
    glm::vec3 m_modelScale       = glm::vec3( 1.0f, 1.0f, 1.0f);
    float m_fov = 60.0f;                                                      // Field Of View

    float m_zNear = 0.1f;
    float m_zFar = 100.0f;
    float m_aspectRatio = 1.0f;

public:

    OpenGLQuadLayer();

    ~OpenGLQuadLayer();

    OpenGLQuadLayer(const OpenGLQuadLayer&) = delete;

    OpenGLQuadLayer(OpenGLQuadLayer&&) = delete;

    OpenGLQuadLayer& operator=(const OpenGLQuadLayer&) = delete;

    OpenGLQuadLayer& operator=(OpenGLQuadLayer&&) = delete;

    bool initialize();

    bool setRenderAspectRatio(float aspectRatio);

    bool setRenderFov(float fov);

    bool setRenderDepth(float zNear, float zFar);

    bool setRenderPose(glm::vec3 modelTranslation, glm::vec3 modelRotation, glm::vec3 modelScale);

    bool setRenderColor(float r, float g, float b, float alpha);

    unsigned int render();

    virtual void setPose(const Eigen::Vector3f& position,
                         const Eigen::Quaternionf &quaternion) override;

    virtual void setPosition(const Eigen::Vector3f& position) override;

    virtual void setQuaternion(const Eigen::Quaternionf &quaternion) override;

    virtual void setDimensions(float widthInMeters, float heightInMeters) override;

    virtual void setVisibility(const Visibility& visibility) override;

    virtual void useAlphaChannel(bool useAlphaChannel = true) override;

    virtual bool getImage(uint32_t& glImage) override;

    virtual bool submitImage() override;

    virtual bool submitImage(int32_t xOffset, int32_t yOffset, int32_t imageWidth, int32_t imageHeight) override;

    virtual int32_t imageMaxHeight() const override;

    virtual int32_t imageMaxWidth() const override;

    virtual float layerWidth() const override;

    virtual float layerHeight() const override;

    virtual Eigen::Vector3f layerPosition() const override;

    virtual Eigen::Quaternionf layerQuaternion() const override;

    virtual void setEnabled(bool enabled) override;
};

#endif // YARP_DEV_OPENGLQUADLAYER_H
