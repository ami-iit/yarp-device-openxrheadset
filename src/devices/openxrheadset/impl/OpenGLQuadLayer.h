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
#include <FrameBuffer.h>


class OpenGLQuadLayer : public IOpenXrQuadLayer
{
    std::vector<float> m_positions;
    std::vector<unsigned int> m_indices;
    IOpenXrQuadLayer::Visibility m_visibility{ IOpenXrQuadLayer::Visibility::NONE };
    int32_t m_imageMaxWidth = 0;
    int32_t m_imageMaxHeight = 0;
    VertexArray m_va;
    VertexBuffer m_vb;
    IndexBuffer m_ib;
    Shader m_shader;
    Texture m_userTexture, m_internalTexture;
    FrameBuffer m_userBuffer;
    FrameBuffer m_internalBuffer;
    bool m_useAlpha{true};
    bool m_isEnabled{true};
    bool m_isReleased{false};

    glm::mat4 m_offsetTra = glm::mat4(1.0f);                                  // position of the Headset Frame WRT the Left or Right Screen Frame
    bool m_offsetIsSet{false};

    Eigen::Vector3f m_modelTraEig {0.0f, 0.0f, -1.0f};
    Eigen::Quaternionf m_modelRotEig {1.0f, 0.0f, 0.0f, 0.0f};
    glm::mat4 m_modelTra = glm::mat4(1.0f);
    glm::mat4 m_modelRot = glm::mat4(1.0f);

    glm::vec3 m_modelScale{1.0f, 1.0f, 1.0f};

    float m_zNear = 0.1f;
    float m_zFar = 100.0f;
    XrFovf m_fov;
    float m_aspectRatio = 1.0f;

public:

    OpenGLQuadLayer();

    ~OpenGLQuadLayer();

    OpenGLQuadLayer(const OpenGLQuadLayer&) = delete;

    OpenGLQuadLayer(OpenGLQuadLayer&&) = delete;

    OpenGLQuadLayer& operator=(const OpenGLQuadLayer&) = delete;

    OpenGLQuadLayer& operator=(OpenGLQuadLayer&&) = delete;

    bool initialize(int32_t imageMaxWidth, int32_t imageMaxHeight);

    void setFOVs(const XrFovf& fov);

    void setDepthLimits(float zNear, float zFar);

    void render();

    void setOffsetPosition(const Eigen::Vector3f& offset);

    bool offsetIsSet() const;

    Texture& getUserTexture();

    const  IOpenXrQuadLayer::Visibility& visibility() const;

    bool shouldRender() const;

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
