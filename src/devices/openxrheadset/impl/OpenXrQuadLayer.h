/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRQUADLAYER_H
#define YARP_DEV_OPENXRQUADLAYER_H

#include <vector>

#include <OpenXrConfig.h>
#include <OpenXrInterface.h>

class OpenXrQuadLayer : public IOpenXrQuadLayer
{
public:

    XrCompositionLayerQuad layer;
    std::vector<XrSwapchainImageOpenGLKHR> swapchain_images;
    uint32_t swapchainWidth;
    uint32_t swapchainHeight;
    bool hasVisibility{true};
    unsigned int acquired{0};
    bool released{false};
    bool isEnabled{true};

    OpenXrQuadLayer();

    ~OpenXrQuadLayer();

    OpenXrQuadLayer(const OpenXrQuadLayer&) = delete;

    OpenXrQuadLayer(OpenXrQuadLayer&&) = delete;

    OpenXrQuadLayer& operator=(const OpenXrQuadLayer&) = delete;

    OpenXrQuadLayer& operator=(OpenXrQuadLayer&&) = delete;

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

    bool shouldSubmit() const;
};

#endif // YARP_DEV_OPENXRQUADLAYER_H
