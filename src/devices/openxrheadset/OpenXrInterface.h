/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRINTERFACE_H
#define YARP_DEV_OPENXRINTERFACE_H

#include <memory>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>

class IOpenXrQuadLayer
{
public:

    enum class Visibility
    {
        LEFT_EYE,
        RIGHT_EYE,
        BOTH_EYES,
        NONE
    };

    virtual void setPose(const Eigen::Vector3f& position,
                         const Eigen::Quaternionf &rotation) = 0;

    virtual void setPosition(const Eigen::Vector3f& position) = 0;

    virtual void setRotation(const Eigen::Quaternionf &rotation) = 0;

    virtual void setDimensions(float widthInMeters, float heightInMeters) = 0;

    virtual void setVisibility(const Visibility& visibility) = 0;

    virtual void useAlphaChannel(bool useAlphaChannel = true) = 0;

    virtual bool getImage(uint32_t& glImage) = 0;

    virtual bool submitImage() = 0;

    virtual bool submitImage(int32_t xOffset, int32_t yOffset, int32_t imageWidth, int32_t imageHeight) = 0;

    virtual int32_t imageMaxHeight() const = 0;

    virtual int32_t imageMaxWidth() const = 0;

    virtual float layerWidth() const = 0;

    virtual float layerHeight() const = 0;

    virtual Eigen::Vector3f layerPosition() const = 0;

    virtual Eigen::Quaternionf layerQuaternion() const = 0;
};

class OpenXrInterface
{
    class Implementation;

    std::unique_ptr<Implementation> m_pimpl;

    bool checkExtensions();

    bool prepareXrInstance();

    void printInstanceProperties();

    bool prepareXrSystem();

    void printSystemProperties();

    bool prepareWindow();

    bool prepareXrSession();

    bool prepareXrSwapchain();

    bool prepareXrCompositionLayers();

    bool prepareXrActions();

    bool prepareGlFramebuffer();

    void pollXrEvents();

    bool startXrFrame();

    void updateXrViews();

    void updateXrActions();

    void render();

    void endXrFrame();

public:

    OpenXrInterface();

    ~OpenXrInterface();

    OpenXrInterface(const OpenXrInterface& other) = delete;

    OpenXrInterface(OpenXrInterface&& other) = delete;

    OpenXrInterface& operator=(const OpenXrInterface& other) = delete;

    OpenXrInterface& operator=(OpenXrInterface&& other) = delete;

    void setKeyCallback(std::function<void(int /*key*/, int /*scancode*/, int /*action*/, int /*mods*/)> keyCallback);

    bool initialize();

    bool isInitialized() const;

    void draw();

    std::shared_ptr<IOpenXrQuadLayer> addHeadFixedQuadLayer();

    bool isRunning() const;

    void close();
};

#endif // YARP_DEV_OPENXRINTERFACE_H
