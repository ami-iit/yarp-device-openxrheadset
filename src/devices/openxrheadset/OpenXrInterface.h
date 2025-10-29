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
#include <vector>
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
                         const Eigen::Quaternionf &quaternion) = 0;

    virtual void setPosition(const Eigen::Vector3f& position) = 0;

    virtual void setQuaternion(const Eigen::Quaternionf &quaternion) = 0;

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

    virtual void setEnabled(bool enabled) = 0;
};

enum class PoseFilterType
{
    NONE,
    JUMP_FILTER
};

struct OpenXrInterfaceSettings
{
    double posesPredictionInMs{0.0};
    bool hideWindow{false};
    bool renderInPlaySpace{false};
    bool useGaze{ true };
    bool useExpressions{ true };
};

class OpenXrInterface
{
    class Implementation;

    std::unique_ptr<Implementation> m_pimpl;

    bool checkExtensions();

    bool prepareXrInstance();

    void printInstanceProperties();

    bool prepareXrSystem();

    void checkSystemProperties();

    bool prepareGL();

    bool prepareXrSession();

    bool prepareXrSwapchain();

    bool prepareXrCompositionLayers();

    bool prepareXrActions();

    bool prepareGlFramebuffer();

    void pollXrEvents();

    bool startXrFrame();

    void updateXrSpaces();

    void updateXrActions();

    bool updateInteractionProfiles();

    void printInteractionProfiles();

    bool updateConnectedTrackers();

    void forceTrackersInteractionProfile();

    void render(double drawableArea);

    void endXrFrame();

public:

    struct Pose
    {
        bool positionValid{false};
        bool rotationValid{false};

        Eigen::Vector3f position;
        Eigen::Quaternionf rotation;

        Pose operator*(const Pose& other) const;
    };

    struct Velocity
    {
        bool linearValid{false};
        bool angularValid{false};

        Eigen::Vector3f linear;
        Eigen::Vector3f angular;
    };

    struct NamedPoseVelocity
    {
        std::string name;
        std::string parentFrame;
        Pose pose;
        Velocity velocity;
        PoseFilterType filterType{ PoseFilterType::JUMP_FILTER };

        static NamedPoseVelocity Identity(const std::string& name);
    };

    OpenXrInterface();

    ~OpenXrInterface();

    OpenXrInterface(const OpenXrInterface& other) = delete;

    OpenXrInterface(OpenXrInterface&& other) = delete;

    OpenXrInterface& operator=(const OpenXrInterface& other) = delete;

    OpenXrInterface& operator=(OpenXrInterface&& other) = delete;

    bool initialize(const OpenXrInterfaceSettings& settings = OpenXrInterfaceSettings());

    bool isInitialized() const;

    void draw(double drawableArea = 1.0);

    std::shared_ptr<IOpenXrQuadLayer> addHeadFixedQuadLayer();

    std::shared_ptr<IOpenXrQuadLayer> addHeadFixedOpenGLQuadLayer();

    bool isRunning() const;

    float ipd() const;

    Pose headPose() const;

    Velocity headVelocity() const;

    Pose leftHandPose() const;

    Velocity leftHandVelocity() const;

    Pose rightHandPose() const;

    Velocity rightHandVelocity() const;

    std::string currentLeftHandInteractionProfile() const;

    std::string currentRightHandInteractionProfile() const;

    void getButtons(std::vector<bool>& buttons) const;

    void getAxes(std::vector<float>& axes) const;

    void getThumbsticks(std::vector<Eigen::Vector2f>& thumbsticks) const;

    void getAllPoses(std::vector<NamedPoseVelocity> &additionalPoses) const;

    int64_t currentNanosecondsSinceEpoch() const;

    bool shouldResetLocalReferenceSpace();

    bool eyeExpressionsSupported() const;

    bool lipExpressionsSupported() const;

    const std::vector<float>& eyeExpressions() const;

    const std::vector<float>& lipExpressions() const;

    bool gazeSupported() const;

    Pose gazePoseInViewFrame() const;

    void close();
};

#endif // YARP_DEV_OPENXRINTERFACE_H
