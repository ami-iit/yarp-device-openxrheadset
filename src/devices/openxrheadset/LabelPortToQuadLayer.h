﻿/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_OPENXRHEADSET_LABELPORTTOQUADLAYER_H
#define YARP_OPENXRHEADSET_LABELPORTTOQUADLAYER_H

#include <OpenGLConfig.h>

#include <GLFont/GLFont.h>

#include <Eigen/Geometry>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>

#include <thread>
#include <cassert>

#include <OpenXrInterface.h>
#include <OpenXrHeadsetLogComponent.h>

class LabelPortToQuadLayer
{
public:

    struct Options
    {
        enum class VerticalAlignement
        {
            Top,
            Center,
            Bottom
        };

        enum class HorizontalAlignement
        {
            Left,
            Center,
            Right
        };

        bool automaticallyEnabled{true};
        bool followEyes{false};
        double disableTimeoutInS{-1.0};
        std::shared_ptr<IOpenXrQuadLayer> quadLayer{nullptr};
        std::string portName;
        std::string labelPrefix;
        std::string labelSuffix;
        std::string fontPath{"Roboto/Roboto-Light.ttf"};
        int pixelSize{64};
        Eigen::Vector4f backgroundColor;
        Eigen::Vector4f labelColor;
        VerticalAlignement verticalAlignement{VerticalAlignement::Center};
        HorizontalAlignement horizontalAlignement{HorizontalAlignement::Center};

        bool parseFromConfigurationFile(const std::string &inputPortName, yarp::os::Searchable& labelGroup);
    };

    bool initialize(const Options& options);

    void close();

    bool updateTexture();

    void setPose(const Eigen::Vector3f& position,
                         const Eigen::Quaternionf &rotation);

    void setPosition(const Eigen::Vector3f& position);

    Eigen::Vector3f layerPosition() const;

    void setQuaternion(const Eigen::Quaternionf &quaternion);

    Eigen::Quaternionf layerQuaternion() const;

    void setDimensions(float widthInMeters, float heightInMeters);

    void setVisibility(const IOpenXrQuadLayer::Visibility& visibility);

    float layerWidth() const;

    float layerHeight() const;

    void setEnabled(bool enabled);

private:

    Options m_options;
    std::shared_ptr<yarp::os::BufferedPort<yarp::os::Bottle>> m_portPtr;
    std::shared_ptr<GLFont> m_glFont;
    std::shared_ptr<FTLabel> m_glLabel;
    GLuint m_glWriteBufferId = 0;
    GLuint m_glReadBufferId = 0;
    GLuint m_imageTexture;
    std::thread::id m_initThreadID;
    std::string m_inputString;
    bool m_enabled{true};
    bool m_firstTime{true};
    bool m_timeoutExpired{false};
    bool m_shouldResume{false};
    double m_lastReceived{0.0};
};

#endif // YARP_OPENXRHEADSET_LABELPORTTOQUADLAYER_H
