/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_POSEPUBLISHER_H
#define YARP_DEV_POSEPUBLISHER_H

#include <OpenXrInterface.h>
#include <yarp/dev/IFrameTransform.h>
#include <string>
#include <memory>

struct PosePublisherSettings
{
    yarp::dev::IFrameTransform* tfPublisher;
    std::string rootFrame;
};

class PosePublisher
{
    std::string m_label;
    OpenXrInterface::NamedPoseVelocity m_data;
    yarp::sig::Matrix m_localPose;
    std::shared_ptr<PosePublisherSettings> m_baseSettings{nullptr};
    bool m_active{false};
    bool m_publishedOnce{false};
    double m_lastWarningTime{0.0};
    size_t m_warningCount{0};

    void resetWarnings();

protected:

    virtual void deactivate();

public:

    PosePublisher();

    void configurePublisher(std::shared_ptr<PosePublisherSettings> settings);

    void setLabel(const std::string& label);

    const std::string& label() const;

    virtual bool configured() const;

    virtual void updateInputPose(const OpenXrInterface::NamedPoseVelocity& input);

    void publish();

    bool isActive() const;
};

#endif // YARP_DEV_POSEPUBLISHER_H
