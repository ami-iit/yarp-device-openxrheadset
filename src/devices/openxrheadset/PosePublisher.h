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
    yarp::dev::IFrameTransform* tfPublisher{nullptr};
    std::string tfBaseFrame;
};

class PosePublisher
{
    std::string m_label, m_tfBaseFrame;
    OpenXrInterface::NamedPoseVelocity m_data, m_previouslyPublishedData;
    yarp::sig::Matrix m_localPose;
    yarp::dev::IFrameTransform* m_tfPublisher{nullptr};
    bool m_active{false};
    bool m_publishedOnce{false};
    double m_lastWarningTime{0.0};
    size_t m_warningCount{0};

    void resetWarnings();

    bool usePreviousPose() const;

    bool canPublish() const;

protected:

    virtual void deactivate();

public:

    PosePublisher();

    void configurePublisher(std::shared_ptr<PosePublisherSettings> settings);

    void setLabel(const std::string& label);

    const std::string& label() const;

    const std::string& tfBaseFrame() const;

    virtual bool configured() const;

    virtual void updateInputPose(const OpenXrInterface::NamedPoseVelocity& input);

    OpenXrInterface::NamedPoseVelocity data() const;

    void publish();

};

#endif // YARP_DEV_POSEPUBLISHER_H
