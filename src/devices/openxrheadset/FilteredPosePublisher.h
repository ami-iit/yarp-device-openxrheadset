/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_FILTEREDPOSEPUBLISHER_H
#define YARP_DEV_FILTEREDPOSEPUBLISHER_H

#include <OpenXrInterface.h>
#include <yarp/dev/IFrameTransform.h>
#include <string>
#include <memory>

struct FilteredPosePublisherSettings
{
    yarp::dev::IFrameTransform* tfPublisher;
    std::string rootFrame;
    double period{0.01};

    struct ValidityChecks
    {
        double maxDistance{0.1};
        double maxAngularDistanceInRad{0.5};
        double lastDataExpirationTime{5.0};
        double maxConvergenceTime{3.0};
        double convergenceRatio{0.05};
    };

    ValidityChecks checks;
};

class FilteredPosePublisher
{

    std::string m_label;
    double m_lastWarningTime{0.0};
    size_t m_warningCount{0};
    bool m_publishedOnce{false};
    yarp::sig::Matrix m_localPose;
    bool m_active{false};
    OpenXrInterface::NamedPoseVelocity m_data;
    OpenXrInterface::NamedPoseVelocity m_lastValidData;
    std::shared_ptr<FilteredPosePublisherSettings> m_settings{nullptr};
    double m_lastValidDataTime;
    bool m_convergingToJump{false};

    bool positionJumped();

    bool rotationJumped();

    void filterJumps();

    void resetWarnings();

    void resetLastValidData();

    void publishOldTransform();

    void publishNewTransform();

    void deactivate();

public:
    FilteredPosePublisher();

    void setLabel(const std::string& label);

    void configure(std::shared_ptr<FilteredPosePublisherSettings> settings);

    bool configured() const;

    void update(const OpenXrInterface::NamedPoseVelocity& input);

    void publish();
};

#endif // YARP_DEV_FILTEREDPOSEPUBLISHER_H
