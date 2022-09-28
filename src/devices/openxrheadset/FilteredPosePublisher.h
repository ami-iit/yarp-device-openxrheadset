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
#include <PosePublisher.h>

struct FilteredPosePublisherSettings : public PosePublisherSettings
{
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

class FilteredPosePublisher : public PosePublisher
{

    OpenXrInterface::NamedPoseVelocity m_filteredData;
    OpenXrInterface::NamedPoseVelocity m_lastValidData;
    std::shared_ptr<FilteredPosePublisherSettings> m_settings{nullptr};
    double m_lastValidDataTime;
    bool m_convergingToJump{false};

    bool positionJumped(const OpenXrInterface::NamedPoseVelocity& input);

    bool rotationJumped(const OpenXrInterface::NamedPoseVelocity& input);

    OpenXrInterface::NamedPoseVelocity filterJumps(const OpenXrInterface::NamedPoseVelocity& input);

    void resetLastValidData();

    virtual void deactivate() override;

public:

    void configure(std::shared_ptr<FilteredPosePublisherSettings> settings);

    virtual bool configured() const override;

    virtual void updateInputPose(const OpenXrInterface::NamedPoseVelocity& input) override;
};

#endif // YARP_DEV_FILTEREDPOSEPUBLISHER_H
