/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_ADDITIONALPOSESPUBLISHER_H
#define YARP_DEV_ADDITIONALPOSESPUBLISHER_H

#include <OpenXrInterface.h>
#include <PosePublisher.h>
#include <yarp/dev/IFrameTransform.h>
#include <string>

class AdditionalPosesPublisher
{
    std::shared_ptr<PosePublisherSettings> m_settings{nullptr};

    std::vector<OpenXrInterface::NamedPoseVelocity> m_additionalPosesInputList;

    std::unordered_map<std::string, PosePublisher> m_additionalPoses;

public:

    struct Label
    {
        std::string original;
        std::string modified;
    };

    void initialize(const std::vector<Label> &labels,
                    const PosePublisherSettings &settings);

    std::vector<OpenXrInterface::NamedPoseVelocity>& inputs();

    void publishFrames();

};


#endif // YARP_DEV_ADDITIONALPOSESPUBLISHER_H
