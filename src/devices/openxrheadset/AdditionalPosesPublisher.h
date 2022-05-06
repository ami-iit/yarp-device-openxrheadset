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
#include <yarp/dev/IFrameTransform.h>
#include <string>

class AdditionalPosesPublisher
{
    yarp::dev::IFrameTransform* m_tfPublisher;
    std::string m_rootFrame;

    std::vector<OpenXrInterface::NamedPoseVelocity> m_additionalPosesInputList;

    struct AdditionalPoseInfo
    {
        std::string label;
        double lastWarningTime{0.0};
        size_t warningCount{0};
        bool publishedOnce{false};
        yarp::sig::Matrix localPose;
        bool active{false};
        OpenXrInterface::NamedPoseVelocity data;
    };
    std::unordered_map<std::string, AdditionalPoseInfo> m_additionalPoses;

public:

    struct Label
    {
        std::string original;
        std::string modified;
    };

    void initialize(yarp::dev::IFrameTransform* tfPublisher,
                    const std::vector<Label> &labels,
                    const std::string& rootFrame);

    std::vector<OpenXrInterface::NamedPoseVelocity>& inputs();

    void publishFrames();

};


#endif // YARP_DEV_ADDITIONALPOSESPUBLISHER_H
