/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRYARPUTILITIES_H
#define YARP_DEV_OPENXRYARPUTILITIES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yarp/sig/Matrix.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Stamp.h>
#include <OpenXrInterface.h>

void poseToYarpMatrix(const Eigen::Vector3f& inputPosition, const Eigen::Quaternionf& inputQuaternion, yarp::sig::Matrix& output);

void poseToYarpMatrix(const OpenXrInterface::Pose& input, yarp::sig::Matrix& output);

void writeVec3OnPort(yarp::os::BufferedPort<yarp::os::Bottle>*const & port, const Eigen::Vector3f& vec3, yarp::os::Stamp& stamp);

void writeQuaternionOnPort(yarp::os::BufferedPort<yarp::os::Bottle>*const & port,
                           const Eigen::Quaternionf& vec3, yarp::os::Stamp& stamp);

#endif // YARP_DEV_OPENXRYARPUTILITIES_H
