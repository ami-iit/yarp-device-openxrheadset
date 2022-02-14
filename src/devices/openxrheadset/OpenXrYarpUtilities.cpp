/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <OpenXrYarpUtilities.h>

void poseToYarpMatrix(const Eigen::Vector3f &inputPosition, const Eigen::Quaternionf &inputQuaternion, yarp::sig::Matrix &output)
{
    Eigen::Matrix3f rotationMatrix = inputQuaternion.toRotationMatrix();
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            output(i,j) = rotationMatrix(i,j);
        }
        output(i, 3) = inputPosition(i);
    }
}

void poseToYarpMatrix(const OpenXrInterface::Pose &input, yarp::sig::Matrix &output)
{
    poseToYarpMatrix(input.position, input.rotation, output);
}

void writeVec3OnPort(yarp::os::BufferedPort<yarp::os::Bottle> * const &port, const Eigen::Vector3f &vec3, yarp::os::Stamp &stamp)
{
    if (port)
    {
        yarp::os::Bottle& output = port->prepare();
        output.clear();
        output.addFloat64(vec3[0]);
        output.addFloat64(vec3[1]);
        output.addFloat64(vec3[2]);
        port->setEnvelope(stamp);
        port->write();
    }
}

void writeQuaternionOnPort(yarp::os::BufferedPort<yarp::os::Bottle> * const &port, const Eigen::Quaternionf &quat, yarp::os::Stamp &stamp)
{
    if (port)
    {
        yarp::os::Bottle& output = port->prepare();
        output.clear();
        output.addFloat64(quat.w());
        output.addFloat64(quat.x());
        output.addFloat64(quat.y());
        output.addFloat64(quat.z());
        port->setEnvelope(stamp);
        port->write();
    }
}
