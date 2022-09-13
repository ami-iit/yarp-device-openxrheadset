/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXREIGENCONVERSIONS_H
#define YARP_DEV_OPENXREIGENCONVERSIONS_H

#include <OpenXrConfig.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


inline Eigen::Vector3f toEigen(const XrVector3f &vector)
{
    Eigen::Vector3f output;
    output[0] = vector.x;
    output[1] = vector.y;
    output[2] = vector.z;
    return output;
}

inline Eigen::Quaternionf toEigen(const XrQuaternionf &quaternion)
{
    Eigen::Quaternionf output;
    output.w() = quaternion.w;
    output.x() = quaternion.x;
    output.y() = quaternion.y;
    output.z() = quaternion.z;
    return output;
}

inline Eigen::Matrix4f toEigen(const XrPosef &pose)
{
    Eigen::Matrix4f output;
    output.setIdentity();
    output.block<3, 3>(0, 0) = toEigen(pose.orientation).toRotationMatrix();
    output.block<3, 1>(0, 3) = toEigen(pose.position);
    return output;
}

inline XrVector3f toXr(const Eigen::Vector3f &position)
{
    XrVector3f output;
    output.x = position[0];
    output.y = position[1];
    output.z = position[2];
    return output;
}

inline XrQuaternionf toXr(const Eigen::Quaternionf &quaternion)
{
    XrQuaternionf output;
    output.w = quaternion.w();
    output.x = quaternion.x();
    output.y = quaternion.y();
    output.z = quaternion.z();
    return output;
}

inline XrPosef toXr(const Eigen::Matrix4f &pose)
{
    XrPosef output;

    output.orientation = toXr(Eigen::Quaternionf(pose.block<3, 3>(0, 0)));
    output.position = toXr(Eigen::Vector3f(pose.block<3, 1>(0, 3)));

    return output;
}

#endif // YARP_DEV_OPENXREIGENCONVERSIONS_H
