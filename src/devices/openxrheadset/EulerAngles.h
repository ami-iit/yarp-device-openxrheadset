/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef YARP_DEV_EULERANGLES_H
#define YARP_DEV_EULERANGLES_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <string>


enum class RotationAxis : Eigen::Index
{
    X = 0,
    Y = 1,
    Z = 2
};

namespace EulerAngles
{

Eigen::Vector3f XYZ(const Eigen::Matrix3f& r);

Eigen::Vector3f XZY(const Eigen::Matrix3f& r);

Eigen::Vector3f YXZ(const Eigen::Matrix3f& r);

Eigen::Vector3f YZX(const Eigen::Matrix3f& r);

Eigen::Vector3f ZXY(const Eigen::Matrix3f& r);

Eigen::Vector3f ZYX(const Eigen::Matrix3f& r);

bool isParametrizationAvailable(const std::string& parametrization);

}

Eigen::Vector3f eulerAngles(const std::array<RotationAxis, 3>& parametrization, const Eigen::Quaternionf& rotation);



#endif // YARP_DEV_EULERANGLES_H
