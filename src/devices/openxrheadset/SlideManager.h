/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef YARP_DEV_SLIDEMANAGER_H
#define YARP_DEV_SLIDEMANAGER_H

#include <ImagePortToQuadLayer.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <string>
#include <unordered_map>

class SlideManager
{
public:

    struct Options
    {
        std::shared_ptr<IOpenXrQuadLayer> quadLayer;
        std::string slidesPath;
        std::string initialSlide;
        std::string portName;

        bool parseFromConfigurationFile(const std::string &inputPortName, yarp::os::Searchable& labelGroup);
    };

    bool initialize(const Options& options);

    void close();

    bool updateTexture();

    void setPose(const Eigen::Vector3f& position,
                 const Eigen::Quaternionf &rotation);

    void setPosition(const Eigen::Vector3f& position);

    Eigen::Vector3f layerPosition() const;

    void setQuaternion(const Eigen::Quaternionf &quaternion);

    Eigen::Quaternionf layerQuaternion() const;

    void setDimensions(float widthInMeters, float heightInMeters);

    void setVisibility(const IOpenXrQuadLayer::Visibility& visibility);

    float layerWidth() const;

    float layerHeight() const;

    void setEnabled(bool enabled);

    bool setImage(const std::string& imagename);

private:

    ImagePortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgba>> m_layer;
    std::unordered_map<std::string, yarp::sig::ImageOf<yarp::sig::PixelRgba>> m_loadedImages;
    std::shared_ptr<yarp::os::BufferedPort<yarp::os::Bottle>> m_portPtr;
    Options m_options;
    std::vector<unsigned char*> m_stbImages;
};

#endif // YARP_DEV_SLIDEMANAGER_H
