/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRHEADSET_H
#define YARP_DEV_OPENXRHEADSET_H

#include <vector>
#include <atomic>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ServiceInterfaces.h>
#include <yarp/sig/Image.h>
#include "OpenXrInterface.h"
#include "PortToQuadLayer.h"

namespace yarp {
namespace dev {

class OpenXrHeadset;

}
}

class yarp::dev::OpenXrHeadset : public yarp::dev::DeviceDriver,
                                 public yarp::os::PeriodicThread,
                                 public yarp::dev::IService
{
public:
    OpenXrHeadset();

    virtual ~OpenXrHeadset();

    // yarp::dev::DeviceDriver methods
    virtual bool open(yarp::os::Searchable& cfg) override;
    virtual bool close() override;

    // yarp::os::RateThread methods
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

    //  yarp::dev::IService methods
    virtual bool startService() override;
    virtual bool updateService() override;
    virtual bool stopService() override;

private:

    struct guiParam
    {
        float         width;
        float         height;
        float         x;
        float         y;
        float         z;
        std::string    portName;
        PortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgba>> layer;
    };

    std::string m_prefix;

    std::array<PortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgb>>, 2> displayPorts;

    std::vector<guiParam> huds;

    std::atomic_bool closed{ false };

    OpenXrInterface openXrInterface;

};

#endif // YARP_DEV_OPENXRHEADSET_H
