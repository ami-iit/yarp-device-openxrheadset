/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/LogStream.h>

#include <map>
#include <algorithm>
#include <csignal>

#include "OpenXrHeadset.h"
#include "OpenXrHeadsetLogComponent.h"

typedef bool(yarp::os::Value::*valueIsType)(void) const;


yarp::dev::OpenXrHeadset::OpenXrHeadset()
    : yarp::dev::DeviceDriver(),
      yarp::os::PeriodicThread(0.011, yarp::os::ShouldUseSystemClock::Yes) // ~90 fps
{
    yCTrace(OPENXRHEADSET);

}

yarp::dev::OpenXrHeadset::~OpenXrHeadset()
{
    yCTrace(OPENXRHEADSET);
}

bool yarp::dev::OpenXrHeadset::open(yarp::os::Searchable &/*cfg*/)
{
    yCTrace(OPENXRHEADSET);

    // Start the thread
    if (!this->start()) {
        yCError(OPENXRHEADSET) << "Thread start failed, aborting.";
        this->close();
        return false;
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::close()
{
    yCTrace(OPENXRHEADSET);
    this->askToStop();
    return true;
}

bool yarp::dev::OpenXrHeadset::threadInit()
{
    yCTrace(OPENXRHEADSET);
    if (!openXrInterface.initialize())
    {
        yCError(OPENXRHEADSET) << "Failed to initialize OpenXr interface.";
        return false;
    }

    return true;
}

void yarp::dev::OpenXrHeadset::threadRelease()
{
    yCTrace(OPENXRHEADSET);
    if (closed)
        return;

    openXrInterface.close();

    std::raise(SIGTERM);

    closed = true;
}

void yarp::dev::OpenXrHeadset::run()
{
    yCTrace(OPENXRHEADSET);

    if (openXrInterface.isRunning())
    {
        openXrInterface.draw();
    }
    else
    {
        close();
        return;
    }
}

bool yarp::dev::OpenXrHeadset::startService()
{
    yCTrace(OPENXRHEADSET);
    //To let the device driver knowing that it need to poll updateService continuosly
    return false;
}

bool yarp::dev::OpenXrHeadset::updateService()
{
    yCTrace(OPENXRHEADSET);
    //To let the device driver that we are still alive
    return !closed;
}

bool yarp::dev::OpenXrHeadset::stopService()
{
    yCTrace(OPENXRHEADSET);
    return this->close();
}

