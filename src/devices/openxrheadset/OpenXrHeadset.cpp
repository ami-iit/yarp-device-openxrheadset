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

bool yarp::dev::OpenXrHeadset::open(yarp::os::Searchable &cfg)
{
    yCTrace(OPENXRHEADSET);

    m_prefix = "/OpenXrHeadset";

    //checking all the parameter in the configuration file..
    {
        constexpr unsigned int STRING = 0;
        constexpr unsigned int BOOL   = 1;
        constexpr unsigned int INT    = 2;
        constexpr unsigned int DOUBLE = 3;

        std::map<int, std::string>                err_msgs;
        std::map<int, valueIsType>                isFunctionMap;
        std::vector<std::pair<std::string, int> > paramParser;

        err_msgs[STRING]      = "a string";
        err_msgs[BOOL]        = "a boolean type";
        err_msgs[INT]         = "an integer type";
        err_msgs[DOUBLE]      = "a real type";
        isFunctionMap[STRING] = &yarp::os::Value::isString;
        isFunctionMap[BOOL]   = &yarp::os::Value::isBool;
        isFunctionMap[INT]    = &yarp::os::Value::isInt32;
        isFunctionMap[DOUBLE] = &yarp::os::Value::isFloat64;

        int guiCount = cfg.find("gui_elements").asInt32();
        paramParser.clear();
        if (guiCount)
        {
            paramParser.push_back(std::make_pair("width",  DOUBLE));
            paramParser.push_back(std::make_pair("height", DOUBLE));
            paramParser.push_back(std::make_pair("x",      DOUBLE));
            paramParser.push_back(std::make_pair("y",      DOUBLE));
            paramParser.push_back(std::make_pair("z",      DOUBLE));

            for (int i = 0; i < guiCount; ++i)
            {
                std::string       groupName  = "GUI_" + std::to_string(i);
                yarp::os::Bottle& guip       = cfg.findGroup(groupName);

                if (guip.isNull())
                {
                    yCError(OPENXRHEADSET) << "group:" << groupName << "not found in configuration file..";
                    return false;
                }

                for (auto& p : paramParser)
                {
                    if (!guip.check(p.first) || !(guip.find(p.first).*isFunctionMap[p.second])())
                    {
                        std::string err_type = err_msgs.find(p.second) == err_msgs.end() ? "[unknow type]" : err_msgs[p.second];
                        yCError(OPENXRHEADSET) << "parameter" << p.first << "not found or not" << err_type << "in" << groupName << "group in configuration file";
                        return false;
                    }
                }

                huds.emplace_back();

                huds.back().width = guip.find("width").asFloat64();
                huds.back().height = guip.find("height").asFloat64();
                huds.back().x       = guip.find("x").asFloat64();
                huds.back().y       = guip.find("y").asFloat64();
                huds.back().z       = -std::max(0.01, std::abs(guip.find("z").asFloat64())); //make sure that z is negative and that is at least 0.01 in modulus
                std::transform(groupName.begin(), groupName.end(), groupName.begin(), ::tolower);
                huds.back().portName = m_prefix + "/" + groupName;
            }
        }
    }

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

    for (size_t eye = 0; eye < 2; ++eye) {
        if (!displayPorts[eye].initialize(openXrInterface.addHeadFixedQuadLayer(),
                                          (eye == 0 ? m_prefix + "/display/left:i" : m_prefix + "/display/right:i"))) {
            yCError(OPENXRHEADSET) << "Cannot initialize" << (eye == 0 ? "left" : "right") << "display texture.";
            return false;
        }
        displayPorts[eye].setVisibility(eye == 0 ? IOpenXrQuadLayer::Visibility::LEFT_EYE : IOpenXrQuadLayer::Visibility::RIGHT_EYE);
    }

    for (guiParam& gui : huds)
    {
        if (!gui.layer.initialize(openXrInterface.addHeadFixedQuadLayer(), gui.portName)) {
            yCError(OPENXRHEADSET) << "Cannot initialize" << gui.portName << "display texture.";
            return false;
        }
        gui.layer.setVisibility(IOpenXrQuadLayer::Visibility::BOTH_EYES);
        gui.layer.setDimensions(gui.width, gui.height);
        gui.layer.setPosition({gui.x, gui.y, gui.z});
    }

    return true;
}

void yarp::dev::OpenXrHeadset::threadRelease()
{
    yCTrace(OPENXRHEADSET);
    if (closed)
        return;

    openXrInterface.close();

    closed = true;
}

void yarp::dev::OpenXrHeadset::run()
{
    yCTrace(OPENXRHEADSET);

    if (openXrInterface.isRunning())
    {
        for (int eye = 0; eye < 2; ++eye) {
            if (!displayPorts[eye].updateTexture()) {
                yCError(OPENXRHEADSET) << "Failed to update" << (eye == 0 ? "left" : "right") << "display texture.";
                return;
            }
        }

        for (guiParam& gui : huds)
        {
            if (!gui.layer.updateTexture()) {
                yCError(OPENXRHEADSET) << "Failed to update" << gui.portName << "display texture.";
                return;
            }
        }
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

