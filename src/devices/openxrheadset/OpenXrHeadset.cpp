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

#include <OpenXrHeadset.h>
#include <OpenXrHeadsetLogComponent.h>

typedef bool(yarp::os::Value::*valueIsType)(void) const;

void poseToYarpMatrix(const OpenXrInterface::Pose& input, yarp::sig::Matrix& output)
{
    Eigen::Matrix3f rotationMatrix = input.rotation.toRotationMatrix();
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            output(i,j) = rotationMatrix(i,j);
        }
        output(i, 3) = input.position(i);
    }
}


yarp::dev::OpenXrHeadset::OpenXrHeadset()
    : yarp::dev::DeviceDriver(),
      yarp::os::PeriodicThread(0.011, yarp::os::ShouldUseSystemClock::Yes) // ~90 fps
{
    yCTrace(OPENXRHEADSET);

}

yarp::dev::OpenXrHeadset::~OpenXrHeadset()
{
    yCTrace(OPENXRHEADSET);
    this->stop();
}

bool yarp::dev::OpenXrHeadset::open(yarp::os::Searchable &cfg)
{
    yCTrace(OPENXRHEADSET);

    std::string name = cfg.check("name", yarp::os::Value("OpenXrHeadset")).toString();
    if (name.front() != '/')
    {
        m_prefix = '/' + name;
    }
    else
    {
        m_prefix = name;
    }

    //checking the additional guis parameter in the configuration file..
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

    getStickAsAxis = cfg.check("stick_as_axis", yarp::os::Value(false)).asBool();
    left_frame     = cfg.check("tf_left_hand_frame", yarp::os::Value("openxr_left_hand")).asString();
    right_frame    = cfg.check("tf_right_hand_frame", yarp::os::Value("openxr_right_hand")).asString();
    head_frame     = cfg.check("tf_head_frame", yarp::os::Value("openxr_head")).asString();
    root_frame     = cfg.check("tf_root_frame", yarp::os::Value("openxr_origin")).asString();


    //opening tf client
    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", cfg.check("tfDevice", yarp::os::Value("transformClient")).asString());
    tfClientCfg.put("local",  cfg.check("tfLocal", yarp::os::Value(m_prefix + "/tf")).asString());
    tfClientCfg.put("remote", cfg.check("tfRemote", yarp::os::Value("/transformServer")).asString());

    if (!driver.open(tfClientCfg))
    {
        yCError(OPENXRHEADSET) << "Unable to open polydriver with the following options:" << tfClientCfg.toString();
        return false;
    }

    if (!driver.view(tfPublisher) || tfPublisher == nullptr)
    {
        yCError(OPENXRHEADSET) << "Unable to view IFrameTransform interface.";
        return false;
    }
    yCInfo(OPENXRHEADSET) << "TransformCLient successfully opened at port: " << cfg.find("tfLocal").asString();

    headPose.resize(4,4);
    headPose.eye();
    leftHandPose.resize(4,4);
    leftHandPose.eye();
    rightHandPose.resize(4,4);
    rightHandPose.eye();

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

    if (tfPublisher)
    {
        driver.close();
        tfPublisher = nullptr;
    }

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

        {
            std::lock_guard<std::mutex> lock(m_mutex);

            openXrInterface.getButtons(buttons);
            openXrInterface.getAxes(axes);
            openXrInterface.getThumbsticks(thumbsticks);

            OpenXrInterface::Pose xrHead = openXrInterface.headPose();
            if (xrHead.positionValid && xrHead.rotationValid)
            {
                poseToYarpMatrix(xrHead, headPose);
                tfPublisher->setTransform(head_frame, root_frame, headPose);
            }

            OpenXrInterface::Pose xrLeftHand = openXrInterface.leftHandPose();
            if (xrLeftHand.positionValid && xrLeftHand.rotationValid)
            {
                poseToYarpMatrix(xrLeftHand, leftHandPose);
                tfPublisher->setTransform(left_frame, root_frame, leftHandPose);
            }

            OpenXrInterface::Pose xrRightHand = openXrInterface.rightHandPose();
            if (xrRightHand.positionValid && xrRightHand.rotationValid)
            {
                poseToYarpMatrix(xrRightHand, rightHandPose);
                tfPublisher->setTransform(right_frame, root_frame, rightHandPose);
            }
        }
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

bool yarp::dev::OpenXrHeadset::getAxisCount(unsigned int &axis_count)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    axis_count = axes.size();

    if (getStickAsAxis)
    {
        axis_count += 2 * thumbsticks.size();
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getButtonCount(unsigned int &button_count)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    button_count = buttons.size();

    return true;
}

bool yarp::dev::OpenXrHeadset::getTrackballCount(unsigned int &trackball_count)
{
    yCTrace(OPENXRHEADSET);

    trackball_count = 0;

    return true;
}

bool yarp::dev::OpenXrHeadset::getHatCount(unsigned int &hat_count)
{
    yCTrace(OPENXRHEADSET);

    hat_count = 0; //These are handled as buttons in OpenXR

    return true;
}

bool yarp::dev::OpenXrHeadset::getTouchSurfaceCount(unsigned int &touch_count)
{
    yCTrace(OPENXRHEADSET);

    touch_count = 0;

    return true;
}

bool yarp::dev::OpenXrHeadset::getStickCount(unsigned int &stick_count)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    if (getStickAsAxis)
    {
        stick_count = 0;
    }
    else
    {
        stick_count = thumbsticks.size();
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getStickDoF(unsigned int stick_id, unsigned int &dof)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    dof = 2; //Al thumbsticks have two degrees of freedom in OpenXR

    if (getStickAsAxis)
    {
        yCError(OPENXRHEADSET) << "The sticks are considered as axis, so there are none.";
        return false;
    }
    else if (stick_id >= thumbsticks.size())
    {
        yCError(OPENXRHEADSET) << "The stick_id" << stick_id << "is out of bound. Only" << thumbsticks.size() << "sticks are available." ;
        return false;
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getButton(unsigned int button_id, float &value)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    if (button_id < buttons.size())
    {
        value = buttons[button_id];
    }
    else
    {
        yCError(OPENXRHEADSET) << "Requested button with index" << button_id << ", but there are" << buttons.size() << "buttons.";
        return false;
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getTrackball(unsigned int /*trackball_id*/, yarp::sig::Vector &value)
{
    yCTrace(OPENXRHEADSET);
    value.zero();
    yCError(OPENXRHEADSET) << "No trackball are considered in this device.";
    return false;
}

bool yarp::dev::OpenXrHeadset::getHat(unsigned int /*hat_id*/, unsigned char &value)
{
    yCTrace(OPENXRHEADSET);
    value = 0;
    yCError(OPENXRHEADSET) << "No hats are considered in this device.";
    return false;
}

bool yarp::dev::OpenXrHeadset::getAxis(unsigned int axis_id, double &value)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    unsigned int inputId = axis_id;

    if (inputId < axes.size())
    {
        value = axes[inputId];
    }
    else
    {
        if (getStickAsAxis)
        {
            inputId -= axes.size();
            if (inputId < 2 * thumbsticks.size())
            {
                unsigned int thumbstickId = inputId / 2;

                value = thumbsticks[thumbstickId][inputId - thumbstickId]; //Each thumbstick counts as two axes
            }
            else
            {
                yCError(OPENXRHEADSET) << "The axis_id" << axis_id << "is out of bounds. There are"
                                       << axes.size() << "axes and" << thumbsticks.size()
                                       << "thumbsticks (counting as two axes each).";
                return false;
            }
        }
        else
        {
            yCError(OPENXRHEADSET) << "The axis_id" << axis_id << "is out of bounds. There are"
                                   << axes.size() << "axes.";
            return false;
        }
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getStick(unsigned int stick_id, yarp::sig::Vector &value,
                                        yarp::dev::IJoypadController::JoypadCtrl_coordinateMode coordinate_mode)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    if (getStickAsAxis)
    {
        yCError(OPENXRHEADSET) << "The sticks are considered axis, so there are none";
        return false;
    }

    if (stick_id < thumbsticks.size())
    {
        value.resize(2);
        if (coordinate_mode == JoypadCtrl_coordinateMode::JypCtrlcoord_POLAR)
        {
            value[0] = thumbsticks[stick_id].norm();
            value[1] = atan2(thumbsticks[stick_id][1], thumbsticks[stick_id][0]);
        }
        else
        {
            value[0] = thumbsticks[stick_id][0];
            value[1] = thumbsticks[stick_id][1];
        }
    }
    else
    {
        yCError(OPENXRHEADSET) << "The stick_id" << stick_id << "is out of bound. Only" << thumbsticks.size() << "sticks are available." ;
        return false;
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getTouch(unsigned int /*touch_id*/, yarp::sig::Vector &value)
{
    yCTrace(OPENXRHEADSET);
    value.clear();
    yCError(OPENXRHEADSET) << "No touch devices are considered in this device.";
    return false;
}
