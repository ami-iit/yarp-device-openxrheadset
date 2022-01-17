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
#include <OpenXrYarpUtilities.h>

typedef bool(yarp::os::Value::*valueIsType)(void) const;

bool yarp::dev::OpenXrHeadset::FramePorts::open(const std::string &name, const std::string &portPrefix, IFrameTransform *tfPublisher, const std::string &tfFrame, const std::string &rootFrame)
{
    //opening ports
    std::initializer_list<std::pair<yarp::os::BufferedPort<yarp::os::Bottle>**,
                                    std::string>> ports =
    {
        { &m_orientationPort,                  "quaternion"                   },
        { &m_positionPort,                     "position"                     },
        { &m_angularVelocityPort,              "angularVelocity"              },
        { &m_linearVelocityPort,               "linearVelocity"               }
    };

    for (auto port : ports)
    {
        if (*port.first)
        {
            yCError(OPENXRHEADSET) << port.second <<  "is already open.";
            continue;
        }

        std::string portName;

        *port.first = new yarp::os::BufferedPort<yarp::os::Bottle>;
        portName        = portPrefix + "/" + port.second + ":o";

        if (!(*port.first)->open(portName))
        {
            yCError(OPENXRHEADSET) << "Cannot open" << portName << "port";
            close();
            return false;
        }

        (*port.first)->setWriteOnly();
    }

    double timeNow = yarp::os::Time::now();
    m_lastWarning["quaternion"] = timeNow - 10.0;
    m_lastWarning["position"] = timeNow - 10.0;
    m_lastWarning["angularVelocity"] = timeNow - 10.0;
    m_lastWarning["linearVelocity"] = timeNow - 10.0;
    m_lastWarning["republish"] = timeNow - 10.0;

    m_name = name;
    m_tfPublisher = tfPublisher;
    m_tfFrame = tfFrame;
    m_rootFrame = rootFrame;

    m_localPose.resize(4,4);
    m_localPose.eye();

    return true;
}

void yarp::dev::OpenXrHeadset::FramePorts::close()
{
    m_localPoseValid = false;

    //Closing and deleting ports
    std::initializer_list<yarp::os::BufferedPort<yarp::os::Bottle>**> ports =
    {
        &m_orientationPort,
        &m_positionPort,
        &m_angularVelocityPort,
        &m_linearVelocityPort,
    };

    for (auto port : ports)
    {
        if (*port)
        {
            (*port)->close();

            delete *port;

            *port = nullptr;
        }
    }
}

void yarp::dev::OpenXrHeadset::FramePorts::publishFrame(const OpenXrInterface::Pose &pose, const OpenXrInterface::Velocity &velocity, os::Stamp &stamp)
{
    if (pose.positionValid && pose.rotationValid)
    {
        poseToYarpMatrix(pose, m_localPose);
        m_localPoseValid = true;
        if (!m_tfPublisher->setTransform(m_tfFrame, m_rootFrame, m_localPose))
        {
            yCWarning(OPENXRHEADSET) << "Failed to publish" << m_tfFrame << "frame.";
        }
    }
    else
    {
        if (m_localPoseValid)
        {
            if (!m_tfPublisher->setTransform(m_tfFrame, m_rootFrame, m_localPose))
            {
                yCWarning(OPENXRHEADSET) << "Failed to publish" << m_tfFrame << "frame.";
            }

            if (yarp::os::Time::now() - m_lastWarning["republish"] > 1.0)
            {
                yCWarning(OPENXRHEADSET) << "Publishing last" << m_name << "known pose.";
                m_lastWarning["republish"] = yarp::os::Time::now();
            }
        }
    }

    if (pose.positionValid)
    {
        writeVec3OnPort(m_positionPort, pose.position, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["position"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "position not valid.";
            m_lastWarning["position"] = yarp::os::Time::now();
        }
    }

    if (pose.rotationValid)
    {
        writeQuaternionOnPort(m_orientationPort, pose.rotation, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["quaternion"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "rotation not valid.";
            m_lastWarning["quaternion"] = yarp::os::Time::now();
        }
    }

    if (velocity.linearValid)
    {
        writeVec3OnPort(m_linearVelocityPort, velocity.linear, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["linearVelocity"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "linear velocity not valid.";
            m_lastWarning["linearVelocity"] = yarp::os::Time::now();
        }
    }

    if (velocity.angularValid)
    {
        writeVec3OnPort(m_angularVelocityPort, velocity.angular, stamp);
    }
    else
    {
        if (yarp::os::Time::now() - m_lastWarning["angularVelocity"] > 5.0)
        {
            yCWarning(OPENXRHEADSET) << m_name << "angular velocity not valid.";
            m_lastWarning["angularVelocity"] = yarp::os::Time::now();
        }
    }
}

yarp::dev::OpenXrHeadset::OpenXrHeadset()
    : yarp::dev::DeviceDriver(),
      yarp::os::PeriodicThread(0.011, yarp::os::ShouldUseSystemClock::Yes), // ~90 fps
      m_stamp(0,0.0)
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
        struct paramDescription
        {
            std::string name;
            int type;
        };

        std::map<int, std::string>                err_msgs;
        std::map<int, valueIsType>                isFunctionMap;
        std::vector<paramDescription> paramParser;

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
            paramParser.push_back({"width",  DOUBLE});
            paramParser.push_back({"height", DOUBLE});
            paramParser.push_back({"x",      DOUBLE});
            paramParser.push_back({"y",      DOUBLE});
            paramParser.push_back({"z",      DOUBLE});

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
                    if (!guip.check(p.name) || !(guip.find(p.name).*isFunctionMap[p.type])())
                    {
                        std::string err_type = err_msgs.find(p.type) == err_msgs.end() ? "[unknow type]" : err_msgs[p.type];
                        yCError(OPENXRHEADSET) << "parameter" << p.name << "not found or not" << err_type << "in" << groupName << "group in configuration file";
                        return false;
                    }
                }

                m_huds.emplace_back();

                m_huds.back().width = guip.find("width").asFloat64();
                m_huds.back().height = guip.find("height").asFloat64();
                m_huds.back().x       = guip.find("x").asFloat64();
                m_huds.back().y       = guip.find("y").asFloat64();
                m_huds.back().z       = -std::max(0.01, std::abs(guip.find("z").asFloat64())); //make sure that z is negative and that is at least 0.01 in modulus
                std::transform(groupName.begin(), groupName.end(), groupName.begin(), ::tolower);
                m_huds.back().portName = m_prefix + "/" + groupName;
            }
        }

        paramParser.clear();
        int labelCount = cfg.find("labels").asInt32();
        if (labelCount)
        {
            paramParser.push_back({"width",  DOUBLE});
            paramParser.push_back({"height", DOUBLE});
            paramParser.push_back({"x",      DOUBLE});
            paramParser.push_back({"y",      DOUBLE});
            paramParser.push_back({"z",      DOUBLE});

            for (int i = 0; i < labelCount; ++i)
            {
                std::string       groupName  = "LABEL_" + std::to_string(i);
                yarp::os::Bottle& labelGroup = cfg.findGroup(groupName);

                if (labelGroup.isNull())
                {
                    yCError(OPENXRHEADSET) << "group:" << groupName << "not found in configuration file..";
                    return false;
                }

                for (auto& p : paramParser)
                {
                    if (!labelGroup.check(p.name) || !(labelGroup.find(p.name).*isFunctionMap[p.type])())
                    {
                        std::string err_type = err_msgs.find(p.type) == err_msgs.end() ? "[unknow type]" : err_msgs[p.type];
                        yCError(OPENXRHEADSET) << "parameter" << p.name << "not found or not" << err_type << "in" << groupName << "group in configuration file";
                        return false;
                    }
                }

                m_labels.emplace_back();
                LabelLayer& label = m_labels.back();

                label.width = labelGroup.find("width").asFloat64();
                label.height = labelGroup.find("height").asFloat64();
                label.x       = labelGroup.find("x").asFloat64();
                label.y       = labelGroup.find("y").asFloat64();
                label.z       = -std::max(0.01, std::abs(labelGroup.find("z").asFloat64())); //make sure that z is negative and that is at least 0.01 in modulus
                std::transform(groupName.begin(), groupName.end(), groupName.begin(), ::tolower);
                label.options.portName = m_prefix + "/" + groupName;
                label.options.labelPrefix = labelGroup.check("prefix", yarp::os::Value("")).asString();
                label.options.labelSuffix = labelGroup.check("suffix", yarp::os::Value("")).asString();
                label.options.fontPath = labelGroup.check("font", yarp::os::Value("Roboto/Roboto-Black.ttf")).asString();
                label.options.pixelSize = labelGroup.check("pixel_size", yarp::os::Value(64)).asInt32();
                label.options.automaticallyEnabled = labelGroup.check("automatically_enabled", yarp::os::Value(true)).asBool();
                label.options.disableTimeoutInS = labelGroup.check("disable_timeout_in_S", yarp::os::Value(-1.0)).asFloat64();

                std::string horizontalAlignement = labelGroup.check("horizontal_alignement", yarp::os::Value("center")).asString();
                std::transform(horizontalAlignement.begin(), horizontalAlignement.end(), horizontalAlignement.begin(), ::tolower);
                if (horizontalAlignement == "left")
                {
                    label.options.horizontalAlignement = LabelPortToQuadLayer::Options::HorizontalAlignement::Left;
                }
                else if (horizontalAlignement == "right")
                {
                    label.options.horizontalAlignement = LabelPortToQuadLayer::Options::HorizontalAlignement::Right;
                }
                else if (horizontalAlignement == "center")
                {
                    label.options.horizontalAlignement = LabelPortToQuadLayer::Options::HorizontalAlignement::Center;
                }
                else
                {
                    yCError(OPENXRHEADSET) << "Unrecognized horizontal_alignement in" << groupName + "."
                                           << "Allowed entries: \"left\", \"right\", \"center\".";
                    return false;
                }

                std::string verticalAlignement = labelGroup.check("vertical_alignement", yarp::os::Value("center")).asString();
                std::transform(verticalAlignement.begin(), verticalAlignement.end(), verticalAlignement.begin(), ::tolower);
                if (verticalAlignement == "top")
                {
                    label.options.verticalAlignement = LabelPortToQuadLayer::Options::VerticalAlignement::Top;
                }
                else if (verticalAlignement == "bottom")
                {
                    label.options.verticalAlignement = LabelPortToQuadLayer::Options::VerticalAlignement::Bottom;
                }
                else if (verticalAlignement == "center")
                {
                    label.options.verticalAlignement = LabelPortToQuadLayer::Options::VerticalAlignement::Center;
                }
                else
                {
                    yCError(OPENXRHEADSET) << "Unrecognized vertical_alignement in" << groupName + "."
                                           << "Allowed entries: \"top\", \"bottom\", \"center\".";
                    return false;
                }

                auto fetchColor = [](const std::string& groupName, const yarp::os::Bottle& group,
                        const std::string& paramName, const Eigen::Vector4f& defaultColor, Eigen::Vector4f& outputColor) -> bool
                {
                    if (!group.check(paramName))
                    {
                        outputColor = defaultColor;
                        return true;
                    }

                    yarp::os::Value list = group.find(paramName);
                    if (!list.isList())
                    {
                        yCError(OPENXRHEADSET) << "The parameter" << paramName << "is specified in" << groupName << "but it is not a list.";
                        return false;
                    }

                    yarp::os::Bottle* yarpVector = list.asList();

                    if (yarpVector->size() != 4)
                    {
                        yCError(OPENXRHEADSET) << "The parameter" << paramName << "is specified in" << groupName << "but it is not a list of size 4.";
                        return false;
                    }

                    for (size_t i = 0; i < 4; ++i)
                    {
                        if (!yarpVector->get(i).isFloat64())
                        {
                            yCError(OPENXRHEADSET) << "The entry" << i << " of parameter" << paramName << "in" << groupName << "is not a double.";
                            return false;
                        }
                        double value = yarpVector->get(i).asFloat64();

                        if (value < 0.0 || value > 1.0)
                        {
                            yCError(OPENXRHEADSET) << "The entry" << i << " of parameter" << paramName << "in" << groupName << "is not in the range [0, 1].";
                            return false;
                        }
                        outputColor[i] = value;
                    }

                    return true;
                };

                if (!fetchColor(groupName, labelGroup, "color", {1.0, 1.0, 1.0, 1.0}, label.options.labelColor))
                {
                    return false;
                }

                if (!fetchColor(groupName, labelGroup, "background_color",  {0.0, 0.0, 0.0, 0.0}, label.options.backgroundColor))
                {
                    return false;
                }
            }
        }
    }

    m_getStickAsAxis       = cfg.check("stick_as_axis", yarp::os::Value(false)).asBool();
    m_leftFrame            = cfg.check("tf_left_hand_frame", yarp::os::Value("openxr_left_hand")).asString();
    m_rightFrame           = cfg.check("tf_right_hand_frame", yarp::os::Value("openxr_right_hand")).asString();
    m_headFrame            = cfg.check("tf_head_frame", yarp::os::Value("openxr_head")).asString();
    m_leftEyeFrame         = cfg.check("tf_left_eye_frame", yarp::os::Value("openxr_left_eye")).asString();
    m_rightEyeFrame        = cfg.check("tf_right_eye_frame", yarp::os::Value("openxr_right_eye")).asString();
    m_rootFrame            = cfg.check("tf_root_frame", yarp::os::Value("openxr_origin")).asString();
    m_leftAzimuthOffset    = cfg.check("left_azimuth_offset", yarp::os::Value(0.0)).asFloat64();
    m_leftElevationOffset  = cfg.check("left_elevation_offset", yarp::os::Value(0.0)).asFloat64();
    m_eyeZPosition         = -std::max(0.01, std::abs(cfg.check("eye_z_position", yarp::os::Value(-1.0)).asFloat64())); //make sure that z is negative and that is at least 0.01 in modulus
    m_interCameraDistance  = std::abs(cfg.check("inter_camera_distance", yarp::os::Value(0.07)).asFloat64()); //Distance between the cameras of the iCub robot
    m_rightAzimuthOffset   = cfg.check("right_azimuth_offset", yarp::os::Value(0.0)).asFloat64();
    m_rightElevationOffset = cfg.check("right_elevation_offset", yarp::os::Value(0.0)).asFloat64();

    //opening tf client
    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", cfg.check("tfDevice", yarp::os::Value("transformClient")).asString());
    tfClientCfg.put("local",  cfg.check("tfLocal", yarp::os::Value(m_prefix + "/tf")).asString());
    tfClientCfg.put("remote", cfg.check("tfRemote", yarp::os::Value("/transformServer")).asString());

    if (!m_driver.open(tfClientCfg))
    {
        yCError(OPENXRHEADSET) << "Unable to open polydriver with the following options:" << tfClientCfg.toString();
        return false;
    }

    if (!m_driver.view(m_tfPublisher) || m_tfPublisher == nullptr)
    {
        yCError(OPENXRHEADSET) << "Unable to view IFrameTransform interface.";
        return false;
    }
    yCInfo(OPENXRHEADSET) << "TransformCLient successfully opened at port: " << cfg.find("tfLocal").asString();

    if (!m_headFramePorts.open("Head", m_prefix + "/headpose", m_tfPublisher, m_headFrame, m_rootFrame))
    {
        return false;
    }

    if (!m_leftHandFramePorts.open("Left Hand", m_prefix + "/left_hand", m_tfPublisher, m_leftFrame, m_rootFrame))
    {
        return false;
    }

    if (!m_rightHandFramePorts.open("Right Hand", m_prefix + "/right_hand", m_tfPublisher, m_rightFrame, m_rootFrame))
    {
        return false;
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

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_openXrInterface.initialize())
        {
            yCError(OPENXRHEADSET) << "Failed to initialize OpenXr interface.";
            return false;
        }

        if (!m_leftEye.open(m_openXrInterface.addHeadFixedQuadLayer(),
                            m_prefix + "/display/left:i",
                            m_prefix + "/eyeAngles/left:i",
                            m_tfPublisher, m_leftEyeFrame, m_headFrame)) {
            yCError(OPENXRHEADSET) << "Cannot initialize left display texture.";
            return false;
        }
        m_leftEye.setVisibility(IOpenXrQuadLayer::Visibility::LEFT_EYE);
        m_leftEye.setEyePosition(Eigen::Vector3f(-m_interCameraDistance / 2.0, 0.0, 0.0));
        m_leftEye.setEyeRotationOffset(m_leftAzimuthOffset, m_leftElevationOffset);
        m_leftEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_eyeZPosition));

        if (!m_rightEye.open(m_openXrInterface.addHeadFixedQuadLayer(),
                             m_prefix + "/display/right:i",
                             m_prefix + "/eyeAngles/right:i",
                             m_tfPublisher, m_rightEyeFrame, m_headFrame)) {
            yCError(OPENXRHEADSET) << "Cannot initialize right display texture.";
            return false;
        }
        m_rightEye.setVisibility(IOpenXrQuadLayer::Visibility::RIGHT_EYE);
        m_rightEye.setEyePosition(Eigen::Vector3f(m_interCameraDistance / 2.0, 0.0, 0.0));
        m_rightEye.setEyeRotationOffset(m_rightAzimuthOffset, m_rightElevationOffset);
        m_rightEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_eyeZPosition));

        for (GuiParam& gui : m_huds)
        {
            if (!gui.layer.initialize(m_openXrInterface.addHeadFixedQuadLayer(), gui.portName)) {
                yCError(OPENXRHEADSET) << "Cannot initialize" << gui.portName << "display texture.";
                return false;
            }
            gui.layer.setVisibility(IOpenXrQuadLayer::Visibility::BOTH_EYES);
            gui.layer.setDimensions(gui.width, gui.height);
            gui.layer.setPosition({gui.x, gui.y, gui.z});
        }

        for (LabelLayer& label : m_labels)
        {
            label.options.quadLayer = m_openXrInterface.addHeadFixedQuadLayer();

            if (!label.layer.initialize(label.options)) {
                yCError(OPENXRHEADSET) << "Cannot initialize" << label.options.portName << "label.";
                return false;
            }
            label.layer.setVisibility(IOpenXrQuadLayer::Visibility::BOTH_EYES);
            label.layer.setDimensions(label.width, label.height);
            label.layer.setPosition({label.x, label.y, label.z});
        }
    }

    for (size_t i = 0; i < 10 && m_openXrInterface.isRunning(); ++i)
    {
        run(); //dry run. This is to make sure that the number of buttons is correctly retrieved by the JoypadControlServer
        yarp::os::Time::delay(this->getPeriod());
    }

    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(m_prefix + "/rpc"))
    {
        yCError(OPENXRHEADSET) << "Could not open" << m_prefix + "/rpc" << " RPC port.";
        return false;
    }

    return true;
}

void yarp::dev::OpenXrHeadset::threadRelease()
{
    yCTrace(OPENXRHEADSET);
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_closed)
        return;

    for (auto& hud : m_huds)
    {
        hud.layer.close();
    }

    for (auto& label : m_labels)
    {
        label.layer.close();
    }

    m_openXrInterface.close();

    if (m_tfPublisher)
    {
        m_driver.close();
        m_tfPublisher = nullptr;
    }

    m_headFramePorts.close();
    m_leftHandFramePorts.close();
    m_rightHandFramePorts.close();
    m_leftEye.close();
    m_rightEye.close();

    m_rpcPort.close();

    m_closed = true;
}

void yarp::dev::OpenXrHeadset::run()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_openXrInterface.isRunning())
    {
        if (!m_leftEye.update()) {
            yCError(OPENXRHEADSET) << "Failed to update left eye.";
            return;
        }

        if (!m_rightEye.update()) {
            yCError(OPENXRHEADSET) << "Failed to update right eye.";
            return;
        }

        for (GuiParam& gui : m_huds)
        {
            if (!gui.layer.updateTexture()) {
                yCError(OPENXRHEADSET) << "Failed to update" << gui.portName << "display texture.";
                return;
            }
        }
        for (LabelLayer& label : m_labels)
        {
            if (!label.layer.updateTexture()) {
                yCError(OPENXRHEADSET) << "Failed to update" << label.options.portName << "display texture.";
                return;
            }
        }
        m_openXrInterface.draw();

        m_openXrInterface.getButtons(m_buttons);
        m_openXrInterface.getAxes(m_axes);
        m_openXrInterface.getThumbsticks(m_thumbsticks);

        m_stamp.update(m_openXrInterface.currentNanosecondsSinceEpoch() * 1e-9);

        m_headFramePorts.publishFrame(m_openXrInterface.headPose(), m_openXrInterface.headVelocity(), m_stamp);
        m_leftHandFramePorts.publishFrame(m_openXrInterface.leftHandPose(), m_openXrInterface.leftHandVelocity(), m_stamp);
        m_rightHandFramePorts.publishFrame(m_openXrInterface.rightHandPose(), m_openXrInterface.rightHandVelocity(), m_stamp);

        m_leftEye.publishEyeTransform();
        m_rightEye.publishEyeTransform();

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
    return !m_closed;
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

    axis_count = m_axes.size();

    if (m_getStickAsAxis)
    {
        axis_count += 2 * m_thumbsticks.size();
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getButtonCount(unsigned int &button_count)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    button_count = m_buttons.size();

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

    if (m_getStickAsAxis)
    {
        stick_count = 0;
    }
    else
    {
        stick_count = m_thumbsticks.size();
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getStickDoF(unsigned int stick_id, unsigned int &dof)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    dof = 2; //Al thumbsticks have two degrees of freedom in OpenXR

    if (m_getStickAsAxis)
    {
        yCError(OPENXRHEADSET) << "The sticks are considered as axis, so there are none.";
        return false;
    }
    else if (stick_id >= m_thumbsticks.size())
    {
        yCError(OPENXRHEADSET) << "The stick_id" << stick_id << "is out of bound. Only" << m_thumbsticks.size() << "sticks are available." ;
        return false;
    }

    return true;
}

bool yarp::dev::OpenXrHeadset::getButton(unsigned int button_id, float &value)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    if (button_id < m_buttons.size())
    {
        value = m_buttons[button_id];
    }
    else
    {
        yCError(OPENXRHEADSET) << "Requested button with index" << button_id << ", but there are" << m_buttons.size() << "buttons.";
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

    if (inputId < m_axes.size())
    {
        value = m_axes[inputId];
    }
    else
    {
        if (m_getStickAsAxis)
        {
            inputId -= m_axes.size();
            if (inputId < 2 * m_thumbsticks.size())
            {
                unsigned int thumbstickId = inputId / 2;

                value = m_thumbsticks[thumbstickId][inputId % 2]; //Each thumbstick counts as two axes
            }
            else
            {
                yCError(OPENXRHEADSET) << "The axis_id" << axis_id << "is out of bounds. There are"
                                       << m_axes.size() << "axes and" << m_thumbsticks.size()
                                       << "thumbsticks (counting as two axes each).";
                return false;
            }
        }
        else
        {
            yCError(OPENXRHEADSET) << "The axis_id" << axis_id << "is out of bounds. There are"
                                   << m_axes.size() << "axes.";
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

    if (m_getStickAsAxis)
    {
        yCError(OPENXRHEADSET) << "The sticks are considered axis, so there are none";
        return false;
    }

    if (stick_id < m_thumbsticks.size())
    {
        value.resize(2);
        if (coordinate_mode == JoypadCtrl_coordinateMode::JypCtrlcoord_POLAR)
        {
            value[0] = m_thumbsticks[stick_id].norm();
            value[1] = atan2(m_thumbsticks[stick_id][1], m_thumbsticks[stick_id][0]);
        }
        else
        {
            value[0] = m_thumbsticks[stick_id][0];
            value[1] = m_thumbsticks[stick_id][1];
        }
    }
    else
    {
        yCError(OPENXRHEADSET) << "The stick_id" << stick_id << "is out of bound. Only" << m_thumbsticks.size() << "sticks are available." ;
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

std::string yarp::dev::OpenXrHeadset::getInteractionProfile()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    return m_openXrInterface.currentHandInteractionProfile();
}

std::vector<double> yarp::dev::OpenXrHeadset::getLeftImageDimensions()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<double> output(2);
    output[0] = m_leftEye.layerWidth();
    output[1] = m_leftEye.layerHeight();

    return output;
}

std::vector<double> yarp::dev::OpenXrHeadset::getRightImageDimensions()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<double> output(2);
    output[0] = m_rightEye.layerWidth();
    output[1] = m_rightEye.layerHeight();

    return output;
}

std::vector<double> yarp::dev::OpenXrHeadset::getLeftImageAnglesOffsets()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<double> output(2);
    output[0] = m_leftEye.azimuthOffset();
    output[1] = m_leftEye.elevationOffset();

    return output;
}

std::vector<double> yarp::dev::OpenXrHeadset::getRightImageAnglesOffsets()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<double> output(2);
    output[0] = m_rightEye.azimuthOffset();
    output[1] = m_rightEye.elevationOffset();

    return output;
}

bool yarp::dev::OpenXrHeadset::setLeftImageAnglesOffsets(const double azimuth, const double elevation)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    m_leftEye.setEyeRotationOffset(azimuth, elevation);

    return true;
}

bool yarp::dev::OpenXrHeadset::setRightImageAnglesOffsets(const double azimuth, const double elevation)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    m_rightEye.setEyeRotationOffset(azimuth, elevation);

    return true;
}

bool yarp::dev::OpenXrHeadset::isLeftEyeActive()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    return m_leftEye.active();
}

bool yarp::dev::OpenXrHeadset::isRightEyeActive()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    return m_rightEye.active();
}

double yarp::dev::OpenXrHeadset::getEyesZPosition()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyeZPosition;
}

bool yarp::dev::OpenXrHeadset::setEyesZPosition(const double eyesZPosition)
{

    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    m_eyeZPosition = -std::max(0.01, std::abs(eyesZPosition));

    m_leftEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_eyeZPosition));
    m_rightEye.setEyeRelativeImagePosition(Eigen::Vector3f(0.0, 0.0, m_eyeZPosition));

    return true;
}

double yarp::dev::OpenXrHeadset::getInterCameraDistance()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    return m_interCameraDistance;
}

bool yarp::dev::OpenXrHeadset::setInterCameraDistance(const double distance)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);

    m_interCameraDistance = std::abs(distance);

    m_leftEye.setEyePosition(Eigen::Vector3f(-m_interCameraDistance / 2.0, 0.0, 0.0));
    m_rightEye.setEyePosition(Eigen::Vector3f(m_interCameraDistance / 2.0, 0.0, 0.0));

    return true;
}

std::string yarp::dev::OpenXrHeadset::getLeftImageControlPortName()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_leftEye.controlPortName();
}

std::string yarp::dev::OpenXrHeadset::getRightImageControlPortName()
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rightEye.controlPortName();
}

bool yarp::dev::OpenXrHeadset::setLabelEnabled(const int32_t labelIndex, const bool enabled)
{
    yCTrace(OPENXRHEADSET);

    std::lock_guard<std::mutex> lock(m_mutex);
    if (labelIndex >= m_labels.size())
    {
        return false;
    }

    m_labels[labelIndex].layer.setEnabled(enabled);

    return true;
}
