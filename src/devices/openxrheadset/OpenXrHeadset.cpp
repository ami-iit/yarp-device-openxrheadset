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

yarp::dev::OpenXrHeadset::OpenXrHeadset()
    : yarp::dev::DeviceDriver(),
      yarp::os::PeriodicThread(0.011, yarp::os::ShouldUseSystemClock::Yes) // ~90 fps
{}

yarp::dev::OpenXrHeadset::~OpenXrHeadset()
{
    this->stop();
}

bool yarp::dev::OpenXrHeadset::open(yarp::os::Searchable &cfg)
{
    std::string name = cfg.check("name", yarp::os::Value("OpenXrHeadset")).toString();
    if (name.front() != '/')
    {
        m_prefix = '/' + name;
    }
    else
    {
        m_prefix = name;
    }

    std::string rpcName = cfg.check("rpc_name", yarp::os::Value("rpc")).toString();
    if (rpcName.front() != '/')
    {
        m_rpcPortName = m_prefix +  '/' + rpcName;
    }
    else
    {
        m_rpcPortName = m_prefix + rpcName;
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

                std::string visibilityString = guip.check("visibility", yarp::os::Value("both")).asString();
                IOpenXrQuadLayer::Visibility visibility;
                std::transform(visibilityString.begin(), visibilityString.end(), visibilityString.begin(), ::tolower);
                if (visibilityString == "left")
                {
                    visibility = IOpenXrQuadLayer::Visibility::LEFT_EYE;
                }
                else if (visibilityString == "right")
                {
                    visibility = IOpenXrQuadLayer::Visibility::RIGHT_EYE;
                }
                else if (visibilityString == "both")
                {
                    visibility = IOpenXrQuadLayer::Visibility::BOTH_EYES;
                }
                else if (visibilityString == "none")
                {
                    visibility = IOpenXrQuadLayer::Visibility::NONE;
                }
                else
                {
                    yCError(OPENXRHEADSET) << "Unrecognized visibility."
                                           << "Allowed entries: \"left\", \"right\", \"both\", \"none\".";
                    return false;
                }

                m_huds.emplace_back();

                m_huds.back().width = guip.find("width").asFloat64();
                m_huds.back().height = guip.find("height").asFloat64();
                m_huds.back().x       = guip.find("x").asFloat64();
                m_huds.back().y       = guip.find("y").asFloat64();
                m_huds.back().z       = -std::max(0.01, std::abs(guip.find("z").asFloat64())); //make sure that z is negative and that is at least 0.01 in modulus
                std::transform(groupName.begin(), groupName.end(), groupName.begin(), ::tolower);
                m_huds.back().portName = m_prefix + "/" + guip.check("port_id", yarp::os::Value(groupName)).toString();
                m_huds.back().followEyes = guip.check("follow_eyes") && (guip.find("follow_eyes").isNull() || guip.find("follow_eyes").asBool());
                m_huds.back().visibility = visibility;
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
                std::string portName = m_prefix + "/" + labelGroup.check("port_id", yarp::os::Value(groupName)).toString();

                if (!label.options.parseFromConfigurationFile(portName, labelGroup))
                {
                    yCError(OPENXRHEADSET) << "Failed to parse" << groupName;
                    return false;
                }
            }
        }

        paramParser.clear();
        int slideCount = cfg.find("slides").asInt32();
        if (slideCount)
        {
            paramParser.push_back({"width",  DOUBLE});
            paramParser.push_back({"height", DOUBLE});
            paramParser.push_back({"x",      DOUBLE});
            paramParser.push_back({"y",      DOUBLE});
            paramParser.push_back({"z",      DOUBLE});

            for (int i = 0; i < slideCount; ++i)
            {
                std::string       groupName  = "SLIDE_" + std::to_string(i);
                yarp::os::Bottle& slideGroup = cfg.findGroup(groupName);

                if (slideGroup.isNull())
                {
                    yCError(OPENXRHEADSET) << "group:" << groupName << "not found in configuration file..";
                    return false;
                }

                for (auto& p : paramParser)
                {
                    if (!slideGroup.check(p.name) || !(slideGroup.find(p.name).*isFunctionMap[p.type])())
                    {
                        std::string err_type = err_msgs.find(p.type) == err_msgs.end() ? "[unknow type]" : err_msgs[p.type];
                        yCError(OPENXRHEADSET) << "parameter" << p.name << "not found or not" << err_type << "in" << groupName << "group in configuration file";
                        return false;
                    }
                }

                m_slides.emplace_back();
                SlideLayer& slide = m_slides.back();

                slide.width   = slideGroup.find("width").asFloat64();
                slide.height  = slideGroup.find("height").asFloat64();
                slide.x       = slideGroup.find("x").asFloat64();
                slide.y       = slideGroup.find("y").asFloat64();
                slide.z       = -std::max(0.01, std::abs(slideGroup.find("z").asFloat64())); //make sure that z is negative and that is at least 0.01 in modulus

                std::transform(groupName.begin(), groupName.end(), groupName.begin(), ::tolower);
                std::string portName = m_prefix + "/" + slideGroup.check("port_id", yarp::os::Value(groupName)).toString();

                if (!slide.options.parseFromConfigurationFile(portName, slideGroup))
                {
                    yCError(OPENXRHEADSET) << "Failed to parse" << groupName;
                    return false;
                }
            }
        }
    }

    double period = cfg.check("vr_period", yarp::os::Value(0.011)).asFloat64();
    this->setPeriod(period);


    m_openXrInterfaceSettings.posesPredictionInMs = cfg.check("vr_poses_prediction_in_ms", yarp::os::Value(0.0)).asFloat64();
    m_openXrInterfaceSettings.hideWindow = cfg.check("hide_window") && (cfg.find("hide_window").isNull() || cfg.find("hide_window").asBool());

    m_getStickAsAxis = cfg.check("stick_as_axis", yarp::os::Value(false)).asBool();
    m_rootFrame = cfg.check("tf_root_frame", yarp::os::Value("openxr_origin")).asString();
    m_rootFrameRaw = m_rootFrame + "_raw";

    bool split_eye_ports = cfg.check("split_eye_ports") && (cfg.find("split_eye_ports").isNull() || cfg.find("split_eye_ports").asBool());//split_eye_ports is found and it is either true or it has not value
    bool split_eye_false = cfg.check("split_eye_ports") && !cfg.find("split_eye_ports").isNull() && !cfg.find("split_eye_ports").asBool(); //split_eye_ports is explictly set to false
    bool mono_eye = cfg.check("mono_eye") && (cfg.find("mono_eye").isNull() || cfg.find("mono_eye").asBool());

    if (split_eye_ports && mono_eye)
    {
        yCError(OPENXRHEADSET) << "Both split_eye_ports and mono_eye are defined. Only one of the two can be true at the same time.";
        return false;
    }

    double interCameraDistanceDefault = 0.07;
    if (mono_eye)
    {
        m_eyesManager.options().mode = EyesManager::Mode::MONO;
        interCameraDistanceDefault = 0.0;
    }
    else if (split_eye_false)
    {
        m_eyesManager.options().mode = EyesManager::Mode::STEREO_SINGLE_PORT;
    }
    else
    {
        m_eyesManager.options().mode = EyesManager::Mode::STEREO_DUAL_PORT;
    }

    m_eyesManager.options().interCameraDistance = std::abs(cfg.check("inter_camera_distance", yarp::os::Value(interCameraDistanceDefault)).asFloat64()); //Distance between the cameras of the iCub robot
        m_eyesManager.options().leftAzimuthOffset = cfg.check("left_azimuth_offset", yarp::os::Value(0.0)).asFloat64();
    m_eyesManager.options().leftElevationOffset = cfg.check("left_elevation_offset", yarp::os::Value(0.0)).asFloat64();
    m_eyesManager.options().leftImageRotation = cfg.check("left_image_rotation", yarp::os::Value(0.0)).asFloat64();
    m_eyesManager.options().eyeZPosition = -std::max(0.01, std::abs(cfg.check("eye_z_position", yarp::os::Value(-1.0)).asFloat64())); //make sure that z is negative and that is at least 0.01 in modulus
    m_eyesManager.options().rightAzimuthOffset = cfg.check("right_azimuth_offset", yarp::os::Value(0.0)).asFloat64();
    m_eyesManager.options().rightElevationOffset = cfg.check("right_elevation_offset", yarp::os::Value(0.0)).asFloat64();
    m_eyesManager.options().rightImageRotation = cfg.check("right_image_rotation", yarp::os::Value(0.0)).asFloat64();
    m_eyesManager.options().portPrefix = m_prefix;

    m_rootFrameRawHRootFrame.position.setZero();
    m_rootFrameRawHRootFrame.positionValid = true;
    m_rootFrameRawHRootFrame.rotation.setIdentity();
    m_rootFrameRawHRootFrame.rotationValid = true;


    std::vector<PosesManager::Label> labels;
    yarp::os::Bottle& labelsGroup = cfg.findGroup("POSES_LABELS");
    for (size_t i = 1; i < labelsGroup.size(); ++i) //The first element is the name of the group itself
    {
        yarp::os::Value& labelElement = labelsGroup.get(i);
        if (!labelElement.isList() || labelElement.asList()->size() != 2)
        {
            yCError(OPENXRHEADSET) << "Each entry of the POSES_LABELS group is supposed to contain only two elements. The original name and the modified one. Cause: " << labelElement.toString();
            return false;
        }
        yarp::os::Bottle* labelList = labelElement.asList();
        labels.push_back({labelList->get(0).asString(), labelList->get(1).asString()});
    }

    //opening tf client
    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", cfg.check("tfDevice", yarp::os::Value("frameTransformClient")).asString());
    tfClientCfg.put("filexml_option",  cfg.check("tfFile", yarp::os::Value("ftc_yarp_only.xml")).asString());
    tfClientCfg.put("ft_client_prefix", cfg.check("tfLocal", yarp::os::Value(m_prefix + "/tf")).asString());
    if (cfg.check("tfRemote"))
    {
        tfClientCfg.put("ft_server_prefix", cfg.find("tfRemote").asString());
    }
    tfClientCfg.put("period", period);
    tfClientCfg.put("local_rpc", m_prefix + "/tf/local_rpc");

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
    yCInfo(OPENXRHEADSET) << "Transform client successfully opened.";

    FilteredPosePublisherSettings posePublisherSettings;
    posePublisherSettings.tfPublisher = m_tfPublisher;
    posePublisherSettings.tfBaseFrame = m_rootFrameRaw;
    posePublisherSettings.period = getPeriod();
    posePublisherSettings.checks.maxDistance = cfg.check("pose_check_max_distance", yarp::os::Value(0.1)).asFloat64();
    posePublisherSettings.checks.maxAngularDistanceInRad = cfg.check("pose_check_max_angle_rad", yarp::os::Value(0.5)).asFloat64();
    posePublisherSettings.checks.lastDataExpirationTime = cfg.check("pose_check_expiration_time", yarp::os::Value(5.0)).asFloat64();
    posePublisherSettings.checks.maxConvergenceTime = cfg.check("pose_check_max_convergence_time", yarp::os::Value(3.0)).asFloat64();
    posePublisherSettings.checks.convergenceRatio = cfg.check("pose_check_convergence_ratio", yarp::os::Value(0.05)).asFloat64();

    if (posePublisherSettings.checks.convergenceRatio < 0 || posePublisherSettings.checks.convergenceRatio > 1)
    {
        posePublisherSettings.checks.convergenceRatio = std::min(1.0, std::max(0.0, posePublisherSettings.checks.convergenceRatio));
        yCWarning(OPENXRHEADSET) << "pose_check_convergence_ratio is supposed to be in the range [0, 1]. Clamping to" << posePublisherSettings.checks.convergenceRatio << ".";
    }

    std::vector<CustomPosePublisherSettings> customPoses;
    int custom_poses_count = cfg.find("custom_poses").asInt32();
    for (int i = 0; i < custom_poses_count; ++i)
    {
        customPoses.emplace_back();
        std::string groupName = "CUSTOM_POSE_" + std::to_string(i);
        if (!customPoses.back().parseFromConfigurationFile(cfg.findGroup(groupName)))
        {
            yCError(OPENXRHEADSET) << "Failed to parse" << groupName;
            return false;
        }
    }

    m_posesManager.initialize(m_rootFrame, labels, customPoses, posePublisherSettings);

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
    this->askToStop();
    return true;
}

bool yarp::dev::OpenXrHeadset::threadInit()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_openXrInterface.initialize(m_openXrInterfaceSettings))
        {
            yCError(OPENXRHEADSET) << "Failed to initialize OpenXr interface.";
            return false;
        }

        m_eyesManager.options().leftEyeQuadLayer = m_openXrInterface.addHeadFixedOpenGLQuadLayer();
        m_eyesManager.options().rightEyeQuadLayer = m_openXrInterface.addHeadFixedOpenGLQuadLayer();

        if (!m_eyesManager.initialize())
        {
            yCError(OPENXRHEADSET) << "Failed to initialize eyes.";
        }

        for (GuiParam& gui : m_huds)
        {
            if (!gui.layer.initialize(m_openXrInterface.addHeadFixedOpenGLQuadLayer(), gui.portName)) {
                yCError(OPENXRHEADSET) << "Cannot initialize" << gui.portName << "display texture.";
                return false;
            }
            gui.layer.setVisibility(gui.visibility);
            gui.layer.setDimensions(gui.width, gui.height);
            gui.layer.setPosition({gui.x, gui.y, gui.z});
        }

        for (LabelLayer& label : m_labels)
        {
            label.options.quadLayer = m_openXrInterface.addHeadFixedOpenGLQuadLayer();

            if (!label.layer.initialize(label.options)) {
                yCError(OPENXRHEADSET) << "Cannot initialize" << label.options.portName << "label.";
                return false;
            }
            label.layer.setVisibility(IOpenXrQuadLayer::Visibility::BOTH_EYES);
            label.layer.setDimensions(label.width, label.height);
            label.layer.setPosition({label.x, label.y, label.z});
        }

        for (SlideLayer& slide : m_slides)
        {
            slide.options.quadLayer = m_openXrInterface.addHeadFixedOpenGLQuadLayer();
            if (!slide.layer.initialize(slide.options)) {
                yCError(OPENXRHEADSET) << "Cannot initialize" << slide.options.portName << "slide.";
                return false;
            }
            slide.layer.setVisibility(IOpenXrQuadLayer::Visibility::BOTH_EYES);
            slide.layer.setDimensions(slide.width, slide.height);
            slide.layer.setPosition({slide.x, slide.y, slide.z});
            slide.layer.setImage(slide.options.initialSlide);
        }
    }

    for (size_t i = 0; i < 10 && m_openXrInterface.isRunning(); ++i)
    {
        run(); //dry run. This is to make sure that the number of buttons is correctly retrieved by the JoypadControlServer
        yarp::os::Time::delay(this->getPeriod());
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        this->yarp().attachAsServer(this->m_rpcPort);
        if(!m_rpcPort.open(m_rpcPortName))
        {
            yCError(OPENXRHEADSET) << "Could not open" << m_rpcPortName << " RPC port.";
            return false;
        }
    }

    return true;
}

void yarp::dev::OpenXrHeadset::threadRelease()
{
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

    for (auto& slide : m_slides)
    {
        slide.layer.close();
    }

    m_huds.clear();
    m_labels.clear();
    m_slides.clear();
    m_eyesManager.close();

    m_openXrInterface.close();

    if (m_tfPublisher)
    {
        m_driver.close();
        m_tfPublisher = nullptr;
    }

    m_rpcPort.close();

    m_closed = true;
}

void yarp::dev::OpenXrHeadset::run()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_openXrInterface.isRunning())
    {
        if (!m_eyesManager.update()) {
            yCError(OPENXRHEADSET) << "Failed to update eyes.";
        }

        Eigen::Vector3f leftAngles = EulerAngles::XYZ(m_eyesManager.getLeftEyeDesiredRotation().matrix());
        Eigen::Vector3f rightAngles = EulerAngles::XYZ(m_eyesManager.getRightEyeDesiredRotation().matrix());

        double averageElevation = (leftAngles[0] + rightAngles[0]) / 2.0;
        double averageAzimuth = (leftAngles[1] + rightAngles[1]) / 2.0;

        Eigen::Quaternionf averageRotation = Eigen::AngleAxisf(averageElevation, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(averageAzimuth, Eigen::Vector3f::UnitY());

        for (GuiParam& gui : m_huds)
        {
            if (gui.followEyes)
            {
                Eigen::Quaternionf newRotation;
                if (gui.visibility == IOpenXrQuadLayer::Visibility::LEFT_EYE)
                {
                    newRotation = m_eyesManager.getLeftEyeDesiredRotation();
                }
                else if (gui.visibility == IOpenXrQuadLayer::Visibility::RIGHT_EYE)
                {
                    newRotation = m_eyesManager.getRightEyeDesiredRotation();
                }
                else
                {
                    newRotation = averageRotation;
                }

                Eigen::Vector3f guiPosition = { gui.x, gui.y, gui.z };
                gui.layer.setPose(newRotation * guiPosition, newRotation);
            }

            if (!gui.layer.updateTexture()) {
                yCError(OPENXRHEADSET) << "Failed to update" << gui.portName << "display texture.";
            }
        }

        for (LabelLayer& label : m_labels)
        {
            if (label.options.followEyes)
            {
                Eigen::Vector3f labelPosition = { label.x, label.y, label.z };
                label.layer.setPosition(averageRotation * labelPosition);
            }

            if (!label.layer.updateTexture()) {
                yCError(OPENXRHEADSET) << "Failed to update" << label.options.portName << "label.";
            }
        }
        for (SlideLayer& slide : m_slides)
        {
            if (!slide.layer.updateTexture()) {
                yCError(OPENXRHEADSET) << "Failed to update" << slide.options.portName << "slide.";
            }
        }
        m_openXrInterface.draw();

        m_openXrInterface.getButtons(m_buttons);
        m_openXrInterface.getAxes(m_axes);
        m_openXrInterface.getThumbsticks(m_thumbsticks);
        m_openXrInterface.getAllPoses(m_posesManager.inputs());

        if (m_openXrInterface.shouldResetLocalReferenceSpace())
        {
            //The local reference space has been changed by the user.
            m_rootFrameRawHRootFrame.position.setZero();
            m_rootFrameRawHRootFrame.rotation.setIdentity();
        }

        //Publish the transformation from the root frame to the OpenXR root frame
        m_posesManager.setTransformFromRawToRootFrame(m_rootFrameRawHRootFrame);

        m_posesManager.publishFrames();
    }
    else
    {
        close();
        return;
    }
}

bool yarp::dev::OpenXrHeadset::startService()
{
    //To let the device driver knowing that it need to poll updateService continuosly
    return false;
}

bool yarp::dev::OpenXrHeadset::updateService()
{
    //To let the device driver that we are still alive
    return !m_closed;
}

bool yarp::dev::OpenXrHeadset::stopService()
{
    return this->close();
}

bool yarp::dev::OpenXrHeadset::getAxisCount(unsigned int &axis_count)
{
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
    std::lock_guard<std::mutex> lock(m_mutex);

    button_count = m_buttons.size();

    return true;
}

bool yarp::dev::OpenXrHeadset::getTrackballCount(unsigned int &trackball_count)
{
    trackball_count = 0;

    return true;
}

bool yarp::dev::OpenXrHeadset::getHatCount(unsigned int &hat_count)
{
    hat_count = 0; //These are handled as buttons in OpenXR

    return true;
}

bool yarp::dev::OpenXrHeadset::getTouchSurfaceCount(unsigned int &touch_count)
{
    touch_count = 0;

    return true;
}

bool yarp::dev::OpenXrHeadset::getStickCount(unsigned int &stick_count)
{
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
    value.zero();
    yCError(OPENXRHEADSET) << "No trackball are considered in this device.";
    return false;
}

bool yarp::dev::OpenXrHeadset::getHat(unsigned int /*hat_id*/, unsigned char &value)
{
    value = 0;
    yCError(OPENXRHEADSET) << "No hats are considered in this device.";
    return false;
}

bool yarp::dev::OpenXrHeadset::getAxis(unsigned int axis_id, double &value)
{
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
    value.clear();
    yCError(OPENXRHEADSET) << "No touch devices are considered in this device.";
    return false;
}

std::string yarp::dev::OpenXrHeadset::getLeftHandInteractionProfile()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_openXrInterface.currentLeftHandInteractionProfile();
}

std::string yarp::dev::OpenXrHeadset::getRightHandInteractionProfile()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_openXrInterface.currentRightHandInteractionProfile();
}

std::vector<double> yarp::dev::OpenXrHeadset::getLeftImageDimensions()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.getLeftImageDimensions();
}

std::vector<double> yarp::dev::OpenXrHeadset::getRightImageDimensions()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.getRightImageDimensions();
}

std::vector<double> yarp::dev::OpenXrHeadset::getLeftImageAnglesOffsets()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.getLeftImageAnglesOffsets();
}

std::vector<double> yarp::dev::OpenXrHeadset::getRightImageAnglesOffsets()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.getRightImageAnglesOffsets();
}

double yarp::dev::OpenXrHeadset::getLeftImageRotation()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_eyesManager.getLeftImageRotation();
}

double yarp::dev::OpenXrHeadset::getRightImageRotation()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_eyesManager.getRightImageRotation();
}

bool yarp::dev::OpenXrHeadset::setLeftImageAnglesOffsets(const double azimuth, const double elevation)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.setLeftImageAnglesOffsets(azimuth, elevation);
}

bool yarp::dev::OpenXrHeadset::setRightImageAnglesOffsets(const double azimuth, const double elevation)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.setRightImageAnglesOffsets(azimuth, elevation);
}

bool yarp::dev::OpenXrHeadset::setLeftImageRotation(const double angle)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_eyesManager.setLeftImageRotation(angle);
}

bool yarp::dev::OpenXrHeadset::setRightImageRotation(const double angle)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_eyesManager.setRightImageRotation(angle);
}

bool yarp::dev::OpenXrHeadset::isLeftEyeActive()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.isLeftEyeActive();
}

bool yarp::dev::OpenXrHeadset::isRightEyeActive()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.isRightEyeActive();
}

double yarp::dev::OpenXrHeadset::getEyesZPosition()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.getEyesZPosition();
}

bool yarp::dev::OpenXrHeadset::setEyesZPosition(const double eyesZPosition)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.setEyesZPosition(eyesZPosition);
}

double yarp::dev::OpenXrHeadset::getInterCameraDistance()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.getInterCameraDistance();
}

bool yarp::dev::OpenXrHeadset::setInterCameraDistance(const double distance)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_eyesManager.setInterCameraDistance(distance);
}

std::string yarp::dev::OpenXrHeadset::getLeftImageControlPortName()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_eyesManager.getLeftImageControlPortName();
}

std::string yarp::dev::OpenXrHeadset::getRightImageControlPortName()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_eyesManager.getRightImageControlPortName();
}

bool yarp::dev::OpenXrHeadset::setGUIEnabled(const int32_t GUIIndex, const bool enabled)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (GUIIndex >= m_huds.size())
    {
        return false;
    }

    m_huds[GUIIndex].layer.setEnabled(enabled);

    return true;
}

bool yarp::dev::OpenXrHeadset::setLabelEnabled(const int32_t labelIndex, const bool enabled)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (labelIndex >= m_labels.size())
    {
        return false;
    }

    m_labels[labelIndex].layer.setEnabled(enabled);

    return true;
}

bool yarp::dev::OpenXrHeadset::alignRootFrameToHeadset()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    OpenXrInterface::Pose headPose = m_openXrInterface.headPose();

    if (!headPose.positionValid || !headPose.rotationValid)
    {
        yCError(OPENXRHEADSET) << "Cannot align the root frame to the headset. The headset pose is not valid";
        return false;
    }

    const Eigen::Matrix3f &root_R_headset = headPose.rotation.matrix();
    // This code was taken from https://www.geometrictools.com/Documentation/EulerAngles.pdf
    // Section 2.2. It computes the XZY inverse kinematics, and we consider only the rotation around Y
    double gravityAngle = 0.0;
    if ((root_R_headset(0,1) < +1.0) && (root_R_headset(0, 1) > -1.0))
    {
        gravityAngle = std::atan2(root_R_headset(0, 2), root_R_headset(0, 0));
    }


    m_rootFrameRawHRootFrame.rotation = Eigen::AngleAxisf(gravityAngle, Eigen::Vector3f::UnitY());
    m_rootFrameRawHRootFrame.position = headPose.position;

    return true;
}

bool yarp::dev::OpenXrHeadset::setCustomPoseRelativePosition(const std::string &customFrameName, const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_posesManager.setCustomPoseRelativePosition(customFrameName, {static_cast<float>(x),
                                                                           static_cast<float>(y),
                                                                           static_cast<float>(z)});
}

bool yarp::dev::OpenXrHeadset::setCustomPoseRelativeOrientation(const std::string &customFrameName, const double angle1, const double angle2, const double angle3)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_posesManager.setCustomPoseRelativeOrientation(customFrameName, {static_cast<float>(angle1),
                                                                              static_cast<float>(angle2),
                                                                              static_cast<float>(angle3)});
}
