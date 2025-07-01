/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <iDynTree/Visualizer.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/conf/version.h>

#include <iostream>
#include <cstdlib>
#include <memory>
#include <csignal>
#include <unordered_map>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

std::atomic<bool> isClosing{false};

void my_handler(int)
{
    yInfo() << "Closing";
    isClosing = true;
}

#ifdef WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType) {
        // Handle the CTRL-C signal.
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        my_handler(0);
        return TRUE;

    // Handle all other events
    default:
        return FALSE;
    }
}
#endif

void handleSigInt()
{
#ifdef WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &my_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#endif
}


struct FrameViewer
{
    std::shared_ptr<FrameViewer> parent{nullptr};
    std::string name;
    size_t vizIndex;
    iDynTree::Transform transform;
};

int main(int argc, char** argv)
{
    handleSigInt();
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.configure(argc, argv);

    iDynTree::Visualizer visualizer;

    bool ok = visualizer.init();

    visualizer.camera().animator()->enableMouseControl();
    visualizer.environment().setElementVisibility("world_frame", false);

    if( !ok )
    {
        std::cerr << "Failed to initialize the visualizer." << std::endl;
        return EXIT_FAILURE;
    }

    std::string name = rf.check("name", yarp::os::Value("OpenXrFrameViewer")).asString();

    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", rf.check("tfDevice", yarp::os::Value("frameTransformClient")).asString());
    tfClientCfg.put("filexml_option",  rf.check("tfFile", yarp::os::Value("ftc_yarp_only.xml")).asString());
    tfClientCfg.put("ft_client_prefix", "/" + name + "/tf");
    if (rf.check("tfRemote"))
    {
        tfClientCfg.put("ft_server_prefix", rf.find("tfRemote").asString());
    }
    tfClientCfg.put("local_rpc", "/" + name + "/tf/local_rpc");

    yarp::dev::PolyDriver driver;
    yarp::dev::IFrameTransform* tfReader;

    if (!driver.open(tfClientCfg))
    {
        yError() << "Unable to open polydriver with the following options:" << tfClientCfg.toString();
        return EXIT_FAILURE;
    }

    if (!driver.view(tfReader) || tfReader == nullptr)
    {
        yError() << "Unable to view IFrameTransform interface.";
        return EXIT_FAILURE;
    }

    bool z_up = rf.check("z_up") && (rf.find("z_up").isNull() || rf.find("z_up").asBool());

    iDynTree::Rotation openXrInertialRotation;
    openXrInertialRotation.zero();

    if (z_up) //Standard convention with the z up
    {
        openXrInertialRotation(0, 0) = 1.0; //x is forward
        openXrInertialRotation(1, 1) = 1.0; //y is left
        openXrInertialRotation(2, 2) = 1.0; //z is up
    }
    else //OpenXR convention, where y is up and -z is forward
    {
        openXrInertialRotation(0, 2) = -1.0; //-z is forward
        openXrInertialRotation(1, 0) = -1.0; //-x is left
        openXrInertialRotation(2, 1) =  1.0; // +y is up
    }
    iDynTree::Transform openXrInertial;
    openXrInertial.setPosition(iDynTree::Position::Zero());
    openXrInertial.setRotation(openXrInertialRotation);

    double frame_length = rf.check("frame_length", yarp::os::Value(0.5)).asFloat64();
    double label_height = rf.check("label_height", yarp::os::Value(0.1)).asFloat64();
    double position_scale = rf.check("position_scale", yarp::os::Value(1.0)).asFloat64();

    std::unordered_map<std::string, std::shared_ptr<FrameViewer>> frames;

    std::shared_ptr<FrameViewer> rootFrame = std::make_shared<FrameViewer>();
    rootFrame->name = rf.check("tf_root_frame", yarp::os::Value("openxr_origin")).asString();
    rootFrame->transform = openXrInertial;
    rootFrame->vizIndex = visualizer.frames().addFrame(openXrInertial, frame_length);
    iDynTree::ILabel* label = visualizer.frames().getFrameLabel(rootFrame->vizIndex);
    label->setSize(label_height);
    label->setText(rootFrame->name);
    frames[rootFrame->name] = rootFrame;

    iDynTree::Transform transformBuffer;
    yarp::sig::Matrix matrixBuffer;
    matrixBuffer.resize(4, 4);
    double lastIDsUpdate = yarp::os::Time::now() - 2.0;

    // Visualization loop
    while( visualizer.run() && !isClosing )
    {
        if (yarp::os::Time::now() - lastIDsUpdate > 1.0)
        {
            std::vector<std::string> ids; //When getting the ids, it seems that the input vector is not cleared. Passing a clean one every time.
            if(tfReader->getAllFrameIds(ids))
            {
                for(std::string& id : ids)
                {
                    if (frames.find(id) == frames.end())
                    {
                        bool canTransform = false;
#if YARP_VERSION_COMPARE(<, 3, 11, 0)
                        canTransform = tfReader->canTransform(id, rootFrame->name);
#else
                        bool canTranformOk = false;
                        bool canTransformRetValue = tfReader->canTransform(id, rootFrame->name, canTranformOk);
                        canTransform = canTranformOk && canTransformRetValue;
#endif
                        if (canTransform) //The frame has never been added and it is linked to the openxr_origin
                        {
                            std::shared_ptr<FrameViewer> newFrame = std::make_shared<FrameViewer>();
                            newFrame->name = id;
                            newFrame->parent = rootFrame;
                            newFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity(), frame_length);
                            iDynTree::ILabel* label = visualizer.frames().getFrameLabel(newFrame->vizIndex);
                            label->setSize(label_height);
                            label->setText(newFrame->name);
                            frames[id] = newFrame;
                        }
                    }
                }
            }
            lastIDsUpdate = yarp::os::Time::now();
        }

        for (auto& frameIt: frames)
        {
            auto& frame = frameIt.second;
            if (frame->parent)
            {
                if (tfReader->getTransform(frame->name, frame->parent->name, matrixBuffer))
                {
                    iDynTree::toiDynTree(matrixBuffer, transformBuffer);
                    frame->transform = frame->parent->transform * transformBuffer;
                    iDynTree::Transform transformScaled = frame->transform;
                    const iDynTree::Position& position = frame->transform.getPosition();
                    iDynTree::Position positionScaled(position[0] * position_scale,
                                                      position[1] * position_scale,
                                                      position[2] * position_scale);
                    transformScaled.setPosition(positionScaled);
                    visualizer.frames().updateFrame(frame->vizIndex, transformScaled);
                }
            }
        }

        visualizer.draw();
        std::this_thread::sleep_for(1ms);
    }

    return EXIT_SUCCESS;
}
