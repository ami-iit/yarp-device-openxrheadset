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

#include <iostream>
#include <cstdlib>
#include <memory>
#include <csignal>

std::atomic<bool> isClosing{false};

void my_handler(int)
{
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


struct FrameViewier
{
    std::shared_ptr<FrameViewier> parent{nullptr};
    std::string name;
    size_t vizIndex;
    iDynTree::Transform transform;
};

int main(int /*argc*/, char** /*argv*/)
{
    std::string namePrefix = "/OpenXrFrameViewer";
    // Visualize the model
    iDynTree::Visualizer visualizer;

    bool ok = visualizer.init();

    visualizer.camera().animator()->enableMouseControl();

    if( !ok )
    {
        std::cerr << "Failed to initialize the visualizer." << std::endl;
        return EXIT_FAILURE;
    }

    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", "transformClient");
    tfClientCfg.put("local",  namePrefix +"/tf");
    tfClientCfg.put("remote", "/transformServer");

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

    iDynTree::Rotation openXrInertialRotation;
    openXrInertialRotation.zero();
    openXrInertialRotation(0,2) = -1.0; //-z is forward
    openXrInertialRotation(1,0) = -1.0; //-x is left
    openXrInertialRotation(2,1) =  1.0; // +y is up
    iDynTree::Transform openXrInertial;
    openXrInertial.setPosition(iDynTree::Position::Zero());
    openXrInertial.setRotation(openXrInertialRotation);

    std::vector<std::shared_ptr<FrameViewier>> frames;

    std::shared_ptr<FrameViewier> rootFrame = std::make_shared<FrameViewier>();
    rootFrame->name = "openxr_origin";
    rootFrame->transform = openXrInertial;
    rootFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity());
    visualizer.frames().getFrameLabel(rootFrame->vizIndex)->setText(rootFrame->name);
    frames.push_back(rootFrame);

    std::shared_ptr<FrameViewier> headFrame = std::make_shared<FrameViewier>();
    headFrame->name = "openxr_head";
    headFrame->parent = rootFrame;
    headFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity());
    visualizer.frames().getFrameLabel(headFrame->vizIndex)->setText(headFrame->name);
    frames.push_back(headFrame);

    std::shared_ptr<FrameViewier> leftHandFrame = std::make_shared<FrameViewier>();
    leftHandFrame->name = "openxr_left_hand";
    leftHandFrame->parent = rootFrame;
    leftHandFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity());
    visualizer.frames().getFrameLabel(leftHandFrame->vizIndex)->setText(leftHandFrame->name);
    frames.push_back(leftHandFrame);

    std::shared_ptr<FrameViewier> rightHandFrame = std::make_shared<FrameViewier>();
    rightHandFrame->name = "openxr_right_hand";
    rightHandFrame->parent = rootFrame;
    rightHandFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity());
    visualizer.frames().getFrameLabel(rightHandFrame->vizIndex)->setText(rightHandFrame->name);
    frames.push_back(rightHandFrame);

    iDynTree::Transform transformBuffer;
    yarp::sig::Matrix matrixBuffer;
    matrixBuffer.resize(4, 4);

    // Visualization loop
    while( visualizer.run() && !isClosing )
    {
        for (auto& frame: frames)
        {
            if (frame->parent)
            {
                if (tfReader->getTransform(frame->name, frame->parent->name, matrixBuffer))
                {
                    iDynTree::toiDynTree(matrixBuffer, transformBuffer);
                    frame->transform = frame->parent->transform * transformBuffer;
                    visualizer.frames().updateFrame(frame->vizIndex, frame->transform);
                }
            }
        }

        visualizer.draw();
    }

    return EXIT_SUCCESS;
}
