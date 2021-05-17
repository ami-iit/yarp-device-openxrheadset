/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRINTERFACE_H
#define YARP_DEV_OPENXRINTERFACE_H

#include <memory>
#include <functional>

class OpenXrInterface
{
    class Implementation;

    std::unique_ptr<Implementation> m_pimpl;

    bool checkExtensions();

    bool prepareXrInstance();

    void printInstanceProperties();

    bool prepareXrSystem();

    void printSystemProperties();

    bool prepareWindow();

    bool prepareXrSession();

    bool prepareXrSwapchain();

    bool prepareXrCompositionLayers();

    bool prepareXrActions();

    bool prepareGlFramebuffer();

    void pollXrEvents();

    bool startXrFrame();

    void updateXrViews();

    void updateXrActions();

    void render();

    void endXrFrame();

public:

    OpenXrInterface();

    ~OpenXrInterface();

    OpenXrInterface(const OpenXrInterface& other) = delete;

    OpenXrInterface(OpenXrInterface&& other) = delete;

    OpenXrInterface& operator=(const OpenXrInterface& other) = delete;

    OpenXrInterface& operator=(OpenXrInterface&& other) = delete;

    void setKeyCallback(std::function<void(int /*key*/, int /*scancode*/, int /*action*/, int /*mods*/)> keyCallback);

    bool initialize();

    bool isInitialized() const;

    void draw();

    bool isRunning() const;

    void close();
};

#endif // YARP_DEV_OPENXRINTERFACE_H
