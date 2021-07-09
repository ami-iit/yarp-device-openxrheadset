/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <OpenXrInterface.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <array>
#include <PortToQuadLayer.h>

int main()
{
    yarp::os::Network yarp;
    OpenXrInterface openXrInterface;
    std::array<PortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgb>>, 2> displayPorts;


    if (!openXrInterface.initialize())
    {
        yError() << "Failed to initialize OpenXr interface.";
        return EXIT_FAILURE;
    }

    for (size_t eye = 0; eye < 2; ++eye) {
        if (!displayPorts[eye].initialize(openXrInterface.addHeadFixedQuadLayer(),
                                          (eye == 0 ? "/openxrtest/display/left:i" : "/openxrtest/display/right:i"))) {
            yError() << "Cannot initialize" << (eye == 0 ? "left" : "right") << "display texture.";
            return EXIT_FAILURE;
        }
        displayPorts[eye].setVisibility(eye == 0 ? IOpenXrQuadLayer::Visibility::LEFT_EYE : IOpenXrQuadLayer::Visibility::RIGHT_EYE);
    }

    for (size_t i = 0; i < 10; ++i)
    {
        if (openXrInterface.isRunning())
        {
            for (int eye = 0; eye < 2; ++eye) {
                if (!displayPorts[eye].updateTexture()) {
                    yError() << "Failed to update" << (eye == 0 ? "left" : "right") << "display texture.";
                    return EXIT_FAILURE;
                }
            }

            openXrInterface.draw();
        }
        else
        {
            yError() << "Premature exit.";
            break;
        }
        yInfo() << "Iteration:" << i;
    }

    openXrInterface.close();

    return EXIT_SUCCESS;
}
