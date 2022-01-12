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
#include <ImagePortToQuadLayer.h>
#include <LabelPortToQuadLayer.h>

int main()
{
    yarp::os::Network yarp;
    OpenXrInterface openXrInterface;
    std::array<ImagePortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgb>>, 2> displayPorts;
    LabelPortToQuadLayer label;

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

    LabelPortToQuadLayer::Options labelOptions;
    labelOptions.quadLayer = openXrInterface.addHeadFixedQuadLayer();
    labelOptions.portName = "/openxrtest/label:i";
    labelOptions.labelPrefix = "testLabel";
    labelOptions.labelSuffix = " 1";
    labelOptions.backgroundColor.setZero();
    labelOptions.labelColor << 1.0, 0.0, 0.0, 1.0; //Opaque red

    label.initialize(labelOptions);

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

            if (!label.updateTexture())
            {
                yError() << "Failed to update label";
                return EXIT_FAILURE;
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

    for (int eye = 0; eye < 2; ++eye) {
        displayPorts[eye].close();
    }

    label.close();

    openXrInterface.close();

    return EXIT_SUCCESS;
}
