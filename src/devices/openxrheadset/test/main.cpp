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
#include <SlideManager.h>
#include <Resources.h>

int main()
{
    yarp::os::Network yarp;
    OpenXrInterface openXrInterface;
    std::array<ImagePortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgb>>, 2> displayPorts;
    LabelPortToQuadLayer label;
    SlideManager slide;


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
    labelOptions.labelSuffix = " 1 with a veeeeeery loooooong test I might eventually go to a new line";
    labelOptions.pixelSize = 32;
    labelOptions.backgroundColor<< 1.0, 1.0, 1.0, 1.0; //Full white
    labelOptions.labelColor << 1.0, 0.0, 0.0, 1.0; //Opaque red
    labelOptions.verticalAlignement = LabelPortToQuadLayer::Options::VerticalAlignement::Center;
    labelOptions.horizontalAlignement = LabelPortToQuadLayer::Options::HorizontalAlignement::Left;

    label.initialize(labelOptions);
    label.setVisibility(IOpenXrQuadLayer::Visibility::LEFT_EYE);
    label.setDimensions(0.1, 0.02);
    label.setPosition({-0.05, 0, -0.1});

    SlideManager::Options slideOptions;
    slideOptions.quadLayer = openXrInterface.addHeadFixedQuadLayer();
    slideOptions.portName = "/openxrtest/slide:i";
    slideOptions.slidesPath = resourcesPath() + "/testImages";

    slide.initialize(slideOptions);
    slide.setVisibility(IOpenXrQuadLayer::Visibility::LEFT_EYE);
    slide.setDimensions(0.1, 0.02);
    slide.setPosition({-0.05, 0, -0.1});

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

            if (!slide.updateTexture())
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

    slide.close();

    openXrInterface.close();

    return EXIT_SUCCESS;
}
