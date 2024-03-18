/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/LogStream.h>
#include <impl/OpenXrQuadLayer.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrEigenConversions.h>

OpenXrQuadLayer::OpenXrQuadLayer()
{

}

void OpenXrQuadLayer::setPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &quaternion)
{
    setPosition(position);
    setQuaternion(quaternion);
}

void OpenXrQuadLayer::setPosition(const Eigen::Vector3f &position)
{
    desiredHeadFixedPose.position = toXr(position);
}

void OpenXrQuadLayer::setQuaternion(const Eigen::Quaternionf &quaternion)
{
    desiredHeadFixedPose.orientation = toXr(quaternion);
}

void OpenXrQuadLayer::setDimensions(float widthInMeters, float heightInMeters)
{
    layer.size.height = heightInMeters;
    layer.size.width = widthInMeters;
}

void OpenXrQuadLayer::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    switch (visibility)
    {
    case IOpenXrQuadLayer::Visibility::LEFT_EYE:
        layer.eyeVisibility = XR_EYE_VISIBILITY_LEFT;
        hasVisibility = true;
        break;

    case IOpenXrQuadLayer::Visibility::RIGHT_EYE:
        layer.eyeVisibility = XR_EYE_VISIBILITY_RIGHT;
        hasVisibility = true;
        break;

    case IOpenXrQuadLayer::Visibility::BOTH_EYES:
        layer.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        hasVisibility = true;
        break;

    case IOpenXrQuadLayer::Visibility::NONE:
        hasVisibility = false;
        break;
    }
}

void OpenXrQuadLayer::useAlphaChannel(bool useAlphaChannel)
{
    layer.layerFlags = 0;

    if (useAlphaChannel)
    {
        layer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT | XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
    }
}

bool OpenXrQuadLayer::getImage(uint32_t &glImage)
{
    // Acquire swapchain images
    XrSwapchainImageAcquireInfo acquire_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
                                                .next = NULL};
    uint32_t acquired_index;
    XrResult result = xrAcquireSwapchainImage(layer.subImage.swapchain, &acquire_info, &acquired_index);
    if (!XR_SUCCEEDED(result))
    {
        yCError(OPENXRHEADSET) << "Failed to acquire swapchain image.";
        if (acquired)
        {
            yCError(OPENXRHEADSET) << "You already acquired some images. Remember to call submitImage after you are done writing on the image.";
        }
        return false;
    }

    XrSwapchainImageWaitInfo wait_info = {
        .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, .next = NULL, .timeout = 1000};
    result = xrWaitSwapchainImage(layer.subImage.swapchain, &wait_info);
    if (!XR_SUCCEEDED(result))
    {
        yCError(OPENXRHEADSET) << "Failed to wait for swapchain image!";
        return false;
    }

    acquired++;
    glImage = swapchain_images[acquired_index].image;

    return true;
}

bool OpenXrQuadLayer::submitImage()
{
    return submitImage(0, 0, imageMaxWidth(), imageMaxHeight());
}

bool OpenXrQuadLayer::submitImage(int32_t xOffset, int32_t yOffset, int32_t imageWidth, int32_t imageHeight)
{
    if (acquired == 0)
    {
        yCError(OPENXRHEADSET) << "First you have to get an image.";
        return false;
    }

    if (xOffset < 0 || yOffset < 0 || imageWidth < 0 || imageHeight < 0)
    {
        yCError(OPENXRHEADSET) << "One or more input is lower than zero.";
        return false;
    }

    if (xOffset + imageWidth > imageMaxWidth())
    {
        yCError(OPENXRHEADSET) << "The image width plus the x offset is supposed to be less than imageMaxWidth().";
        return false;
    }

    if (yOffset + imageHeight > imageMaxHeight())
    {
        yCError(OPENXRHEADSET) << "The image height plus the y offset is supposed to be less than imageMaxHeight().";
        return false;
    }
    // Release the oldest acquired swapchain image
    XrSwapchainImageReleaseInfo release_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
                                                .next = NULL};
    XrResult result = xrReleaseSwapchainImage(layer.subImage.swapchain, &release_info);

    if (!XR_SUCCEEDED(result))
    {
        yCError(OPENXRHEADSET) << "Failed to release swapchain image!";
        return false;
    }

    acquired--;
    layer.subImage.imageRect.offset.x = xOffset;
    layer.subImage.imageRect.offset.y = yOffset;

    layer.subImage.imageRect.extent.height = imageHeight;
    layer.subImage.imageRect.extent.width = imageWidth;
    released = true;

    return true;

}

int32_t OpenXrQuadLayer::imageMaxHeight() const
{
    return swapchainHeight;
}

int32_t OpenXrQuadLayer::imageMaxWidth() const
{
    return swapchainWidth;
}

float OpenXrQuadLayer::layerWidth() const
{
    return layer.size.width;
}

float OpenXrQuadLayer::layerHeight() const
{
    return layer.size.height;
}

Eigen::Vector3f OpenXrQuadLayer::layerPosition() const
{
    return toEigen(desiredHeadFixedPose.position);
}

Eigen::Quaternionf OpenXrQuadLayer::layerQuaternion() const
{
    return toEigen(desiredHeadFixedPose.orientation);
}

void OpenXrQuadLayer::setEnabled(bool enabled)
{
    this->isEnabled = enabled;
}

bool OpenXrQuadLayer::shouldSubmit() const
{
    return hasVisibility && released && isEnabled;
}

OpenXrQuadLayer::~OpenXrQuadLayer()
{

}
