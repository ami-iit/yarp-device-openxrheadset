/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */


#include <impl/OpenXrInterfaceImpl.h>


XrBool32 OpenXrInterface::Implementation::OpenXrDebugCallback(XrDebugUtilsMessageSeverityFlagsEXT severity, XrDebugUtilsMessageTypeFlagsEXT, const XrDebugUtilsMessengerCallbackDataEXT *data, void *) {
    yCTrace(OPENXRHEADSET);

    if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
    {
        yCError(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
    }
    else if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
    {
        yCWarning(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
    }
    else if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT)
    {
        yCInfo(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
    }
    else if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT)
    {
        yCDebug(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
    }

    return XR_TRUE;
}

void OpenXrInterface::Implementation::glfwErrorCallback(int error, const char *description)
{
    yCError(OPENXRHEADSET) << "GLFW Error:" << error << description;
}

void OpenXrInterface::Implementation::GLMessageCallback(GLenum, GLenum type, GLuint, GLenum severity, GLsizei, const GLchar *message, const void *) {
    yCError(OPENXRHEADSET, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s",
            ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
            type, severity, message );
}

void OpenXrInterface::Implementation::submitLayer(const XrCompositionLayerBaseHeader *layer)
{
    layer_count++;
    if (layer_count > submitted_layers.size())
    {
        submitted_layers.resize(layer_count);
    }
    submitted_layers[layer_count-1] = layer;
}

OpenXrInterface::Pose OpenXrInterface::Implementation::getPose(const XrSpaceLocation &spaceLocation)
{
    Pose output;
    output.positionValid = spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
    output.position = toEigen(spaceLocation.pose.position);

    output.rotationValid = spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    output.rotation = toEigen(spaceLocation.pose.orientation);

    return output;
}

bool OpenXrInterface::Implementation::suggestInteractionProfileBindings(const std::string &interactionProfile, const std::vector<XrActionSuggestedBinding> &suggestedBindings)
{
    XrPath interaction_profile_path;
    XrResult result = xrStringToPath(instance, interactionProfile.c_str(),
                                     &interaction_profile_path);
    if (!checkXrOutput(result, "Failed to get interaction profile %s.", interactionProfile.c_str()))
        return false;

    const XrInteractionProfileSuggestedBinding suggested_bindings = {
        .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
        .next = NULL,
        .interactionProfile = interaction_profile_path,
        .countSuggestedBindings = static_cast<uint32_t>(suggestedBindings.size()),
        .suggestedBindings = suggestedBindings.data()};

    result = xrSuggestInteractionProfileBindings(instance, &suggested_bindings);
    if (!checkXrOutput(result, "Failed to suggest bindings for %s.", interactionProfile.c_str()))
        return false;

    return true;
}
