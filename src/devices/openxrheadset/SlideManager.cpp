/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */


#include <SlideManager.h>
#include <stb_image.h>
#include <filesystem>
#include <vector>
#include <unordered_set>
#include <algorithm>


bool SlideManager::initialize(const Options &options)
{

    if (!options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The input quadLayer is not valid.";
        return false;
    }

    if (!std::filesystem::exists(options.slidesPath))
    {
        yCError(OPENXRHEADSET) << "Failed to find the slides path:" << options.slidesPath;
        return false;
    }

    std::unordered_map<std::string, std::string> imagesPath;
    std::unordered_set<std::string> supportedExtensions = {".jpeg", ".jpg", ".png", ".bmp", ".tga",
                                                           ".psd", ".gif", ".hdr", ".pic", ".pnm"};

    for (const auto & entry : std::filesystem::directory_iterator(options.slidesPath))
    {
        if (entry.is_regular_file())
        {
            std::string extension = entry.path().extension().generic_string();
            if (supportedExtensions.find(extension) != supportedExtensions.end())
            {
                imagesPath[entry.path().stem().generic_string()] = entry.path().generic_string(); //Map: filename (wihtout extension) -> path
            }
            else
            {
                yCInfo(OPENXRHEADSET) << "Skipping" << entry.path().generic_string() << "since it is not a supported file.";
            }
        }
        else
        {
            yCInfo(OPENXRHEADSET) << "Skipping" << entry.path().generic_string() << "since it is not a file.";
        }
    }

    for (auto& image : imagesPath)
    {
        int width, height, components;
        unsigned char * rawImage = stbi_load(image.second.c_str(), &width, &height, &components, 4); //We force to have 4 components, i.e. RGBA
        if (rawImage)
        {
            m_stbImages.push_back(rawImage);
            m_loadedImages[image.first].setExternal(rawImage, width, height);
            yCInfo(OPENXRHEADSET) << "Loaded image:" << image.second;
        }
        else
        {
            yCWarning(OPENXRHEADSET) << "Failed to load" << image.second;
        }
    }

    m_portPtr = std::make_shared<yarp::os::BufferedPort<yarp::os::Bottle>>();

    if (!m_portPtr->open(options.portName))
    {
        yCError(OPENXRHEADSET) << "Failed to open the port named" << options.portName << ".";
        return false;
    }

    m_portPtr->setReadOnly();

    if (!m_layer.initialize(options.quadLayer))
    {
        yCError(OPENXRHEADSET) << "Failed to initialize quad layer.";
        return false;
    }

    m_options = options;

    return true;
}

void SlideManager::close()
{
    m_portPtr->close();
    m_layer.close();
    for (size_t i = 0; i < m_stbImages.size(); ++i)
    {
        stbi_image_free(m_stbImages[i]);
    }
    m_loadedImages.clear();
}

bool SlideManager::updateTexture()
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return false;
    }

    yarp::os::Bottle* bottle = m_portPtr->read(false);

    std::string inputString = "";
    bool received = bottle && bottle->size() > 0;
    if (received)
    {
        inputString = bottle->get(0).toString();
    }
    else
    {
        return true;
    }

    return setImage(inputString);
}

void SlideManager::setPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &rotation)
{
    m_layer.setPose(position, rotation);
}

void SlideManager::setPosition(const Eigen::Vector3f &position)
{
    m_layer.setPosition(position);
}

Eigen::Vector3f SlideManager::layerPosition() const
{
    return m_layer.layerPosition();
}

void SlideManager::setQuaternion(const Eigen::Quaternionf &quaternion)
{
    m_layer.setQuaternion(quaternion);
}

Eigen::Quaternionf SlideManager::layerQuaternion() const
{
    return m_layer.layerQuaternion();
}

void SlideManager::setDimensions(float widthInMeters, float heightInMeters)
{
    m_layer.setDimensions(widthInMeters, heightInMeters);
}

void SlideManager::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    m_layer.setVisibility(visibility);
}

float SlideManager::layerWidth() const
{
    return m_layer.layerWidth();
}

float SlideManager::layerHeight() const
{
    return m_layer.layerHeight();
}

void SlideManager::setEnabled(bool enabled)
{
    m_layer.setEnabled(enabled);
}

bool SlideManager::setImage(const std::string &imagename)
{
    if (imagename == "")
    {
        m_layer.setEnabled(false);
        return true;
    }

    auto yarpImageIt = m_loadedImages.find(imagename);

    if (yarpImageIt == m_loadedImages.end())
    {
        yCWarning(OPENXRHEADSET) << "Failed to find the requested image:" << imagename;
        return true;
    }

    m_layer.setEnabled(true);

    if (!m_layer.updateTexture(yarpImageIt->second, 0, 0, yarpImageIt->second.width(), yarpImageIt->second.height()))
    {
        return false;
    }

    return true;
}

bool SlideManager::Options::parseFromConfigurationFile(const std::string &inputPortName, yarp::os::Searchable &labelGroup)
{
    portName = inputPortName;

    slidesPath = labelGroup.check("slides_path", yarp::os::Value("./")).asString();
    initialSlide = labelGroup.check("initial_slide", yarp::os::Value("")).asString();

    return true;
}
