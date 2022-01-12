/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <LabelPortToQuadLayer.h>
#include <filesystem>

bool LabelPortToQuadLayer::initialize(const Options &options)
{
    yCTrace(OPENXRHEADSET);

    if (!options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The input quadLayer is not valid.";
        return false;
    }

    //This has to be called from the same thread where the window has been created
    if (!glfwInit()) {
        yCError(OPENXRHEADSET, "Unable to initialize GLFW");
        return false;
    }

    m_initThreadID = std::this_thread::get_id();

    //Generate internal framebuffer
    glGenFramebuffers(1, &(m_glWriteBufferId));
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    std::string fontPath = options.fontPath;
    if (!std::filesystem::exists(fontPath))
    {
        fontPath = GLFont::DefaultFontsPathPrefix() + fontPath;
        if (!std::filesystem::exists(fontPath))
        {
            yCError(OPENXRHEADSET) << "Failed to find font:" << fontPath + "."
                                   << "The font can be either a global or a relative path to a font file."
                                   << "You can also specify a font in the folder " << GLFont::DefaultFontsPathPrefix()
                                   << "as a relative path (for example \"Roboto/Roboto-Light.ttf\").";
            return false;
        }
    }

    m_glFont = std::make_shared<GLFont>(fontPath);
    m_glLabel = std::make_shared<FTLabel>(m_glFont, options.labelPrefix + options.labelSuffix, 0, 0, options.quadLayer->imageMaxWidth(), options.quadLayer->imageMaxHeight());
    m_glLabel->setPixelSize(options.pixelSize);
    m_glLabel->setColor(options.labelColor[0], options.labelColor[1], options.labelColor[2], options.labelColor[3]);

    m_portPtr = std::make_shared<yarp::os::BufferedPort<yarp::os::Bottle>>();

    if (!m_portPtr->open(options.portName))
    {
        yCError(OPENXRHEADSET) << "Failed to open the port named" << options.portName << ".";
        return false;
    }

    m_portPtr->setReadOnly();

    m_options = options;
    m_options.quadLayer->useAlphaChannel(true);

    return true;
}

void LabelPortToQuadLayer::close()
{
    if (m_glWriteBufferId != 0)
    {
        glDeleteFramebuffers(1, &m_glWriteBufferId);
        m_glWriteBufferId = 0;
    }

    m_portPtr->close();
}

bool LabelPortToQuadLayer::updateTexture()
{
    yCTrace(OPENXRHEADSET);
    assert(m_initThreadID == std::this_thread::get_id() &&
           "The updateTexture has to be called from the same thread in which it has been initialized.");

    yarp::os::Bottle* bottle = m_portPtr->read(false);

    if (bottle && bottle->size() > 0)
    {
        m_inputString = bottle->get(0).toString();
    }

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return false;
    }

    std::string textToDisplay = m_options.labelPrefix + m_inputString + m_options.labelSuffix;

    if (textToDisplay.empty())
    {
        return true;
    }

    m_glLabel->setText(textToDisplay);
    int horizontalPosition = 0;
    switch (m_options.horizontalAlignement)
    {
    case Options::HorizontalAlignement::Center:
        m_glLabel->setAlignment(FTLabel::FontFlags::CenterAligned);
        horizontalPosition = m_options.quadLayer->imageMaxWidth()/2;
        break;
    case Options::HorizontalAlignement::Left:
        m_glLabel->setAlignment(FTLabel::FontFlags::LeftAligned);
        horizontalPosition = 0;
        break;
    case Options::HorizontalAlignement::Right:
        m_glLabel->setAlignment(FTLabel::FontFlags::RightAligned);
        horizontalPosition = m_options.quadLayer->imageMaxWidth();
        break;
    }

    int verticalPosition = 0;
    int labelHeight = m_glLabel->getCurrentLabelHeight();
    switch (m_options.verticalAlignement)
    {
    case Options::VerticalAlignement::Center:
        verticalPosition = m_options.quadLayer->imageMaxHeight()/2 - labelHeight / 2;
        break;
    case Options::VerticalAlignement::Top:
        verticalPosition = 0;
        break;
    case Options::VerticalAlignement::Bottom:
        verticalPosition = m_options.quadLayer->imageMaxHeight() - labelHeight;
        break;
    }

    m_glLabel->setPosition(horizontalPosition, verticalPosition);

    uint32_t textureImage;

    if (!m_options.quadLayer->getImage(textureImage))
    {
        yCError(OPENXRHEADSET) << "Failed to get texture image.";
        return false;
    }

    //Bind the write framebuffer, using the OpenXr texture as target texture
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_glWriteBufferId);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureImage, 0);

    glClearColor(m_options.backgroundColor[0], m_options.backgroundColor[1],
                 m_options.backgroundColor[2], m_options.backgroundColor[3]);
    glClear(GL_COLOR_BUFFER_BIT);

    m_glLabel->render(); //Renders to the texture

    //Resetting read and draw framebuffers
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    if (!m_options.quadLayer->submitImage())
    {
        yCError(OPENXRHEADSET) << "Failed to submit texture image.";
        return false;
    }

    m_active = true;

    return true;
}

void LabelPortToQuadLayer::setPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &rotation)
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setPose(position, rotation);
}

void LabelPortToQuadLayer::setPosition(const Eigen::Vector3f &position)
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setPosition(position);
}

Eigen::Vector3f LabelPortToQuadLayer::layerPosition() const
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return Eigen::Vector3f::Zero();
    }

    return m_options.quadLayer->layerPosition();
}

void LabelPortToQuadLayer::setRotation(const Eigen::Quaternionf &rotation)
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setRotation(rotation);
}

Eigen::Quaternionf LabelPortToQuadLayer::layerQuaternion() const
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return Eigen::Quaternionf::Identity();
    }

    return m_options.quadLayer->layerQuaternion();
}

void LabelPortToQuadLayer::setDimensions(float widthInMeters, float heightInMeters)
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setDimensions(widthInMeters, heightInMeters);
}

void LabelPortToQuadLayer::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setVisibility(visibility);
}

float LabelPortToQuadLayer::layerWidth() const
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return 0.0;
    }

    return m_options.quadLayer->layerWidth();
}

float LabelPortToQuadLayer::layerHeight() const
{
    yCTrace(OPENXRHEADSET);

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return 0.0;
    }

    return m_options.quadLayer->layerHeight();
}

bool LabelPortToQuadLayer::active() const
{
    yCTrace(OPENXRHEADSET);

    return m_active;
}
