/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <LabelPortToQuadLayer.h>
#include <filesystem>
#include <yarp/os/Time.h>

//#define DRAW_DEBUG_RECTANGLES

bool LabelPortToQuadLayer::initialize(const Options &options)
{
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
    glGenFramebuffers(1, &(m_glReadBufferId));
    glBindFramebuffer(GL_FRAMEBUFFER, m_glReadBufferId);
    glGenTextures(1, &m_imageTexture);

    //Trying to preallocate texture for read frame buffer
    glBindTexture(GL_TEXTURE_2D, m_imageTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, m_imageTexture, 0);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8,  options.quadLayer->imageMaxWidth(), options.quadLayer->imageMaxHeight());

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
    m_glLabel = std::make_shared<FTLabel>(m_glFont, options.labelPrefix + options.labelSuffix, 0, 0, options.quadLayer->imageMaxWidth(), options.quadLayer->imageMaxHeight(),
                                          options.quadLayer->imageMaxWidth(), options.quadLayer->imageMaxHeight());
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

    m_lastReceived = yarp::os::Time::now();

    return true;
}

void LabelPortToQuadLayer::close()
{
    if (m_glWriteBufferId != 0)
    {
        glDeleteFramebuffers(1, &m_glWriteBufferId);
        m_glWriteBufferId = 0;
    }

    if (m_glReadBufferId != 0)
    {
        glDeleteFramebuffers(1, &m_glReadBufferId);
        m_glReadBufferId = 0;
    }

    m_portPtr->close();
    m_options.quadLayer = nullptr;
}

bool LabelPortToQuadLayer::updateTexture()
{
    assert(m_initThreadID == std::this_thread::get_id() &&
           "The updateTexture has to be called from the same thread in which it has been initialized.");

    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return false;
    }

    yarp::os::Bottle* bottle = m_portPtr->read(false);

    bool received = bottle && bottle->size() > 0;
    if (received)
    {
        m_inputString = bottle->get(0).toString();
        m_lastReceived = yarp::os::Time::now();
        if (m_timeoutExpired)
        {
            m_shouldResume = true;
            m_timeoutExpired = false;
        }
    }

    std::string textToDisplay = m_options.labelPrefix + m_inputString + m_options.labelSuffix;

    bool inactive = m_options.disableTimeoutInS >= 0 &&
            ((yarp::os::Time::now() - m_lastReceived) > m_options.disableTimeoutInS); //->disable
    bool sameText = textToDisplay == m_glLabel->getText(); //->skip
    bool printAnyway = (m_firstTime && m_options.automaticallyEnabled) || m_shouldResume;

    if (!m_enabled)
    {
        m_options.quadLayer->setEnabled(false);
        return true;
    }

    if (!printAnyway)
    {
        if (inactive)
        {
            m_options.quadLayer->setEnabled(false);
            m_timeoutExpired = true;
            return true;
        }

        if (sameText)
        {
            return true;
        }
    }

    m_options.quadLayer->setEnabled(true);

    float textureAspectRatio = m_options.quadLayer->imageMaxWidth() / (float) m_options.quadLayer->imageMaxHeight();
    float layerAspectRatio = m_options.quadLayer->layerWidth() / (float) m_options.quadLayer->layerHeight();

    float aspectRatioRatio = textureAspectRatio/layerAspectRatio;
    m_glLabel->setFontAspectRatio(aspectRatioRatio);

    m_glLabel->setText(textToDisplay);
    float horizontalPosition = 0;
    switch (m_options.horizontalAlignement)
    {
    case Options::HorizontalAlignement::Center:
        m_glLabel->setAlignment(FTLabel::FontFlags::CenterAligned);
        horizontalPosition = m_options.quadLayer->imageMaxWidth() * 0.5;
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

    float verticalPosition = 0;
    int labelHeight = m_glLabel->getCurrentLabelHeight();
    switch (m_options.verticalAlignement)
    {
    case Options::VerticalAlignement::Center:
        verticalPosition = std::round((m_options.quadLayer->imageMaxHeight() - labelHeight) * 0.5);
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

    //First bind the read framebuffer, using the input image as source
    //This has to be called from the same thread where the initialize method has been called
    glBindFramebuffer(GL_FRAMEBUFFER, m_glReadBufferId);
    glBindTexture(GL_TEXTURE_2D, m_imageTexture);

    glClearColor(m_options.backgroundColor[0], m_options.backgroundColor[1],
                 m_options.backgroundColor[2], m_options.backgroundColor[3]);
    glClear(GL_COLOR_BUFFER_BIT);

    GLint initialViewport[4];
    glGetIntegerv(GL_VIEWPORT, initialViewport);

    glViewport(0, 0, m_options.quadLayer->imageMaxWidth(), m_options.quadLayer->imageMaxHeight());

# ifdef DRAW_DEBUG_RECTANGLES
    auto colRect = [](float r, float g, float b, float a, float x, float y, float h, float w)
    {
        glBegin(GL_QUADS);
        glColor4f(r, g, b, a);
        glVertex3f(x,y,0);
        glVertex3f(x+w,y,0);
        glVertex3f(x+w,y+h,0);
        glVertex3f(x,y+h,0);
        glEnd();
        glColor4f(1, 1, 1, 1);
    };

    colRect(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.5, 0.1); //red
    colRect(0.0, 1.0, 0.0, 1.0, 0.1, -1.0, 1.0, 0.1); //green
    colRect(0.0, 0.0, 1.0, 1.0, 0.2, 0.0, 0.9, 0.1); //blue
#endif


    m_glLabel->render(); //Renders to the texture

//    Then bind the write framebuffer, using the OpenXr texture as target texture
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_glWriteBufferId);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureImage, 0);

    //Copy from the read framebuffer to the draw framebuffer
    glBlitNamedFramebuffer(m_glReadBufferId, m_glWriteBufferId,
                           0, 0, m_options.quadLayer->imageMaxWidth(), m_options.quadLayer->imageMaxHeight(),
                           0, 0, m_options.quadLayer->imageMaxWidth(), m_options.quadLayer->imageMaxHeight(),
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);

    glViewport(initialViewport[0], initialViewport[1],
               initialViewport[2], initialViewport[3]);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    if (!m_options.quadLayer->submitImage())
    {
        yCError(OPENXRHEADSET) << "Failed to submit texture image.";
        return false;
    }

    m_firstTime = false;
    m_shouldResume = false;

    return true;
}

void LabelPortToQuadLayer::setPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &rotation)
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setPose(position, rotation);
}

void LabelPortToQuadLayer::setPosition(const Eigen::Vector3f &position)
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setPosition(position);
}

Eigen::Vector3f LabelPortToQuadLayer::layerPosition() const
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return Eigen::Vector3f::Zero();
    }

    return m_options.quadLayer->layerPosition();
}

void LabelPortToQuadLayer::setQuaternion(const Eigen::Quaternionf &quaternion)
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setQuaternion(quaternion);
}

Eigen::Quaternionf LabelPortToQuadLayer::layerQuaternion() const
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return Eigen::Quaternionf::Identity();
    }

    return m_options.quadLayer->layerQuaternion();
}

void LabelPortToQuadLayer::setDimensions(float widthInMeters, float heightInMeters)
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setDimensions(widthInMeters, heightInMeters);
}

void LabelPortToQuadLayer::setVisibility(const IOpenXrQuadLayer::Visibility &visibility)
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return;
    }

    m_options.quadLayer->setVisibility(visibility);
}

float LabelPortToQuadLayer::layerWidth() const
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return 0.0;
    }

    return m_options.quadLayer->layerWidth();
}

float LabelPortToQuadLayer::layerHeight() const
{
    if (!m_options.quadLayer)
    {
        yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
        return 0.0;
    }

    return m_options.quadLayer->layerHeight();
}

void LabelPortToQuadLayer::setEnabled(bool enabled)
{
    m_shouldResume = enabled;
    m_lastReceived = yarp::os::Time::now();
    m_timeoutExpired = false;
    m_enabled = enabled;
}

bool LabelPortToQuadLayer::Options::parseFromConfigurationFile(const std::string& inputPortName, yarp::os::Searchable &labelGroup)
{
    portName = inputPortName;
    labelPrefix = labelGroup.check("prefix", yarp::os::Value("")).asString();
    labelSuffix = labelGroup.check("suffix", yarp::os::Value("")).asString();
    fontPath = labelGroup.check("font", yarp::os::Value("Roboto/Roboto-Black.ttf")).asString();
    pixelSize = labelGroup.check("pixel_size", yarp::os::Value(64)).asInt32();
    automaticallyEnabled = labelGroup.check("automatically_enabled", yarp::os::Value(true)).asBool();
    disableTimeoutInS = labelGroup.check("disable_timeout_in_S", yarp::os::Value(-1.0)).asFloat64();

    std::string inputHorAlignement = labelGroup.check("horizontal_alignement", yarp::os::Value("center")).asString();
    std::transform(inputHorAlignement.begin(), inputHorAlignement.end(), inputHorAlignement.begin(), ::tolower);
    if (inputHorAlignement == "left")
    {
        horizontalAlignement = LabelPortToQuadLayer::Options::HorizontalAlignement::Left;
    }
    else if (inputHorAlignement == "right")
    {
        horizontalAlignement = LabelPortToQuadLayer::Options::HorizontalAlignement::Right;
    }
    else if (inputHorAlignement == "center")
    {
        horizontalAlignement = LabelPortToQuadLayer::Options::HorizontalAlignement::Center;
    }
    else
    {
        yCError(OPENXRHEADSET) << "Unrecognized horizontal_alignement."
                               << "Allowed entries: \"left\", \"right\", \"center\".";
        return false;
    }

    std::string inputVertAlignement = labelGroup.check("vertical_alignement", yarp::os::Value("center")).asString();
    std::transform(inputVertAlignement.begin(), inputVertAlignement.end(), inputVertAlignement.begin(), ::tolower);
    if (inputVertAlignement == "top")
    {
        verticalAlignement = LabelPortToQuadLayer::Options::VerticalAlignement::Top;
    }
    else if (inputVertAlignement == "bottom")
    {
        verticalAlignement = LabelPortToQuadLayer::Options::VerticalAlignement::Bottom;
    }
    else if (inputVertAlignement == "center")
    {
        verticalAlignement = LabelPortToQuadLayer::Options::VerticalAlignement::Center;
    }
    else
    {
        yCError(OPENXRHEADSET) << "Unrecognized vertical_alignement."
                               << "Allowed entries: \"top\", \"bottom\", \"center\".";
        return false;
    }

    auto fetchColor = [](const yarp::os::Searchable& group, const std::string& paramName,
            const Eigen::Vector4f& defaultColor, Eigen::Vector4f& outputColor) -> bool
    {
        if (!group.check(paramName))
        {
            outputColor = defaultColor;
            return true;
        }

        yarp::os::Value list = group.find(paramName);
        if (!list.isList())
        {
            yCError(OPENXRHEADSET) << "The parameter" << paramName << "is specified but it is not a list.";
            return false;
        }

        yarp::os::Bottle* yarpVector = list.asList();

        if (yarpVector->size() != 4)
        {
            yCError(OPENXRHEADSET) << "The parameter" << paramName << "is specified but it is not a list of size 4.";
            return false;
        }

        for (size_t i = 0; i < 4; ++i)
        {
            if (!yarpVector->get(i).isFloat64())
            {
                yCError(OPENXRHEADSET) << "The entry" << i << " of parameter" << paramName << "is not a double.";
                return false;
            }
            double value = yarpVector->get(i).asFloat64();

            if (value < 0.0 || value > 1.0)
            {
                yCError(OPENXRHEADSET) << "The entry" << i << " of parameter" << paramName << "is not in the range [0, 1].";
                return false;
            }
            outputColor[i] = value;
        }

        return true;
    };

    if (!fetchColor(labelGroup, "color", {1.0, 1.0, 1.0, 1.0}, labelColor))
    {
        return false;
    }

    if (!fetchColor(labelGroup, "background_color",  {0.0, 0.0, 0.0, 0.0}, backgroundColor))
    {
        return false;
    }
    return true;
}
