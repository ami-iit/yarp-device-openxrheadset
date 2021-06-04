/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_OPENXRHEADSET_PORTTOQUADLAYER_H
#define YARP_OPENXRHEADSET_PORTTOQUADLAYER_H

#include <OpenGLConfig.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/os/LogStream.h>

#include <thread>
#include <cassert>

#include <OpenXrInterface.h>
#include <OpenXrHeadsetLogComponent.h>

template <typename ImageType>
class PortToQuadLayer
{
    std::shared_ptr<IOpenXrQuadLayer> m_quadLayer{nullptr};
    std::shared_ptr<yarp::os::BufferedPort<ImageType>> m_portPtr;
    GLuint m_glWriteBufferId = 0;
    GLuint m_glReadBufferId = 0;
    GLuint m_imageTexture;
    GLint m_pixelFormat;
    std::thread::id m_initThreadID;

public:

    bool initialize(std::shared_ptr<IOpenXrQuadLayer> quadLayer, const std::string& portName)
    {
        yCTrace(OPENXRHEADSET);

        if (!quadLayer)
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

        ImageType dummyImage; //Needed to get the pixel code
        int pixelCode = dummyImage.getPixelCode();

        if (pixelCode == VOCAB_PIXEL_RGBA)
        {
            m_pixelFormat = GL_RGBA;
        }
        else if (pixelCode == VOCAB_PIXEL_RGB)
        {
            m_pixelFormat = GL_RGB;
        }
        else
        {
            yCError(OPENXRHEADSET) << "Use this class only with RGB or RGBA images.";
            return false;
        }

        //Generate internal framebuffer
        glGenFramebuffers(1, &(m_glWriteBufferId));
        glGenFramebuffers(1, &(m_glReadBufferId));
        glBindFramebuffer(GL_FRAMEBUFFER, m_glReadBufferId);
        glGenTextures(1, &m_imageTexture);

        //Trying to preallocate texture for read frame buffer
        glBindTexture(GL_TEXTURE_2D, m_imageTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, m_imageTexture, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, m_pixelFormat, quadLayer->imageMaxWidth(), quadLayer->imageMaxHeight(), 0,
            m_pixelFormat, GL_UNSIGNED_BYTE, NULL);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);


        m_quadLayer = quadLayer;
        m_quadLayer->useAlphaChannel(m_pixelFormat == GL_RGBA);

        m_portPtr = std::make_shared<yarp::os::BufferedPort<ImageType>>();

        if (!m_portPtr->open(portName))
        {
            yCError(OPENXRHEADSET) << "Failed to open the port named" << portName << ".";
            return false;
        }

        m_portPtr->setReadOnly();

        return true;
    }

    bool updateTexture()
    {
        yCTrace(OPENXRHEADSET);
        assert(m_initThreadID == std::this_thread::get_id() &&
               "The updateTexture has to be called from the same thread in which it has been initialized.");

        ImageType* img = m_portPtr->read(false);

        if (!img)
        {
            return true;
        }

        if ((img->width() == 0) || (img->height() == 0))
        {
            return true;
        }

        if (!m_quadLayer)
        {
            yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
            return false;
        }

        uint32_t textureImage;

        if (!m_quadLayer->getImage(textureImage))
        {
            yCError(OPENXRHEADSET) << "Failed to get texture image.";
            return false;
        }

        //First bind the read framebuffer, using the input image as source
        //This has to be called from the same thread where the initialize method has been called
        glBindFramebuffer(GL_FRAMEBUFFER, m_glReadBufferId);
        glBindTexture(GL_TEXTURE_2D, m_imageTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, m_pixelFormat, img->width(), img->height(),
            0, m_pixelFormat, GL_UNSIGNED_BYTE, img->getRawImage());

        //Then bind the write framebuffer, using the OpenXr texture as target texture
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_glWriteBufferId);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureImage, 0);

        //Copy from the read framebuffer to the draw framebuffer
        glBlitFramebuffer(0, img->height(), img->width(), 0,
            0, 0, m_quadLayer->imageMaxWidth(), m_quadLayer->imageMaxHeight(),
            GL_COLOR_BUFFER_BIT, GL_NEAREST);   // When writing the texture, OpenGl starts from the bottom left corner and
                                                // goes from left to right, bottom to up.
                                                // See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml
                                                // In Yarp, the first pixel is the top left, so we have to flip vertically.

        //Resetting read and draw framebuffers
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        //Adjust the aspect ratio
        float aspectRatio = img->width()/ (float) img->height();
        float newWidth = m_quadLayer->layerWidth();
        float newHeight = m_quadLayer->layerHeight();

        if (aspectRatio >= 1)
        {
            newHeight = newWidth / aspectRatio;
        }
        else
        {
            newWidth = newHeight * aspectRatio;
        }

        m_quadLayer->setDimensions(newWidth, newHeight);

        if (!m_quadLayer->submitImage())
        {
            yCError(OPENXRHEADSET) << "Failed to submit texture image.";
            return false;
        }

        return true;
    }

    void setPose(const Eigen::Vector3f& position,
                         const Eigen::Quaternionf &rotation)
    {
        yCTrace(OPENXRHEADSET);

        if (!m_quadLayer)
        {
            yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
            return;
        }

        m_quadLayer->setPose(position, rotation);
    }

    void setPosition(const Eigen::Vector3f& position)
    {
        yCTrace(OPENXRHEADSET);

        if (!m_quadLayer)
        {
            yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
            return;
        }

        m_quadLayer->setPosition(position);
    }

    void setRotation(const Eigen::Quaternionf &rotation)
    {
        yCTrace(OPENXRHEADSET);

        if (!m_quadLayer)
        {
            yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
            return;
        }

        m_quadLayer->setRotation(rotation);
    }

    void setDimensions(float widthInMeters, float heightInMeters)
    {
        yCTrace(OPENXRHEADSET);

        if (!m_quadLayer)
        {
            yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
            return;
        }

        m_quadLayer->setDimensions(widthInMeters, heightInMeters);
    }

    void setVisibility(const IOpenXrQuadLayer::Visibility& visibility)
    {
        yCTrace(OPENXRHEADSET);

        if (!m_quadLayer)
        {
            yCError(OPENXRHEADSET) << "The initialization phase did not complete correctly.";
            return;
        }

        m_quadLayer->setVisibility(visibility);
    }

};

#endif // YARP_OPENXRHEADSET_PORTTOQUADLAYER_H
