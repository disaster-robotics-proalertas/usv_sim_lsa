/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2012 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#ifndef __OSGWTOOLS_SCREEN_CAPTURE_H__
#define __OSGWTOOLS_SCREEN_CAPTURE_H__ 1

#include <osgwTools/Export.h>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <osg/Camera>
#include <osg/FrameStamp>

#include <string>
#include <list>

namespace osgwTools
{


/** \brief A Camera post-draw callback to read the rendered image from the
current OpenGL read buffer and write the image data to an image file

File writes are performed in a separate thread, so the disk IO doesn't
bottleneck the rendering thread.

\test screencap

*/
struct OSGWTOOLS_EXPORT ScreenCapture : public osg::Camera::DrawCallback
{
public:
    ScreenCapture( const std::string& rootName=std::string( "" ), const std::string& ext=std::string( ".png" ), bool useFrameNum=true );
    ~ScreenCapture();

    virtual void operator()( osg::RenderInfo& ri ) const;

    /** Passes true to start capture and false to stop. While on, this class 
	captures each frame from the framebuffer and writes it to a file. */
    void setCapture( bool enable );
    bool getCaptureEnabled() const;

    /** Sets to a positive number of frames to capture a specific frame count.
    If set to 0 (the default), capture continues until it explicitly
    disables with setCapture(false). */
    void setNumFramesToCapture( unsigned int numFrames );

    /** File name is \b rootname \b framenum \b extension.
    \li \b rootname defaults to screencapture if blank.
    \li \b framenum is the frame number, unless useFrameNum is false.
    \li \b extension is .png by default. Note it includes the dot.
    */
    void setRootName( const std::string& name );
    void setExtension( const std::string& extension );
    void setUseFrameNumber( bool useFrameNum );

    /** The default is full screen. Passes non-NULL to specify a viewport other than full screen. */
    void setViewport( osg::Viewport* vp );

protected:
    std::string rootName_;
    std::string ext_;
    bool useFrameNum_;
    osg::ref_ptr< osg::Viewport > vp_;

    mutable bool captureOn_;
    mutable unsigned int numFrames_;

    std::string getFileName( osg::FrameStamp* fs ) const;


    typedef std::list< osg::ref_ptr< osg::Image > > ImageList;

    /* \cond */
    class WriteImageThread : public OpenThreads::Thread
    {
    public:
        OpenThreads::Mutex lock_;
        ImageList imageList_;

        WriteImageThread();

        virtual void run();
    }; /* \endcond */
    mutable WriteImageThread* wit_;
};


// osgwTools
}

// __OSGWTOOLS_SCREEN_CAPTURE_H__
#endif
