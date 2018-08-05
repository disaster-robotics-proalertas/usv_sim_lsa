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

#include <osgwTools/ScreenCapture.h>
#include <osg/Camera>
#include <osg/Image>
#include <osg/FrameStamp>
#include <osgDB/WriteFile>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <string>
#include <sstream>


using namespace osgwTools;


ScreenCapture::ScreenCapture( const std::string& rootName, const std::string& ext, bool useFrameNum )
  : rootName_( rootName ),
    ext_( ext ),
    useFrameNum_( useFrameNum ),
    captureOn_( false ),
    numFrames_( 0 ),
    wit_( NULL )
{
}

ScreenCapture::~ScreenCapture()
{
    if( wit_ != NULL )
    {
        if( wit_->isRunning() )
        {
            wit_->cancel();
            wit_->join();
        }
        if( wit_->isRunning() )
            osg::notify( osg::ALWAYS ) << "Thread is running after join() call." << std::endl;
        delete wit_;
        wit_ = NULL;
    }
}

void
ScreenCapture::operator()( osg::RenderInfo& ri ) const
{
    if( captureOn_ )
    {
        bool newThread( false );
        if( wit_ == NULL )
        {
            wit_ = new WriteImageThread();
            newThread = true;
        }
        osg::Image* image = new osg::Image;
        image->setFileName( 
            getFileName( useFrameNum_ ? ri.getState()->getFrameStamp() : NULL ) );

        osg::notify( osg::INFO ) << "ScreenCapture: Reading image for file " << image->getFileName() << " ... " << std::endl;
        const osg::Viewport* vp( ( vp_ == NULL ) ?
            ri.getState()->getCurrentViewport() : vp_.get() );
        image->readPixels( vp->x(), vp->y(), vp->width(), vp->height(), GL_RGBA, GL_UNSIGNED_BYTE );
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > lock( wit_->lock_ );
            wit_->imageList_.push_back( image );
        }

        if( numFrames_ > 0 )
        {
            if( --numFrames_ == 0 )
                captureOn_ = false;
        }

        if( newThread )
            wit_->start();
    }
    else
    {
        if( ( wit_ != NULL ) )
        {
            osg::notify( osg::INFO ) << "ScreenCapture: Thread cleanup" << std::endl;
            if( wit_->isRunning() )
            {
                wit_->cancel();
                wit_->join();
            }
            if( wit_->isRunning() )
                osg::notify( osg::ALWAYS ) << "Thread is running after join() call." << std::endl;
            delete wit_;
            wit_ = NULL;
        }
    }
}

void
ScreenCapture::setCapture( bool enable )
{
    captureOn_ = enable;
}
bool
ScreenCapture::getCaptureEnabled() const
{
    return( captureOn_ );
}
void
ScreenCapture::setNumFramesToCapture( unsigned int numFrames )
{
    numFrames_ = numFrames;
}


void
ScreenCapture::setRootName( const std::string& name )
{
    rootName_ = name;
}
void
ScreenCapture::setExtension( const std::string& extension )
{
    ext_ = extension;
}
void
ScreenCapture::setUseFrameNumber( bool useFrameNum )
{
    useFrameNum_ = useFrameNum;
}
void
ScreenCapture::setViewport( osg::Viewport* vp )
{
    vp_ = vp;
}


std::string
ScreenCapture::getFileName( osg::FrameStamp* fs ) const
{
    std::string fileName;
    if( !rootName_.empty() )
        fileName = rootName_;
    else
        fileName = "screencapture";

    if( fs != NULL )
    {
        std::ostringstream ostr;
        ostr << fs->getFrameNumber();
        fileName += ostr.str();
    }

    fileName += ext_;

    return( fileName );
}




ScreenCapture::WriteImageThread::WriteImageThread()
{
}

void
ScreenCapture::WriteImageThread::run()
{
    osg::ref_ptr< osg::Image > image( NULL );
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( lock_ );
        if( !imageList_.empty() )
        {
            image = imageList_.front();
            imageList_.pop_front();
        }
    }
    bool finished( image == NULL );
    while( !finished )
    {
        if( image != NULL )
        {
            osg::notify( osg::INFO ) << "ScreenCapture: Writing \"" << image->getFileName() << "\"" << std::endl;
            osgDB::writeImageFile( *image, image->getFileName() );
            image = NULL;
        }
        else
        {
            YieldCurrentThread();
            microSleep( 500 );
        }

        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > lock( lock_ );
            if( !imageList_.empty() )
            {
                image = imageList_.front();
                imageList_.pop_front();
            }
        }
        finished = ( (image == NULL) && testCancel() );
    }
}
