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

#include <osgwTools/CameraConfigObject.h>
#include <osgwTools/Version.h>
#include <osgDB/ReadFile>
#include <osg/Object>
#include <osgViewer/Viewer>
#include <osg/Matrixd>

#include <vector>
#include <string>
#include <osg/io_utils>


namespace osgwTools
{


CameraConfigInfo::CameraConfigInfo()
  : _version( 1 )
{
}

CameraConfigInfo::CameraConfigInfo( const osgwTools::CameraConfigInfo& rhs, const osg::CopyOp& )
  : _viewOffset( rhs._viewOffset ),
    _projectionOffset( rhs._projectionOffset ),
    _version( rhs._version )
{
}

CameraConfigInfo::~CameraConfigInfo()
{
}

CameraConfigObject::CameraConfigObject()
  : _version( 1 )
{
}

CameraConfigObject::CameraConfigObject( const osgwTools::CameraConfigObject& rhs, const osg::CopyOp& copyop )
  : _version( rhs._version ),
    _slaveConfigInfo( rhs._slaveConfigInfo )
{
}

CameraConfigObject::~CameraConfigObject()
{
}


void
CameraConfigObject::take( const osgViewer::Viewer& viewer )
{
    if( viewer.getNumSlaves() == 0 )
        return;

    if( _slaveConfigInfo.size() != viewer.getNumSlaves() )
        _slaveConfigInfo.resize( viewer.getNumSlaves() );

    unsigned int idx;
    for( idx=0; idx<viewer.getNumSlaves(); idx++ )
    {
        _slaveConfigInfo[ idx ] = new osgwTools::CameraConfigInfo;
        osgwTools::CameraConfigInfo* cci( _slaveConfigInfo[ idx ].get() );
        const osg::View::Slave& slave = viewer.getSlave( idx );
        cci->_viewOffset = slave._viewOffset;
        cci->_projectionOffset = slave._projectionOffset;
    }
}

void
CameraConfigObject::store( osgViewer::Viewer& viewer )
{
    osg::Camera* masterCamera = viewer.getCamera();


    //
    // What follows is taken from View::setUpViewAcrossAllScreens()
    // and modified as needed to work with what we're doing.
    //

    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi) 
    {
        osg::notify(osg::NOTICE)<<"View::setUpViewAcrossAllScreens() : Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
        return;
    }

    osg::DisplaySettings* ds = masterCamera->getDisplaySettings() != NULL ?
        masterCamera->getDisplaySettings() :
#if( OSGWORKS_OSG_VERSION > 20907 )
            // 2.9.7 2/22/2010
            // r11399, 4/30/2010 Changed DisplaySetting::instance() to return a ref_ptr<>& rathern than a raw C pointer to enable apps to delete the singleton or assign their own.
            // 2.9.8 6/18/2010
            osg::DisplaySettings::instance().get();
#else
            osg::DisplaySettings::instance();
#endif
    
    double fovy, aspectRatio, zNear, zFar;        
    masterCamera->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

    osg::GraphicsContext::ScreenIdentifier si;
    si.readDISPLAY();
    
    // displayNum has not been set so reset it to 0.
    if (si.displayNum<0) si.displayNum = 0;
    
    unsigned int numScreens = wsi->getNumScreens(si);
    if( numScreens != _slaveConfigInfo.size() )
    {
        osg::notify( osg::WARN ) << "Number of screens not equal to number of config slaves." << std::endl;
        return;
    }

    for(unsigned int i=0; i<numScreens; ++i)
    {
        si.screenNum = i;
    
        unsigned int width, height;
        wsi->getScreenResolution(si, width, height);

#if ( OSGWORKS_OSG_VERSION >= 20906 )
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds);
#else
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
#endif
        traits->hostName = si.hostName;
        traits->displayNum = si.displayNum;
        traits->screenNum = si.screenNum;
        traits->screenNum = i;
        traits->x = 0;
        traits->y = 0;
        traits->width = width;
        traits->height = height;
        traits->windowDecoration = false;
        traits->doubleBuffer = true;
        traits->sharedContext = 0;

        osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext(gc.get());

        osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
        if (gw)
        {
            osg::notify(osg::INFO)<<"  GraphicsWindow has been created successfully."<<gw<<std::endl;

            gw->getEventQueue()->getCurrentEventState()->setWindowRectangle(traits->x, traits->y, traits->width, traits->height );
        }
        else
        {
            osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<std::endl;
        }

        camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));

        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);

        const osgwTools::CameraConfigInfo* cci = _slaveConfigInfo[ i ].get();
        viewer.addSlave( camera.get(), cci->_projectionOffset, cci->_viewOffset );
    }

    viewer.assignSceneDataToCameras();
}

bool
configureViewer( osgViewer::Viewer& viewer, const std::string& configFile )
{
    std::string fileName;
    if( !configFile.empty() )
        fileName = configFile;
    else
    {
        const char* buffer( getenv( "OSGW_VIEWER_CONFIG" ) );
        if( buffer != NULL )
            fileName = std::string( buffer );
    }
    if( fileName.empty() )
    {
        osg::notify( osg::INFO ) << "configureViewer: No Viewer config file." << std::endl;
        return( false );
    }

    osg::ref_ptr< osgwTools::CameraConfigObject > cco = 
        dynamic_cast< osgwTools::CameraConfigObject* >(
            osgDB::readObjectFile( fileName ) );
    if( !cco.valid() )
    {
        osg::notify( osg::WARN ) << "configureViewer: Can't load config object from \"" << fileName << "\"." << std::endl;
        return( false );
    }

    cco->store( viewer );

    return( true );
}



// namespace osgwTools
}
