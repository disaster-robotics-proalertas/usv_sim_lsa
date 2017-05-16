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
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgGA/GUIEventHandler>

/* \cond */
class KeyHandler : public osgGA::GUIEventHandler
{
public:
    KeyHandler( osgwTools::ScreenCapture* sc )
      : _sc( sc )
    {}

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if( ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN )
        {
            if( ea.getKey() == 'c' )
            {
                _sc->setCapture( !_sc->getCaptureEnabled() );
                return( true );
            }
            else
            {
                int keyv( ea.getKey() - '0' );
                if( (keyv > 0) && (keyv < 10) )
                {
                    // keys 1 through 9 capture N frames
                    _sc->setNumFramesToCapture( keyv );
                    _sc->setCapture( true );
                }
            }
        }
        return( false );
    }

private:
    osgwTools::ScreenCapture* _sc;
};
/* \endcond */



int
main( int argc, char ** argv )
{
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 40, 800, 600 );
    viewer.setSceneData( osgDB::readNodeFile( "cow.osg" ) );
    viewer.realize();

    osg::ref_ptr< osgwTools::ScreenCapture > sc = new osgwTools::ScreenCapture(
        std::string( "cap" ), std::string( ".png" ) );

    KeyHandler* kh = new KeyHandler( sc.get() );
    viewer.addEventHandler( kh );

    viewer.getCamera()->setPostDrawCallback( sc.get() );

    return( viewer.run() );
}

