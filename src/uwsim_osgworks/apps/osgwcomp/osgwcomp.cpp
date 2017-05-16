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

#include <osgDB/ReadFile>
#include <osgViewer/CompositeViewer>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>

#include <osgwTools/ParallelVisitor.h>




int
main( int argc,
      char ** argv )
{
    if( argc != 3 )
    {
        osg::notify( osg::FATAL ) << "Usage: osgwcomp <fileA> <fileB>" << std::endl;
        return( 1 );
    }

    osg::ref_ptr< osg::Node > sgA = osgDB::readNodeFile( std::string( argv[1] ) );
    osg::ref_ptr< osg::Node > sgB = osgDB::readNodeFile( std::string( argv[2] ) );
    if( ( sgA == NULL ) || ( sgB == NULL ) )
    {
        osg::notify( osg::FATAL ) << "Failed to load one or both files." << std::endl;
        return( 1 );
    }


    osgwTools::ParallelVisitor pv( sgA.get(), sgB.get() );
    if( pv.compare() )
        osg::notify( osg::ALWAYS ) << "ParallelVisitor indicates match." << std::endl;
    else
        osg::notify( osg::ALWAYS ) << "ParallelVisitor indicates no match." << std::endl;


    //
    // Viewer setup.
    osgViewer::CompositeViewer viewer;

    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi)
    {
        osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
        return 1;
    }
    unsigned int width, height;
    wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);
    const float aspect( (float)width/(float)(height * 2.f) );
    const osg::Matrix proj( osg::Matrix::perspective( 50., aspect, 1., 10. ) );

    // Shared event handlers and state sets.
    osg::ref_ptr< osgGA::TrackballManipulator > tb = new osgGA::TrackballManipulator;

    osg::ref_ptr< osg::StateSet > camSS;
    osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator;

    osg::ref_ptr<osg::GraphicsContext> gc;

    // view one
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView(view);
        view->setSceneData( sgA.get() );

        camSS = view->getCamera()->getOrCreateStateSet();
        statesetManipulator->setStateSet( camSS.get() );
        view->addEventHandler( statesetManipulator.get() );

        view->setCameraManipulator( tb.get() );

        viewer.realize();
        gc = view->getCamera()->getGraphicsContext();
        view->getCamera()->setViewport(new osg::Viewport(0,0, width/2, height));
        view->getCamera()->setProjectionMatrix( proj );
    }

    // view two
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView(view);
        view->setSceneData( sgB.get() );

        view->getCamera()->setStateSet( camSS.get() );
        view->addEventHandler( statesetManipulator.get() );

        view->setCameraManipulator( tb.get() );

        view->getCamera()->setViewport(new osg::Viewport(width/2,0, width/2, height));
        view->getCamera()->setGraphicsContext(gc.get());
        view->getCamera()->setProjectionMatrix( proj );
    }

    return( viewer.run() );
}



/** \page osgwcomp The osgwcomp Application
osgwcomp compares the structure of two scene graphs using the \ref ParallelVisitor.

Further documentation for osgwcomp is TBD.

*/
