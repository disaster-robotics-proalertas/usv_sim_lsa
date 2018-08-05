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

#include <osgwQuery/QueryUtils.h>
#include <osgwQuery/QueryBenchmarks.h>
#include <osgwQuery/QueryStats.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/FindNamedNode.h>
#include <osgUtil/RenderBin>

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/Optimizer>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>



osg::Node* makeSceneA()
{
    osg::Group* grp = new osg::Group;
    osg::Geode* geode = new osg::Geode;
    grp->addChild( geode );
    osg::Geometry* geom = osgwTools::makeGeodesicSphere( 3., 5 );
    geode->addDrawable( geom );

    return( grp );
}
osg::Node* makeSceneB()
{
    osg::Group* grp = new osg::Group;
    osg::Geode* geode = new osg::Geode;
    grp->addChild( geode );
    osg::Geometry* geom = osgwTools::makePlane( osg::Vec3( -10., -50., -10. ),
        osg::Vec3( 20., 0., 0. ), osg::Vec3( 0., 0., 20. ) );
    geode->addDrawable( geom );

    return( grp );
}

int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    std::string debugNodeName;
    bool debugStats( arguments.read( "--debug", debugNodeName ) );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 0., 0., 1024., 768. );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator );
    viewer.addEventHandler( new osgViewer::StatsHandler );
    viewer.addEventHandler( new osgGA::StateSetManipulator(
        viewer.getCamera()->getOrCreateStateSet() ) );


    osg::Group* root = new osg::Group;
    root->setName( "Test root node" );
    osg::Node* models = osgDB::readNodeFiles( arguments );
    if( models != NULL )
    {
        osgUtil::Optimizer opt;
        opt.optimize( models, osgUtil::Optimizer::REMOVE_LOADED_PROXY_NODES );
        root->addChild( models );
    }
    else
    {
        root->addChild( makeSceneA() );
        root->addChild( makeSceneB() );
    }
    // Any models using occlusion query must render in front-to-back order.
    root->getOrCreateStateSet()->setRenderBinDetails( 0, _QUERY_FRONT_TO_BACK_BIN_NAME );


    // Add the Query statistics HUD.
    osg::ref_ptr< osgwQuery::QueryStats > qs;
    if( debugStats )
    {
        osgwTools::FindNamedNode fnn( debugNodeName );
        root->accept( fnn );
        if( fnn._napl.empty() )
            osg::notify( osg::ALWAYS ) << "Debug error: Couldn't find any nodes named " << debugNodeName << std::endl;
        else
        {
            qs = new osgwQuery::QueryStats( fnn._napl[ 0 ].first );
            //qs->setConsoleDisplay(); // In case nothing appears onscreen.
            root->addChild( qs->getSceneGraph() );

            viewer.addEventHandler( new osgwQuery::QueryStatsHandler( qs.get() ) );
        }
    }

    viewer.setSceneData( root );
    viewer.setThreadingModel( osgViewer::ViewerBase::SingleThreaded );

    // Realized the viewer, then root's parent should
    // be the top-level Camera. We want to add queries starting at that node.
    viewer.realize();
    osgwQuery::AddQueries aqs;
    aqs.setQueryStats( qs.get() );
    root->getParent( 0 )->accept( aqs );
    osg::notify( osg::ALWAYS ) << "Added " << aqs.getQueryCount() << " queries." << std::endl;

    // Guthe algorithm requires gathering of init-time constants. We do
    // this with a pre-draw callback.
    viewer.getCamera()->setPreDrawCallback( new osgwQuery::InitCallback() );

    while( !viewer.done() )
    {
        osg::notify( osg::INFO ) << "        *** Frame ***" << std::endl;

        viewer.frame();

        if( qs.valid() )
            qs->incFrames();
    }
}
