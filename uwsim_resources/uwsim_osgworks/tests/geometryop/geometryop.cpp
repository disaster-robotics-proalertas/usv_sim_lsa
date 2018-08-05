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

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <osg/io_utils>
#include <iostream>
#include <sstream>

#include <osgwTools/GeometryModifier.h>
#include <osgwTools/ReducerOp.h>
#include <osgwTools/DecimatorOp.h>
#include <osgwTools/SimplifierOp.h>
#include <osgwTools/ShortEdgeOp.h>
#include <osgwTools/DecimationTestModel.h>

// TBD #include <osgBullet/VertexAggOp.h>

#include <osg/NodeVisitor>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/TransformAttributeFunctor>
#include <osg/Version>
#include <osg/io_utils>
#include <osgGA/StateSetManipulator>



int main( int argc,
          char* argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " shows a before and after image of the DecimatorOp module, using a default decimation of 0.6." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "--reducer", "Use ReducerOp. (Use DecimatorOp if neither --reducer nor --shortEdge are specified.)" );
    arguments.getApplicationUsage()->addCommandLineOption( "--shortEdge", "Use ShortEdgeOp. (Use DecimatorOp if neither --reducer nor --shortEdge are specified.)" );
    arguments.getApplicationUsage()->addCommandLineOption( "--percent <n>", "Reduction percentage for DecimatorOp and ShortEdgeOp. <n> is the target percentage of triangles to remain, in the range 0.0 to 1.0. Default 0.6." );
    arguments.getApplicationUsage()->addCommandLineOption( "--maxError <n>", "Maximum error tolerance for DecimatorOp, ReducerOp. Geometry exceeding this tolerance is not reduced. <n> is in the range 0.0 to FLT_MAX. Default 10.0." );
    arguments.getApplicationUsage()->addCommandLineOption( "--respectBoundaries", "Prevents DecimatorOp and ShortEdgeOp from removing boundary edges and polygons. Default False." );
    arguments.getApplicationUsage()->addCommandLineOption( "--minPrimitives <n>", "Minimum primitives in a geometry for DecimatorOp and ShortEdgeOp to consider it for reduction. Default 1." );
    arguments.getApplicationUsage()->addCommandLineOption( "--maxFeature <n>", "Specifies the ShortEdgeOp largest feature size to be removed, measured in model units. Can be combined with --percent to limit the decimation using ShortEdgeOp. Default 0.1" );
    arguments.getApplicationUsage()->addCommandLineOption( "--grpThreshold <n>", "Specifies the ReducderOp group threshold, in degrees. Default is 10.0" );
    arguments.getApplicationUsage()->addCommandLineOption( "--attemptMerge", "Attempt to merge geometry drawables into one using osgUtil::Optimizer::MergeGeometryVisitor before using specified geometry reduction operator." );
    arguments.getApplicationUsage()->addCommandLineOption( "--save", "Attempt to merge geometry drawables into one using Optimizer::MergeGeometryVisitor before using specified geometry reduction operator." );

    if( arguments.read( "-h" ) || arguments.read( "--help" ) )
    {
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }


    bool useReducer( arguments.find( "--reducer" ) > 0 );
    bool useShortEdge( arguments.find( "--shortEdge" ) > 0 );
    bool useDecimator( !useReducer && !useShortEdge );

    float percent( .6f );
    arguments.read( "--percent", percent );

    float maxError( 10.f );
    arguments.read( "--maxError", maxError );

    const bool ignoreBoundaries( arguments.read( "--respectBoundaries" ) == 0 );

    int minPrim( 1 );
    arguments.read( "--minPrimitives", minPrim );

    float maxFeature( .1f );
    arguments.read( "--maxFeature", maxFeature );

    float grpThreshold( 10.f );
    arguments.read( "--grpThreshold", grpThreshold );

    bool attemptMerge( arguments.find( "--attemptMerge" ) > 0 );

    bool saveOutput( arguments.find( "--save" ) > 0 );

    osg::Node*  model = osgDB::readNodeFiles( arguments );
    if( model == NULL )
    {
        osg::notify( osg::ALWAYS ) << "Unable to load data file." << std::endl;
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    osg::ref_ptr<osg::Group> grporig = new osg::Group;
    grporig->addChild(model);
    osg::notify( osg::INFO ) << "geometryop: Loaded model(s)." << std::endl;


    osg::ref_ptr<osg::Group> grpcopy = new osg::Group( *grporig , osg::CopyOp::DEEP_COPY_ALL);
    grpcopy->setName("grpcopy"); // for ease of debugging

    osgwTools::GeometryOperation* reducer;
    if( useDecimator )
    {
        osgwTools::DecimatorOp* decimate = new osgwTools::DecimatorOp;
        decimate->setSampleRatio( percent );
        decimate->setMaximumError( maxError );
        decimate->setIgnoreBoundaries( ignoreBoundaries );
        decimate->setMinPrimitives( minPrim );
        reducer = decimate;
    }
    else if( useShortEdge )
    {
        osgwTools::ShortEdgeOp* seOp = new osgwTools::ShortEdgeOp( percent, maxFeature );
        seOp->setIgnoreBoundaries( ignoreBoundaries );
        seOp->setMinPrimitives( minPrim );
        reducer = seOp;
    }
    else if( useReducer )
    {
        osgwTools::ReducerOp* redOp = new osgwTools::ReducerOp;
        redOp->setGroupThreshold( grpThreshold );
        redOp->setMaxEdgeError( maxError );
        reducer = redOp;
    }

    if( reducer != NULL )
    {
        osgwTools::GeometryModifier modifier( reducer );
        modifier.setDrawableMerge( attemptMerge );
        grpcopy->accept( modifier );
        modifier.displayStatistics( osg::notify( osg::ALWAYS ) );
    }

    if( saveOutput )
    {
        std::string modelName( "reduced.osg" );
        osgDB::writeNodeFile( *grpcopy, modelName );
    }

    //
    // Viewer setup.
    osgViewer::CompositeViewer viewer(arguments);

    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi)
    {
        osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
        return 1;
    }
    unsigned int width( 800 ), height( 600 );
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
        view->setUpViewInWindow( 10, 30, width, height );
        viewer.addView(view);
        view->setSceneData( grporig.get() );

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
        view->setSceneData( grpcopy.get() );

        view->getCamera()->setStateSet( camSS.get() );
        view->addEventHandler( statesetManipulator.get() );

        view->setCameraManipulator( tb.get() );

        view->getCamera()->setViewport(new osg::Viewport(width/2,0, width/2, height));
        view->getCamera()->setGraphicsContext(gc.get());
        view->getCamera()->setProjectionMatrix( proj );
    }

    return( viewer.run() );
}
