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
#include <osgwTools/DecimationTestModel.h>
#include <osgwTools/LODCreationNodeVisitor.h>

#include <osg/NodeVisitor>
#include <osg/Version>
#include <osg/io_utils>
#include <osgGA/StateSetManipulator>



int main( int argc,
          char* argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " shows a before and after image of the Create LOD Visitor module." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "--retPercent <n>", "<n> is the minimum percentage of triangles to remain, in the range 0.0 to 1.0. Default 0.01" );
    arguments.getApplicationUsage()->addCommandLineOption( "--ignoreBoundaries", "Will collapse boundary edges perhaps leading to greater reduction and more visual degradation. Default False" );
    arguments.getApplicationUsage()->addCommandLineOption( "--minPrimitives <n>", "Sets the minimum primitives a geometry must have to start LOD reduction. Default 100." );
    arguments.getApplicationUsage()->addCommandLineOption( "--attemptMerge <n>", "Attempts to merge drawables within the model prior to any geometry reduction using a osgUtil::Optimizer::MergeGeometryVisitor. In cases where there are multiple drawables, more functional decimation may result. Default False" );
    arguments.getApplicationUsage()->addCommandLineOption( "--numParts <n>", "Controls the geometry building process if user chooses to use a model built in software (see GeometryModifier.h). numParts controls the geometry and can be used to test different aspects of the decimation routines. Default 3. Range 0-4." );

    bool useShortEdge( arguments.find( "--shortedge" ) >= 0 );

    if( arguments.read( "-h" ) || arguments.read( "--help" ) )
    {
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    if (arguments.errors())
    {
        arguments.writeErrorMessages( osg::notify( osg::FATAL ) );
        return 1;
    }


    float retainPercent( .01 );
    std::string str;
    if ( arguments.read( "--retPercent", str ) )
    {
        if( sscanf( str.c_str(), "%f", &retainPercent ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }

    }
    bool ignoreBoundaries = (false);
    if (arguments.read( "--ignoreBoundaries" ))
        ignoreBoundaries = true;

    int minprim(1);
    if (arguments.read("--minPrimitives", str))
    {
        if( sscanf( str.c_str(), "%u", &minprim ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }

    bool attemptMerge = (false);
    if (arguments.read( "--attemptMerge" ))
        attemptMerge = true;

    std::string modelname, namebase;
    namebase = "C:\\OSGDev\\Stable\\data\\";
    bool savefile = true;
    // builds a model for testing if no model file supplied
    osg::Node*  model = osgDB::readNodeFiles( arguments );
    if( !model )
        {
            // built model can consist of one part or two or three. If one part, the front, back and sides can share vertices or have duplicate sets of vertices for the edge triangles. 
            // Merge results may differ depending which is modeled and how the decimation algorithm is implemented and whether or not a geometry merge is attempted prior to decimation.
            int numParts( 3 );
            if ( arguments.read( "--numParts", str ) )
            {
                if( sscanf( str.c_str(), "%u", &numParts ) != 1 )
                {
                    arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
                    return 1;
                }
            }
            osgwTools::DecimationTestModel* builtModel = new osgwTools::DecimationTestModel(numParts);
            if (builtModel)
            {
                model = builtModel->getModel();
                if (savefile)
                {
                    char partNum[64];
                    sprintf(partNum, "%1d_", numParts);
                    namebase.append("DecimationTestModel_");
                    namebase.append(partNum);
                }
            }
            if (!model)
            {
                osg::notify( osg::FATAL ) << "Can't load input file(s)." << std::endl;
                return 1;
            }
           
        }
    else
    {
        namebase.append("LoadedTestModel_");
    }

    osg::ref_ptr<osg::Group> grporig = new osg::Group;
    grporig->addChild(model);
    osg::notify( osg::INFO ) << "createlodvisitor: Loaded model(s)." << std::endl;


    osg::ref_ptr<osg::Group> grpcopy = new osg::Group( *grporig , osg::CopyOp::DEEP_COPY_ALL);
    grpcopy->setName("grpcopy"); // for ease of debugging

    // test LODCreationNodeVisitor
    osgwTools::LODCreationNodeVisitor lodNodeVis;
    // the minimum number of primitives necessary in a Geode in order for an LOD node or nodes to be generated
    lodNodeVis.setTestMinVertices( 50 );
    lodNodeVis.setTestMinPrimitives( minprim );
    // the desired number of remaining triangles in percent (0-1)
    lodNodeVis.setMinRetentionPercent( retainPercent );
    lodNodeVis.setIgnoreBoundaries( ignoreBoundaries );
    lodNodeVis.setAttemptMerge( attemptMerge );
    // if you want to set the list of LOD pixel sizes and maximum feature sizes you must create an LODPairList
    //lodNodeVis.setLODPairs(LODPairList& lodPairList);
    grpcopy->accept( lodNodeVis );
    lodNodeVis.finishProcessingGeodes();

    if( savefile )
    {
        modelname.assign(namebase);
        modelname.append("orig.osg");
        osgDB::writeNodeFile(*grporig, modelname.c_str());
        modelname.assign(namebase);
        modelname.append("reduced.osg");
        osgDB::writeNodeFile(*grpcopy, modelname.c_str());
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
