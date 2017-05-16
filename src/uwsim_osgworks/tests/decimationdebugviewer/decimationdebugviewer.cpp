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

#include <osg/NodeVisitor>
#include <osg/Version>
#include <osgGA/StateSetManipulator>

#include <osgViewer/ViewerEventHandlers>

// class to handle keyboard events
class KeyHandler : public osgGA::GUIEventHandler 
{
public: 

    KeyHandler(osg::Group* sourceNode, osgwTools::GeometryOperation* reducer, bool attemptMerge):
        _mx(0.0),_my(0.0),
        _usePolytopeIntersector(false),
        _useWindowCoordinates(false),
        _attemptMerge(attemptMerge),
        _displayGroup(DISPLAY_LIVE),
        _copyNode(0),
        _stepbackNode(0),
        _liveNode(sourceNode),
        _reducer(reducer) 
    {
        _rootNode = new osg::Group;
        _rootNode->addChild(sourceNode);

    }

    ~KeyHandler() {}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        if (!viewer) return false;

        switch(ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYUP):
            {
                bool handled = 0;
                if (ea.getKey()=='1')
                {
                    handled = decimateModel(1);
                }                
                else if (ea.getKey()=='2')
                {
                    handled = decimateModel(2);
                }                
                else if (ea.getKey()=='5')
                {
                    handled = decimateModel(5);
                }                
                else if (ea.getKey()=='d')
                {
                    handled = decimateModel(10);
                }                
                else if (ea.getKey()=='c')
                {
                    handled = decimateModel(100);
                }                
                else if (ea.getKey()=='k')
                {
                    handled = decimateModel(1000);
                }                
                else if (ea.getKey()=='y')
                {
                    handled = swapModels();
                }
                else if (ea.getKey()=='u')
                {
                    handled = stepbackModels();
                }
                else if (ea.getKey()=='r')
                {
                    osg::notify(osg::NOTICE)<<"Reloading original model"<<std::endl;
                    handled = reloadModel();
                }
                if (handled)
                {
                    if (_displayGroup == DISPLAY_LIVE)
                        setSceneData(viewer, _liveNode.get());
                    else if (_displayGroup == DISPLAY_STEPBACK)
                        setSceneData(viewer, _stepbackNode.get());
                    else
                        setSceneData(viewer, _copyNode.get());
               }
                return false;
            }

            default:
                return false;
        }
    }


    bool decimateModel(unsigned int steps)
    {
        bool handled(0);
        if (_liveNode.valid())
        {
            _reducer->setMaxSteps(steps);
            if (! _copyNode.valid())
            {
                // make a copy of the model
                _copyNode = new osg::Group( *_liveNode, osg::CopyOp::DEEP_COPY_ALL);
                _geoMod = new osgwTools::GeometryModifier(_reducer.get());
                _geoMod->setDrawableMerge(_attemptMerge);
            }
            if (_displayGroup == DISPLAY_ORIG || _displayGroup == DISPLAY_LIVE || ! _stepbackNode.valid())
            {
                if (_stepbackNode.valid())
                        _stepbackNode.release();
                // make a copy of the model to use for stepping back one decimation step
                _stepbackNode = new osg::Group( *_liveNode, osg::CopyOp::DEEP_COPY_ALL);
            }
            else
            {
                _liveNode.release();
                // start with the previous state
                _liveNode = new osg::Group( *_stepbackNode, osg::CopyOp::DEEP_COPY_ALL);
            }
            // decimate the model
            _liveNode->accept(*_geoMod);
            _geoMod->displayStatistics( osg::notify( osg::ALWAYS ) );
            handled = true;
            _displayGroup = DISPLAY_LIVE;
        }
        return (handled);
    }

    bool reloadModel(void)
    {
        bool handled(0);
        osg::Group* temp1 = _copyNode.get();
        osg::Group* temp2 = _liveNode.get();
        if (_copyNode.valid())
        {
            // restore copy of the model, remove the modified copy
            _liveNode = _copyNode;
            _copyNode.release();
            _geoMod.release();
            handled = true;
            _displayGroup = DISPLAY_LIVE;
       }
        return (handled);
    }

    bool swapModels(void)
    {
        bool handled(0);
        if (_copyNode.valid() && _liveNode.valid())
        {
            handled = true;
            if (_displayGroup != DISPLAY_ORIG)
                _displayGroup = DISPLAY_ORIG;
            else
                _displayGroup = DISPLAY_LIVE;
        }
        return (handled);
    }

    bool stepbackModels(void)
    {
        bool handled(0);
        if (_stepbackNode.valid() && _liveNode.valid())
        {
            handled = true;
            if (_displayGroup != DISPLAY_STEPBACK)
                _displayGroup = DISPLAY_STEPBACK;
            else
                _displayGroup = DISPLAY_LIVE;
        }
        return (handled);
    }

    void setSceneData(osgViewer::Viewer* viewer, osg::Group* setGroupActive)
    {
        if (_rootNode->getChild(0) != setGroupActive)
        {
            _rootNode->removeChild(0, 1);
            _rootNode->addChild(setGroupActive);
        }
    }

    osg::Group* getRootNode(void)   {return(_rootNode.get());}

protected:

    enum DisplayGroup
    {
        DISPLAY_ORIG,
        DISPLAY_LIVE,
        DISPLAY_STEPBACK
    };

    float _mx,_my;
    bool _usePolytopeIntersector;
    bool _useWindowCoordinates;
    bool _attemptMerge;
    osg::ref_ptr<osg::Group> _liveNode; 
    osg::ref_ptr<osg::Group> _copyNode; 
    osg::ref_ptr<osg::Group> _stepbackNode; 
    osg::ref_ptr<osg::Group> _rootNode; 
    DisplayGroup _displayGroup; 
    osg::ref_ptr<osgwTools::GeometryOperation> _reducer;
    osg::ref_ptr<osgwTools::GeometryModifier> _geoMod;
};


int main( int argc,
          char* argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "--reducer", "Use ReducerOp." );
    arguments.getApplicationUsage()->addCommandLineOption( "--shortedge", "Use ShortEdgeOp." );
    arguments.getApplicationUsage()->addCommandLineOption( "--decPercent <n>", "Use DecimatorOp (also valid parameter for ShortEdgeOp). <n> is the target percentage of triangles to remain, in the range 0.0 to 1.0. Default 0.6" );
    arguments.getApplicationUsage()->addCommandLineOption( "--decMaxError <n>", "Specifies the Decimator maximum error tolerance. Geometry exceeding this tolerance is not reduced. <n> is in the range 0.0 to FLT_MAX. Default FLT_MAX" );
    arguments.getApplicationUsage()->addCommandLineOption( "--respectBoundaries", "Will not decimate boundary polygons, will not decimate fully but may fix some mesh errors. Default False" );
    arguments.getApplicationUsage()->addCommandLineOption( "--minPrimitives <n>", "Sets the minimum primitives a geometry must have to start Decimation. Default 1." );
    arguments.getApplicationUsage()->addCommandLineOption( "--maxFeature <n>", "Specifies the ShortEdgeOp largest feature size to be removed, measured in model units. Can be combined with decPercent to limit the decimation using ShortEdgeOp. Default 0.1" );
    arguments.getApplicationUsage()->addCommandLineOption( "--attemptMerge <n>", "Attempts to merge drawables within the model prior to any geometry reduction using a MergeGeometryVisitor. In cases where there are multiple drawables, more functional decimation may result. Default False" );
    arguments.getApplicationUsage()->addCommandLineOption( "--numParts <n>", "Controls the geometry building process if user chooses to use a model built in software (see GeometryModifier.h). numParts controls the geometry and can be used to test different aspects of the decimation routines. Default 3. Range 0-4." );
    arguments.getApplicationUsage()->addCommandLineOption( "--attemptMerge <n>", "Attempt to merge geometry drawables into one using Optimizer::MergeGeometryVisitor before using specified geometry reduction operator." );


    arguments.getApplicationUsage()->addKeyboardMouseBinding("1","Decimate one edge.");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("2","Decimate two edges.");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("5","Decimate five edges.");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("d","Decimate 10 edges.");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("c","Decimate 100 edges.");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("k","Decimate 1000 edges.");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("r","Restore original model, discard decimation.");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("u","Undo/Restore last decimation step(s).");
    arguments.getApplicationUsage()->addKeyboardMouseBinding("y","Swap display between decimated model and original.");



    bool useReducer( arguments.find( "--reducer" ) >= 0 );
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


    float decimatorPercent( .6 );
    std::string str;
    if ( arguments.read( "--decPercent", str ) )
    {
        if( sscanf( str.c_str(), "%f", &decimatorPercent ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }

    }
    float decimatorMaxError( FLT_MAX );
    if ( arguments.read( "--decMaxError", str ) )
    {
        if( sscanf( str.c_str(), "%f", &decimatorMaxError ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    bool decimatorIgnoreBoundaries = (true);
    if (arguments.read( "--respectBoundaries" ))
        decimatorIgnoreBoundaries = false;

    int minprim(1);
    if (arguments.read("--minPrimitives", str))
    {
        if( sscanf( str.c_str(), "%u", &minprim ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    if (decimatorPercent < 1.f )
        osg::notify( osg::INFO ) << "DecimatorOp: " << decimatorPercent << ", " << decimatorMaxError << std::endl;

    float shortEdgeFeature( .1 );
    if ( arguments.read( "--maxFeature", str ) )
    {
        if( sscanf( str.c_str(), "%f", &shortEdgeFeature ) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
            return 1;
        }
    }
    bool attemptMerge = (false);
    if (arguments.read( "--attemptMerge" ))
        attemptMerge = true;

    bool useAgg = false;
    unsigned int aggMaxVerticies( 0 );
    osg::Vec3 aggMinCellSize( 0., 0., 0. );
    if(arguments.read( "--aggMaxVerts", str) )
    {
        if( sscanf( str.c_str(), "%u", &aggMaxVerticies) != 1 )
        {
            arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ) );
        }
        useAgg = true;
        if ( arguments.read( "--aggMinCellSize", str ) )
        {
            char comma;
            std::istringstream oStr( str );
            oStr >> aggMinCellSize[ 0 ] >> comma >>
                aggMinCellSize[ 1 ] >> comma >>
                aggMinCellSize[ 2 ];
        }
    }
    
    std::string modelname, namebase;
    namebase = "C:\\OSGDev\\Stable\\data\\";
    bool savefile = false;
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
    osg::notify( osg::INFO ) << "decimationdebugviewer: Loaded model(s)." << std::endl;


    osg::ref_ptr<osg::Group> grpcopy = new osg::Group( *grporig , osg::CopyOp::DEEP_COPY_ALL);

    osg::ref_ptr<osgwTools::GeometryOperation> reducer;
    if(!useAgg && !useReducer && !useShortEdge)
    {
        osgwTools::DecimatorOp* decimate = new osgwTools::DecimatorOp;
        decimate->setSampleRatio(decimatorPercent);
        decimate->setMaximumError(decimatorMaxError);
        decimate->setIgnoreBoundaries(decimatorIgnoreBoundaries);
        decimate->setMinPrimitives(minprim);
        reducer = decimate;
    }
    else if( useReducer )
    {
        osgwTools::ReducerOp* redOp = new osgwTools::ReducerOp;
        reducer = redOp;
    }
    else if( useShortEdge )
    {
        osgwTools::ShortEdgeOp* seOp = new osgwTools::ShortEdgeOp(decimatorPercent, shortEdgeFeature, 1);
        seOp->setIgnoreBoundaries(decimatorIgnoreBoundaries);
        seOp->setMinPrimitives(minprim);
        reducer = seOp;
    }

    // Viewer setup.
    // create the window to draw to.
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 200;
    traits->y = 200;
    traits->width = 800;
    traits->height = 600;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
    if (!gw)
    {
        osg::notify(osg::NOTICE)<<"Error: unable to create graphics window."<<std::endl;
        return 1;
    }


    osgViewer::Viewer viewer;
    viewer.getCamera()->setGraphicsContext(gc.get());
    viewer.getCamera()->setViewport(0,0,800,600);

    // add the pick handler
    osg::ref_ptr<KeyHandler> keyHandler = new KeyHandler(grpcopy.get(), reducer.get(), attemptMerge);
    osg::Group* sceneRoot = keyHandler->getRootNode();
    viewer.setSceneData(sceneRoot);

    // create a tracball manipulator to move the camera around in response to keyboard/mouse events
    viewer.setCameraManipulator( new osgGA::TrackballManipulator );

    osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator(viewer.getCamera()->getStateSet());
    viewer.addEventHandler(statesetManipulator.get());
    viewer.addEventHandler(keyHandler.get());

    osg::ref_ptr<osgViewer::HelpHandler> helpHandler = new osgViewer::HelpHandler(arguments.getApplicationUsage());
    viewer.addEventHandler( helpHandler.get() );
    osg::ref_ptr<osgViewer::StatsHandler> statsHandler = new osgViewer::StatsHandler;
    //statsHandler->setKeyEventTogglesOnScreenStats('t');
    viewer.addEventHandler( statsHandler.get() );

    viewer.realize();

    // main loop (note, window toolkits which take control over the main loop will require a window redraw callback containing the code below.)
    while(!viewer.done())
    {
        viewer.frame();
    }

    return 0;
}

