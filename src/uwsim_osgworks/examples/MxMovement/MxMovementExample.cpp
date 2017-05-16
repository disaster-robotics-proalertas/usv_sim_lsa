// Copyright (c) 2011 Skew Matrix Software LLC. All rights reserved.
// Copyright (c) 2011 Digital Transforms.

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgwMx/MxCore.h>
#include <osgwMx/MxUtils.h>

#ifdef DIRECTINPUT_ENABLED
#  include <osgwMx/MxGamePadDX.h>
#endif

#ifdef VRPN_ENABLED
#  include <osgwMx/MxSpaceBallVRPN.h>
#endif // VRPN_ENABLED


using namespace osgwMx;

int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

	double moveRate( 1. );
    arguments.read( "--rate", moveRate );

    std::string mapFile( "" );
    arguments.read( "--map", mapFile );

    osg::ref_ptr< osg::Node > root = osgDB::readNodeFiles( arguments );
    if( !( root.valid() ) )
    {
        root = osgDB::readNodeFile( "dumptruck.osg" );
        if( !( root.valid() ) )
        {
            osg::notify( osg::FATAL ) << "Can't load input file." << std::endl;
            return( 1 );
        }
    }

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 30, 30, 800, 450 );
    viewer.setSceneData( root.get() );
    viewer.addEventHandler( new osgViewer::StatsHandler );

#ifdef DIRECTINPUT_ENABLED
	// create a game pad input handler and data interpreter to control the view.
    osg::ref_ptr< MxGamePadDX > mxController = new MxGamePadDX;
    mxController->setStickRate( moveRate );

#elif VRPN_ENABLED
	moveRate *= 10.0; // Spaceball, even with normalized range, seems very sluggish
	double rotRate( 30. );
	// create a spaceball input handler and data interpreter to control the view.
	osg::ref_ptr< MxSpaceBallVRPN > mxController = new MxSpaceBallVRPN;
	mxController->setAllRates( moveRate, rotRate );

#endif // VRPN_ENABLED

	// Load the optional functional mapping, if specified.
	if( !( mapFile.empty() ) )
	{
		osgwMx::FunctionalMap* map = dynamic_cast< osgwMx::FunctionalMap* >(
			osgDB::readObjectFile( mapFile ) );
		if( map != NULL )
			mxController->setFunctionalMap( map );
		else
			osg::notify( osg::WARN ) << "Unable to load map file \"" << mapFile << "\"" << std::endl;
	}

	// Set some MxCore defaults:
	osgwMx::MxCore* mxCore = mxController->getMxCore();

	mxCore->setInitialValues( osg::Vec3d( 0., 0., 1. ), osg::Vec3d( 0., 1., 0. ),
        osg::Vec3d( 0., -30., 0. ) );
    mxCore->reset();


    // We'll modify the Camera node, so mark it as DYNAMIC for thread safety.
    viewer.getCamera()->setDataVariance( osg::Object::DYNAMIC );

    // Do not use viewer.run(), which automatically adds a camera manipulator
    // if one doesn't already exist.
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        // DX uses polling; other VR device SDKs might send events.
        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        mxController->poll( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        // Update the current view matrix.
        viewer.getCamera()->setViewMatrix( mxCore->getInverseMatrix() );

        viewer.frame();
    }
}
