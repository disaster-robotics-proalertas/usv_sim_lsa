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

#include <osgwTools/RemoveLOD.h>
#include <osgwTools/CollapseLOD.h>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgGA/GUIEventHandler>

osgViewer::Viewer viewer;


/* \cond */

class KeyHandler : public osgGA::GUIEventHandler
{
public:
	KeyHandler() {}

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if( ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN )
        {
            if( ea.getKey() == 'c' )
            { // create demo scene
				osg::ref_ptr<osg::Group> topLevelGroup, bottomLevelGroup;
				osg::ref_ptr<osg::LOD> midLOD, bottomLOD;
				osg::ref_ptr<osg::Node> childA, childB, childC,
					childOne, childTwo, childThree;

				// no error checking in this trivial test code
				topLevelGroup = new osg::Group;
				topLevelGroup->setName("topLevelGroup");
				bottomLevelGroup = new osg::Group;
				bottomLevelGroup->setName("bottomLevelGroup");
				midLOD = new osg::LOD;
				midLOD->setName("midLOD");
				bottomLOD = new osg::LOD;
				bottomLOD->setName("bottomLOD");
				childA = new osg::Node;
				childA->setName("childA");
				childB = new osg::Node;
				childB->setName("childB");
				childC = new osg::Node;
				childC->setName("childC");
				childOne   = new osg::Node;
				childOne->setName("childOne");
				childTwo   = new osg::Node;
				childTwo->setName("childTwo");
				childThree = new osg::Node;
				childThree->setName("childThree");

				topLevelGroup->addChild( midLOD.get() );
				midLOD->addChild(childA.get(), 0.0f, 10.0f);
				midLOD->addChild(childB.get(), 10.0f, 100.0f);
				midLOD->addChild(childC.get(), 100.0f, 1000.0f);
				midLOD->addChild(bottomLevelGroup.get(), 1000.0f, 100000.0f);
				bottomLevelGroup->addChild(bottomLOD.get());
				bottomLOD->addChild(childOne.get(), 0.0f, 10.0f);
				bottomLOD->addChild(childTwo.get(), 10.0f, 100.0f);
				bottomLOD->addChild(childThree.get(), 100.0f, 1000.0f);

				// add topLevelGroup to scene root
				viewer.setSceneData(topLevelGroup.get());
				// write out a file showing topology before processing
			    osgDB::writeNodeFile(*topLevelGroup, "lodvisitor_before.dot" );
                return( true );
            }
            if( ea.getKey() == 'd' )
            { // create difficult demo scene for COLLAPSE_TO_PARENT mode
				osg::ref_ptr<osg::Group> bottomLevelGroup;
				osg::ref_ptr<osg::LOD> topLevelLOD, midLOD, bottomLOD;
				osg::ref_ptr<osg::Node> childA, childB, childC,
					childOne, childTwo, childThree, topLevelSecondChild;

				// no error checking in this trivial test code
				topLevelLOD = new osg::LOD;
				topLevelLOD->setName("topLevelLOD");
				bottomLevelGroup = new osg::Group;
				bottomLevelGroup->setName("bottomLevelGroup");
				midLOD = new osg::LOD;
				midLOD->setName("midLOD");
				bottomLOD = new osg::LOD;
				bottomLOD->setName("bottomLOD");
				childA = new osg::Node;
				childA->setName("childA");
				childB = new osg::Node;
				childB->setName("childB");
				childC = new osg::Node;
				childC->setName("childC");
				childOne   = new osg::Node;
				childOne->setName("childOne");
				childTwo   = new osg::Node;
				childTwo->setName("childTwo");
				childThree = new osg::Node;
				childThree->setName("childThree");
				topLevelSecondChild = new osg::Node;
				topLevelSecondChild->setName("topLevelSecondChild");

				topLevelLOD->addChild(midLOD.get(), 0.0f, 10000.0f);
				topLevelLOD->addChild(topLevelSecondChild.get(), 10000.0f, 100000.0f);
				midLOD->addChild(childA.get(), 0.0f, 10.0f);
				midLOD->addChild(childB.get(), 10.0f, 100.0f);
				midLOD->addChild(childC.get(), 100.0f, 1000.0f);
				midLOD->addChild(bottomLevelGroup.get(), 1000.0f, 100000.0f);
				bottomLevelGroup->addChild(bottomLOD.get());
				bottomLOD->addChild(childOne.get(), 0.0f, 10.0f);
				bottomLOD->addChild(childTwo.get(), 10.0f, 100.0f);
				bottomLOD->addChild(childThree.get(), 100.0f, 1000.0f);

				// add topLevelGroup to scene root
				viewer.setSceneData( topLevelLOD.get() );
				// write out a file showing topology before processing
			    osgDB::writeNodeFile(*topLevelLOD, "lodvisitor_complex_before.dot" );
				// write scene file for human examination
			    osgDB::writeNodeFile(*topLevelLOD, "lodvisitor_complex_before.osg" );
                return( true );
            }
            if( ea.getKey() == 'l' )
            {
                if(osgwTools::RemoveLOD *rLOD = new osgwTools::RemoveLOD())
                {
					osg::notify( osg::WARN ) << "Running osgwTools::RemoveLOD" << std::endl;
					rLOD->traverse(*(viewer.getSceneData()));
					// write files for human examination
				    osgDB::writeNodeFile( *(viewer.getSceneData()), "RemoveLODvisitor_after.dot" );
				    osgDB::writeNodeFile( *(viewer.getSceneData()), "RemoveLODvisitor_after.osg" );
                } // if
                return( true );
            }
            if( ea.getKey() == 'j' ) // 'j' calls "traverse()" on top node, affecting only its children
            {
				osg::ref_ptr<osgwTools::HighestLODChildSelectorCallback> selectorCallback;
				selectorCallback = new osgwTools::HighestLODChildSelectorCallback;
				if(osgwTools::CollapseLOD *colLOD = new osgwTools::CollapseLOD(selectorCallback.get(), osgwTools::CollapseLOD::COLLAPSE_TO_PARENT))
                {
					osg::notify( osg::WARN ) << "Running osgwTools::CollapseLOD" << std::endl;
					colLOD->traverse(*(viewer.getSceneData())); // collect the LODs to process
					colLOD->finishProcessingLODs(); // process them once we're out of traverse()
					// write files for human examination
				    osgDB::writeNodeFile(*(viewer.getSceneData()), "CollapseLODvisitor_after.dot" );
				    osgDB::writeNodeFile(*(viewer.getSceneData()), "CollapseLODvisitor_after.osg" );
                } // if
                return( true );
            }
            if( ea.getKey() == 'k' ) // 'k' calls "accept()" on root node, affecting the root node as well (if it's an LOD)
            {
				// Note: Even though we call it as apply(), neither COLLAPSE_TO_PARENT nor COLLAPSE_TO_GROUP mode can
				// alter the root node if it is an LOD itself, because it has no parents for us to mess about with, and
				// we can't rewrite the existing root node in-place. But this is an uncommon situation and easily avoidable.
				osg::ref_ptr<osgwTools::HighestLODChildSelectorCallback> selectorCallback;
				selectorCallback = new osgwTools::HighestLODChildSelectorCallback;
				if(osgwTools::CollapseLOD *colLOD = new osgwTools::CollapseLOD(selectorCallback.get(), osgwTools::CollapseLOD::COLLAPSE_TO_PARENT))
                {
					osg::notify( osg::WARN ) << "Running osgwTools::CollapseLOD" << std::endl;
					viewer.getSceneData()->accept(*colLOD); // collect the LODs to process
					colLOD->finishProcessingLODs(); // process them once we're out of traverse()
					// write files for human examination
				    osgDB::writeNodeFile(*(viewer.getSceneData()), "CollapseLODvisitor_after.dot" );
				    osgDB::writeNodeFile(*(viewer.getSceneData()), "CollapseLODvisitor_after.osg" );
                } // if
                return( true );
            }
        }
        return( false );
    }
};

/* \endcond */



int
main( int argc, char ** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " tests running various utility visitors." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );
    arguments.getApplicationUsage()->addCommandLineOption( "--saveoutput <filename>", "Resaves the modified scene to the file specified." );

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

    std::string outputfilestr;
    arguments.read( "--saveoutput", outputfilestr );

    viewer.setUpViewInWindow( 10, 40, 800, 600 );
    viewer.setSceneData( osgDB::readNodeFiles( arguments ) );
    viewer.realize();

    // setup keyhandler to run RemoveLOD on top of scene when invoked
	KeyHandler* kh = new KeyHandler();
    viewer.addEventHandler( kh );

	int runResult = viewer.run();

	if(!outputfilestr.empty())
	{
		osgDB::writeNodeFile(*viewer.getSceneData(), outputfilestr);
	} // if
	return(runResult);
}
