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

#include "osgwTools/Version.h"
#include <osgDB/ReadFile>
#include <osg/NodeVisitor>

#include <string>
#include <sstream>


/* \cond */
class ShowNodeNamesVisitor : public osg::NodeVisitor
{
public:
    ShowNodeNamesVisitor( const bool showGeodes )
      : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _level( 0 ),
        _showGeodes( showGeodes )
    {}

    void apply( osg::Node& n )
    {
        display( n.getName(), n.className() );

        _level++;
        traverse( n );
        _level--;
    }

    void apply( osg::Geode& n )
    {
        if( !_showGeodes )
            return;

        // For Geodes, along with className, also include number of Drawables.
        std::ostringstream ostr;
        const unsigned int nd = n.getNumDrawables();
        ostr << n.className() << " with " << nd << " Drawable" << ((nd!=1)?"s":"");
        display( n.getName(), ostr.str() );

        // No traverse for Geodes (they have no children, so it's a no-op).
    }

protected:
    void display( const std::string& name, const std::string& classInfo )
    {
        int idx;
        for( idx=0; idx<_level; idx++ )
            osg::notify( osg::NOTICE ) << "  ";

        osg::notify( osg::NOTICE ) << ( ( !name.empty() ) ? name : "NULL" );
        osg::notify( osg::NOTICE ) << " (" << classInfo << ")" << std::endl;
    }

    int _level;
    bool _showGeodes;
};
/* \endcond */

int
main( int argc,
      char ** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " displays the structure of a scene graph in human-readable form." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "-O <option>", "Pass <option> to import plugin." );
    arguments.getApplicationUsage()->addCommandLineOption( "--nogeodes", "Do not display Geodes in the output." );
    arguments.getApplicationUsage()->addCommandLineOption( "--mask <x>", "Traversal mask. <x> is up to 8 hex digits. Default is \"ffffffff\"." );
    arguments.getApplicationUsage()->addCommandLineOption( "--allNodes", "Traverse all nodes regardless of nodemask." );
    arguments.getApplicationUsage()->addCommandLineOption( "-v/--version", "Display the osgWorks version string." );

    if( arguments.read( "-h" ) || arguments.read( "--help" ) )
    {
        osg::notify( osg::ALWAYS ) << arguments.getApplicationUsage()->getDescription() << std::endl;
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    bool showGeodes( true );
    if( arguments.read( "--nogeodes" ) )
        showGeodes = false;

    unsigned int mask( 0xffffffff );
    int maskPos( 0 );
    if( ( maskPos = arguments.find( "--mask" ) ) > 0 )
    {
        bool failed( true );
        if( arguments.argc() > maskPos+1 )
        {
            std::istringstream istr( arguments[ maskPos+1] );
            istr >> std::hex >> mask;
            failed = istr.fail();
        }
        if( failed )
        {
            osg::notify( osg::WARN ) << "--mask must be followed by a hex number." << std::endl;
            mask = 0xffffffff;
        }
        else
        {
            arguments.remove( maskPos, 2 );
        }
    }

    const bool allNodes( arguments.read( "--allNodes" ) );

    std::string str;
    while( arguments.read( "-O", str ) )
    {
        osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options;
        options->setOptionString( str );
        osgDB::Registry::instance()->setOptions( options );
    }

    if( arguments.read( "-v" ) || arguments.read( "--version" ) )
        osg::notify( osg::ALWAYS ) << osgwTools::getVersionString() << std::endl << std::endl;


    osg::ref_ptr< osg::Node > root = osgDB::readNodeFiles( arguments );
    if( root.get() == NULL )
    {
        osg::notify( osg::FATAL ) << "Unable to load model(S)." << std::endl;
        arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return( 1 );
    }


    ShowNodeNamesVisitor snnv( showGeodes );
    snnv.setTraversalMask( mask );
    if( allNodes )
        snnv.setNodeMaskOverride( 0xffffffff );

    root->accept( snnv );

    return( 0 );
}



/** \page osgwnames The osgwnames Application
osgwnames displays the structure of a scene graph in human-readable form.
It displays a summary of the hierarchy using indentation with each node
represented as a single line of text, containing the class and Node name.
If the Node name is empty, \e NULL displays in its place.

\section su Simple Usage
Try running osgwnames on \c dectest20.ive, one of the osgWorks data files:

\code
C:\Projects>osgwnames dectest20.ive
world (Group)
  door_assembly.asm.2 (Group)
    NULL (MatrixTransform)
      KNOB.PRT (Geode with 1 Drawable)
    NULL (MatrixTransform)
      KNOB.PRT #2 (Geode with 1 Drawable)
    NULL (MatrixTransform)
      DOOR.PRT (Geode with 1 Drawable)
    POLE.PRT (Geode with 1 Drawable)\endcode
\endcode

The output shows the scene graph hierarchical structure:
\li The scene graph has a top-level Group node named \e world, has a single Group child named
\e door_assembly.asm.2.
\li \e door_assembly.asm.2 has four children, three of which are unnamed MatrixTransform objects
(the name is displayed as \e NULL), and the fourth child is a Geode named \e POLE.PRT.
\li Each of the MatrixTransform objects have Geode children, with names \e KNOB.PRT,
<i>KNOB.PRT #2</i>, and \e DOOR.PRT.

To produce more readable output, you can prevent the display of Geodes with the
\b --nogeodes option:

\code
C:\Projects>osgwnames --nogeodes dectest20.ive
world (Group)
  door_assembly.asm.2 (Group)
    NULL (MatrixTransform)
    NULL (MatrixTransform)
    NULL (MatrixTransform)
\endcode

\section clp Command Line Parameters
<table border="0">
  <tr>
    <td><b>--nogeodes</b></td>
    <td>Do not display Geodes in the output.</td>
  </tr>
  <tr>
    <td><b>--allNodes</b></td>
    <td>Traverse all nodes regardless of node mask.</td>
  </tr>
  <tr>
    <td><b>--mask <x></b></td>
    <td>Specify the scene graph traversal mask. Replace <x> with up to 8 hex digits. Default is ffffffff.</td>
  </tr>
  <tr>
    <td><b>-O <option></b></td>
    <td>Pass \b <option> to the OSG import plugin.</td>
  </tr>
  <tr>
    <td><b>-v/--version</b></td>
    <td>Display the osgWorks version string.</td>
  </tr>
</table>

*/
