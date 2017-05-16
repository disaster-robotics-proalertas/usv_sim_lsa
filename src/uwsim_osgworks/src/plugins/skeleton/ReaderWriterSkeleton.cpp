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

#include "ReaderWriterSkeleton.h"
#include "osgwTools/RemoveData.h"
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <string>


ReaderWriterSkeleton::ReaderWriterSkeleton()
{
    supportsExtension( "skeleton", "Invokes RemoveData visitor to produce a minimal hierarchy skeleton." );
    supportsExtension( "skel", "Synonym for \"skeleton\"." );

    supportsOption( "[~]ALL", "Data to remove." );
    supportsOption( "[~]DEFAULT", "Data to remove." );
    unsigned int shift( 0 );
    while( true )
    {
        std::string opt( osgwTools::RemoveData::flagsToString( 0x1 << shift++ ) );
        if( !opt.empty() )
            supportsOption( std::string( "[~]" ) + opt, "Data to remove." );
        else
            // Ran out of bits.
            break;
    }
}
ReaderWriterSkeleton::~ReaderWriterSkeleton()
{
}

const char*
ReaderWriterSkeleton::className() const
{
    return "Skeleton pseudo-writer";
}

osgDB::ReaderWriter::ReadResult
ReaderWriterSkeleton::readNode( const std::string& fileName, const Options* options ) const
{
    const std::string ext = osgDB::getFileExtension( fileName );
    if( !acceptsExtension( ext ) )
        return ReadResult::FILE_NOT_HANDLED;

    // Get the file name without the ".skel" or ".skeleton" extension.
    const std::string subName = osgDB::getNameLessExtension( fileName );
    if (subName.empty())
        return ReadResult::FILE_NOT_HANDLED;

    // Get the actual file name extension.
    const std::string subExt = osgDB::getFileExtension( subName );

    // Find the appropriate reader/writer for the actual extension.
    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension( subExt );
    if( rw == NULL )
        return ReadResult::FILE_NOT_HANDLED;

    // Load the specified scene graph from the actual file name.
    osgDB::ReaderWriter::ReadResult rr( rw->readNode( subName, options ) );
    if( ( rr.status() != osgDB::ReaderWriter::ReadResult::FILE_LOADED ) ||
        ( rr.status() != osgDB::ReaderWriter::ReadResult::FILE_LOADED_FROM_CACHE ) )
        return rr;

    // Get RemoveData flags from Options
    unsigned int flags( osgwTools::RemoveData::DEFAULT );
    if( options && !(options->getOptionString().empty()) )
        flags = osgwTools::RemoveData::stringToFlags( options->getOptionString() );

    // Remove data from the scene graph.
    osg::Node* root = rr.getNode();
    osgwTools::RemoveData rd( flags );
    root->accept( rd );

    return( root );
}

osgDB::ReaderWriter::WriteResult
ReaderWriterSkeleton::writeNode( const osg::Node& node, const std::string& fileName, const Options* options ) const
{
    const std::string ext = osgDB::getFileExtension( fileName );
    if( !acceptsExtension( ext ) )
        return WriteResult::FILE_NOT_HANDLED;

    // Get the file name without the ".skel" or ".skeleton" extension.
    const std::string subName = osgDB::getNameLessExtension( fileName );
    if (subName.empty())
        return WriteResult::FILE_NOT_HANDLED;

    // Get the actual file name extension.
    const std::string subExt = osgDB::getFileExtension( subName );

    // Find the appropriate reader/writer for the actual extension.
    osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension( subExt );
    if( rw == NULL )
        return WriteResult::FILE_NOT_HANDLED;

    // Get RemoveData flags from Options
    unsigned int flags( osgwTools::RemoveData::DEFAULT );
    if( options && !(options->getOptionString().empty()) )
        flags = osgwTools::RemoveData::stringToFlags( options->getOptionString() );

    // Remove data from the scene graph.
    osg::Node* nonConstNode = const_cast< osg::Node* >( &node );
    osgwTools::RemoveData rd( flags );
    nonConstNode->accept( rd );

    // Write out the data-stripped scene graph to the actual file name.
    return( rw->writeNode( *nonConstNode, subName, options ) );
}


REGISTER_OSGPLUGIN( skeleton, ReaderWriterSkeleton )
