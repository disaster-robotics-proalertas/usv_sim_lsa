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

#include "osgwTools/RemoveData.h"
#include <osgwTools/StateSetUtils.h>
#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Geometry>
#include <osgwTools/Version.h>

#include <vector>
#include <string>


namespace osgwTools
{


RemoveData::RemoveData( unsigned int flags, const osg::NodeVisitor::TraversalMode travMode )
  : osg::NodeVisitor( travMode ),
    _removalFlags( flags )
{
}
RemoveData::~RemoveData()
{
}


void RemoveData::setRemovalFlags( unsigned int flags )
{
    _removalFlags = flags;
}
unsigned int RemoveData::getRemovalFlags() const
{
    return( _removalFlags );
}

void RemoveData::addRemoveMode( GLenum mode )
{
    _removeModes.push_back( mode );
}
void RemoveData::addRemoveAttribute( osg::StateAttribute::Type attribute )
{
    _removeAttrs.push_back( attribute );
}

std::string
RemoveData::flagsToString( unsigned int flags )
{
    if( flags == ALL )
        return( "ALL" );
    else if( flags == DEFAULT )
        return( "DEFAULT" );
    else
    {
        std::string str( "" );

        if( flags & STATESETS )
            str += "STATESETS ";
        else if( flags & STATESET_TEXTURES )
            str += "STATESET_TEXTURES ";
        else if( flags & EMPTY_STATESETS )
            str += "EMPTY_STATESETS ";
        else if( flags & DRAWABLES )
            str += "DRAWABLES ";
        else if( flags & GEOMETRY_ARRAYS )
            str += "GEOMETRY_ARRAYS ";
        else if( flags & GEOMETRY_PRIMITIVESETS )
            str += "GEOMETRY_PRIMITIVESETS ";
        else if( flags & GEODES )
            str += "GEODES ";
        else if( flags & USERDATA )
            str += "USERDATA ";
        else if( flags & DESCRIPTIONS )
            str += "DESCRIPTIONS ";

        if( !str.empty() )
            // Remove trailing space
            str[ str.length() - 1 ] = '\0';
        return( str );
    }
}
unsigned int
RemoveData::stringToFlags( const std::string& str )
{
    unsigned int flags( 0 );

    if( str.find( "~ALL" ) != std::string::npos )
        flags |= ~ALL;
    else if( str.find( "ALL" ) != std::string::npos )
        flags |= ALL;

    if( str.find( "~DEFAULT" ) != std::string::npos )
        flags |= ~DEFAULT;
    else if( str.find( "DEFAULT" ) != std::string::npos )
        flags |= DEFAULT;

    if( str.find( "~STATESETS" ) != std::string::npos )
        flags |= ~STATESETS;
    else if( str.find( "STATESETS" ) != std::string::npos )
        flags |= STATESETS;

    if( str.find( "~STATESET_TEXTURES" ) != std::string::npos )
        flags |= ~STATESET_TEXTURES;
    else if( str.find( "STATESET_TEXTURES" ) != std::string::npos )
        flags |= STATESET_TEXTURES;

    if( str.find( "~EMPTY_STATESETS" ) != std::string::npos )
        flags |= ~EMPTY_STATESETS;
    else if( str.find( "EMPTY_STATESETS" ) != std::string::npos )
        flags |= EMPTY_STATESETS;

    if( str.find( "~DRAWABLES" ) != std::string::npos )
        flags |= ~DRAWABLES;
    else if( str.find( "DRAWABLES" ) != std::string::npos )
        flags |= DRAWABLES;

    if( str.find( "~GEOMETRY_ARRAYS" ) != std::string::npos )
        flags |= ~GEOMETRY_ARRAYS;
    else if( str.find( "GEOMETRY_ARRAYS" ) != std::string::npos )
        flags |= GEOMETRY_ARRAYS;

    if( str.find( "~GEOMETRY_PRIMITIVESETS" ) != std::string::npos )
        flags |= ~GEOMETRY_PRIMITIVESETS;
    else if( str.find( "GEOMETRY_PRIMITIVESETS" ) != std::string::npos )
        flags |= GEOMETRY_PRIMITIVESETS;

    if( str.find( "~GEODES" ) != std::string::npos )
        flags |= ~GEODES;
    else if( str.find( "GEODES" ) != std::string::npos )
        flags |= GEODES;

    if( str.find( "~USERDATA" ) != std::string::npos )
        flags |= ~USERDATA;
    else if( str.find( "USERDATA" ) != std::string::npos )
        flags |= USERDATA;

    if( str.find( "~DESCRIPTIONS" ) != std::string::npos )
        flags |= ~DESCRIPTIONS;
    else if( str.find( "DESCRIPTIONS" ) != std::string::npos )
        flags |= DESCRIPTIONS;

    return( flags );
}


void
RemoveData::apply( osg::Node& node )
{
    apply( node.getStateSet() );
    if( _removalFlags & STATESETS )
        node.setStateSet( NULL );
    else if( ( _removalFlags & EMPTY_STATESETS ) &&
            ( node.getStateSet() != NULL ) &&
            ( isEmpty( *( node.getStateSet() ) ) ) )
        node.setStateSet( NULL );

    if( _removalFlags & USERDATA )
        node.setUserData( NULL );

    if( _removalFlags & DESCRIPTIONS )
        node.getDescriptions().clear();

    traverse( node );
}

void
RemoveData::apply( osg::Group& node )
{
    apply( node.getStateSet() );
    if( _removalFlags & STATESETS )
        node.setStateSet( NULL );
    else if( ( _removalFlags & EMPTY_STATESETS ) &&
            ( node.getStateSet() != NULL ) &&
            ( isEmpty( *( node.getStateSet() ) ) ) )
        node.setStateSet( NULL );

    if( _removalFlags & USERDATA )
        node.setUserData( NULL );

    if( _removalFlags & DESCRIPTIONS )
        node.getDescriptions().clear();

    if( _removalFlags & GEODES )
    {
        unsigned int idx;
        for( idx=node.getNumChildren(); idx>0; idx-- )
        {
            const unsigned int realIdx( idx-1 );
#if( OSGWORKS_OSG_VERSION >= 20900 )
            osg::Geode* geode = node.getChild( realIdx )->asGeode();
#else
            osg::Geode* geode = dynamic_cast< osg::Geode* >( node.getChild( realIdx ) );
#endif
            if( geode != NULL )
                node.removeChild( realIdx );
        }
    }

    traverse( node );
}

void
RemoveData::apply( osg::Geode& node )
{
    apply( node.getStateSet() );
    if( _removalFlags & STATESETS )
        node.setStateSet( NULL );
    else if( ( _removalFlags & EMPTY_STATESETS ) &&
            ( node.getStateSet() != NULL ) &&
            ( isEmpty( *( node.getStateSet() ) ) ) )
        node.setStateSet( NULL );

    if( _removalFlags & USERDATA )
        node.setUserData( NULL );

    if( _removalFlags & DESCRIPTIONS )
        node.getDescriptions().clear();

    if( _removalFlags & DRAWABLES )
        node.removeDrawables( 0, node.getNumDrawables() );
    else
    {
        unsigned int idx;
        for( idx=0; idx<node.getNumDrawables(); idx++ )
        {
            osg::Drawable* draw( node.getDrawable( idx ) );

            apply( draw->getStateSet() );
            if( _removalFlags & STATESETS )
                draw->setStateSet( NULL );
            else if( ( _removalFlags & EMPTY_STATESETS ) &&
                    ( node.getStateSet() != NULL ) &&
                    ( isEmpty( *( node.getStateSet() ) ) ) )
                draw->setStateSet( NULL );

            if( _removalFlags & USERDATA )
                draw->setUserData( NULL );

            osg::Geometry* geom( draw->asGeometry() );
            if( geom != NULL )
                apply( *geom );
        }
    }
}

void
RemoveData::apply( osg::StateSet* ss )
{
    if( ss == NULL )
        return;

    if( _removalFlags & STATESET_TEXTURES )
    {
        unsigned int idx;
        for( idx=0; idx<16; idx++ )
            ss->setTextureAttribute( idx, NULL );
    }

    ModeVector::const_iterator mvit;
    for( mvit = _removeModes.begin(); mvit != _removeModes.end(); mvit++ )
        ss->removeMode( *mvit );

    AttributeTypeVector::const_iterator avit;
    for( avit = _removeAttrs.begin(); avit != _removeAttrs.end(); avit++ )
        ss->removeAttribute( *avit );

    if( _removalFlags & USERDATA )
        ss->setUserData( NULL );
}

void
RemoveData::apply( osg::Geometry& geom )
{
    if( _removalFlags & GEOMETRY_ARRAYS )
    {
        geom.setVertexArray( NULL );
        geom.setNormalArray( NULL );
        geom.setColorArray( NULL );
        geom.setSecondaryColorArray( NULL );
        geom.setFogCoordArray( NULL );

        // TBD need a better solution for this
        unsigned int idx;
        for( idx=0; idx<16; idx++ )
            geom.setTexCoordArray( idx, NULL );
        for( idx=0; idx<16; idx++ )
            geom.setVertexAttribArray( idx, NULL );
    }

    if( _removalFlags & GEOMETRY_PRIMITIVESETS )
        geom.removePrimitiveSet( 0, geom.getNumPrimitiveSets() );
}


// osgwTools
}
