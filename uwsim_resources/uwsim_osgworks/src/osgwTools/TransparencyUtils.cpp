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

#include <osgwTools/TransparencyUtils.h>
#include <osg/BlendColor>
#include <osg/BlendFunc>
#include <osg/NodeVisitor>
#include <osg/Geode>

#include <string>


namespace osgwTools
{


bool isTransparent( const osg::StateSet* stateSet )
{
    if( stateSet == NULL )
        return( false );

    const bool hasBlendColor = ( stateSet->getAttribute( osg::StateAttribute::BLENDCOLOR ) != NULL );
    const bool hasBlendFunc = ( stateSet->getAttribute( osg::StateAttribute::BLENDFUNC ) != NULL );
    const bool blendEnabled = ( ( stateSet->getMode( GL_BLEND ) & osg::StateAttribute::ON ) != 0 );
    const bool hasRenderingHint = ( stateSet->getRenderingHint() == osg::StateSet::TRANSPARENT_BIN );

    return( hasBlendColor && hasBlendFunc && blendEnabled && hasRenderingHint );
}




ProtectTransparencyVisitor::ProtectTransparencyVisitor()
  : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
{
}

void ProtectTransparencyVisitor::apply( osg::Node& node )
{
    protectTransparent( node.getStateSet() );
    traverse( node );
}

void ProtectTransparencyVisitor::apply( osg::Geode& geode )
{
    protectTransparent( geode.getStateSet() );

    unsigned int idx;
    for( idx=0; idx<geode.getNumDrawables(); idx++ )
    {
        protectTransparent( geode.getDrawable( idx )->getStateSet() );
    }

    traverse( geode );
}

void ProtectTransparencyVisitor::protectTransparent( osg::StateSet* stateSet ) const
{
    if( stateSet == NULL )
    {
        return;
    }

    if( isTransparentInternal( stateSet ) )
    {
        stateSet->setMode( GL_BLEND, stateSet->getMode( GL_BLEND ) | osg::StateAttribute::PROTECTED );

        osg::BlendColor* bc = dynamic_cast< osg::BlendColor* >( stateSet->getAttribute( osg::StateAttribute::BLENDCOLOR ) );
        if( bc != NULL )
            stateSet->setAttributeAndModes( bc, stateSet->getMode( GL_BLEND ) | osg::StateAttribute::PROTECTED );

        osg::BlendFunc* bf = dynamic_cast< osg::BlendFunc* >( stateSet->getAttribute( osg::StateAttribute::BLENDFUNC ) );
        if( bf != NULL )
            stateSet->setAttributeAndModes( bf, stateSet->getMode( GL_BLEND ) | osg::StateAttribute::PROTECTED );
    }
}

bool ProtectTransparencyVisitor::isTransparentInternal( const osg::StateSet* stateSet ) const
{
    bool blendEnabled = ( ( stateSet->getMode( GL_BLEND ) & osg::StateAttribute::ON ) != 0 );
    bool hasTranslucentTexture = false;
    bool hasBlendFunc = ( stateSet->getAttribute( osg::StateAttribute::BLENDFUNC ) != 0 );
    bool hasTransparentRenderingHint = stateSet->getRenderingHint() == osg::StateSet::TRANSPARENT_BIN;
    bool hasDepthSortBin = ( stateSet->getRenderBinMode() == osg::StateSet::USE_RENDERBIN_DETAILS ) ? 
        ( stateSet->getBinName()=="DepthSortedBin" ) : false;

    // search for the existence of any texture object attributes
    for( unsigned int i=0;i<stateSet->getTextureAttributeList().size();++i )
    {
        const osg::Texture* texture = dynamic_cast< const osg::Texture* >(
            stateSet->getTextureAttribute( i, osg::StateAttribute::TEXTURE ) );
        if( texture != NULL )
        {
            for( unsigned int im=0;im<texture->getNumImages();++im )
            {
                const osg::Image* image = texture->getImage(im);
                if (image && image->isImageTranslucent())
                {
                    hasTranslucentTexture = true;   
                }
            }
        }
    }
    
    return( blendEnabled &&
        ( hasTranslucentTexture || hasBlendFunc || hasTransparentRenderingHint || hasDepthSortBin ) );
}




RestoreOpacityVisitor::RestoreOpacityVisitor()
  : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
{
}

void RestoreOpacityVisitor::apply( osg::Node& node )
{
    transparentDisable( &node );

    traverse( node );
}

void RestoreOpacityVisitor::apply( osg::Geode& geode )

{
    transparentDisable( &geode );

    unsigned int idx;
    for( idx=0; idx<geode.getNumDrawables(); idx++ )
    {
        transparentDisable( geode.getDrawable( idx ) );
    }

    traverse( geode );
}


// osgwTools
}
