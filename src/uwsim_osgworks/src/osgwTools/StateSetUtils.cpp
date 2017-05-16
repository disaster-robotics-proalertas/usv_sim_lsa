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

#include <osgwTools/StateSetUtils.h>
#include <osg/StateSet>
#include <osg/Node>


namespace osgwTools
{


bool isEmpty( const osg::StateSet& stateSet )
{
    bool empty( true );
    if( stateSet.getDataVariance() != osg::Object::STATIC )
        empty = false;
    else if( !stateSet.getModeList().empty() )
        empty = false;
    else if( !stateSet.getAttributeList().empty() )
        empty = false;
    else if( !stateSet.getTextureModeList().empty() )
        empty = false;
    else if( !stateSet.getTextureAttributeList().empty() )
        empty = false;
    else if( !stateSet.getUniformList().empty() )
        empty = false;
    else if( stateSet.getRenderBinMode() != osg::StateSet::INHERIT_RENDERBIN_DETAILS )
        empty = false;
    else if( !stateSet.getNestRenderBins() )
        empty = false;

    return( empty );
}

osg::StateSet* accumulateStateSets( const osg::NodePath& nodePath )
{
    osg::ref_ptr< osg::StateSet > stateSet = new osg::StateSet();

    osg::NodePath::const_iterator it;
    for( it = nodePath.begin(); it != nodePath.end(); it++ )
    {
        osg::Node* node = *it;
        osg::StateSet* nextStateSet = node->getStateSet();
        if( nextStateSet != NULL )
            stateSet->merge( *nextStateSet );
    }

    return( stateSet.release() );
}


// osgwTools
}
