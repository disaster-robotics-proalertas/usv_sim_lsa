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

#include <osgwTools/CallbackSupport.h>
#include <osg/Camera>

#include <vector>


namespace osgwTools
{


CompositeDrawCallback::CompositeDrawCallback()
{
}
CompositeDrawCallback::CompositeDrawCallback( const CompositeDrawCallback& rhs, const osg::CopyOp& copyop )
  : osg::Camera::DrawCallback( rhs, copyop ),
    _dcl( rhs._dcl )
{
}
CompositeDrawCallback::~CompositeDrawCallback()
{
}


void
CompositeDrawCallback::operator()( osg::RenderInfo& renderInfo ) const
{
    DrawCallbackList::const_iterator it;
    for( it=_dcl.begin(); it!= _dcl.end(); it++ )
    {
        osg::Camera::DrawCallback& cb = *(*it);
        cb( renderInfo );
    }
}

CompositeDrawCallback::DrawCallbackList&
CompositeDrawCallback::getDrawCallbackList()
{
    return( _dcl );
}
const CompositeDrawCallback::DrawCallbackList&
CompositeDrawCallback::getDrawCallbackList() const
{
    return( _dcl );
}


// namespace osgwTools
}
