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

#ifndef __OSGWTOOLS_STATE_SET_UTILS__
#define __OSGWTOOLS_STATE_SET_UTILS__ 1


#include "osgwTools/Export.h"
#include <osg/StateSet>
#include <osg/Node>


namespace osgwTools
{


/** \defgroup StateSetUtils Utilities for StateSets

*/
/*@{*/


/** Return true if the specified \c StateSet is empty, and false otherwise.
*/
OSGWTOOLS_EXPORT bool isEmpty( const osg::StateSet& stateSet );

/** Return a \c StateSet that represents the accumulation of all \c StateSets
in the specified \c NodePath.

Note: The function returns the \c StateSet using ref_ptr::release(). The
calling code must take responsibility for handling reference counting
(but storing the StateSet in a ref_ptr, for example).
*/
OSGWTOOLS_EXPORT osg::StateSet* accumulateStateSets( const osg::NodePath& nodePath );


/*@}*/


// namespace osgwTools
}

// __OSGWTOOLS_STATE_SET_UTILS__
#endif
