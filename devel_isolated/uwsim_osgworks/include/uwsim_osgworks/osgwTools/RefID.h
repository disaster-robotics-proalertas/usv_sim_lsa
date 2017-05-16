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

#ifndef __REF_ID_H__
#define __REF_ID_H__ 1


#include "osgwTools/Export.h"

#include <osg/Object>

#include <string>


namespace osgwTools
{


/** \brief A reference counter string-based identifier
*/
class OSGWTOOLS_EXPORT RefID : public osg::Object
{
public:
    RefID();
    RefID( const std::string& id );

    RefID( const RefID& rhs, const osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );

    META_Object(osgwTools,RefID);

    bool operator==( const RefID& rhs ) const;
    bool operator<( const RefID& rhs ) const;

    void set( const std::string& id );
    const std::string& str() const;

protected:
    ~RefID();

    std::string _str;
};


// osgwTools
}

// __REF_ID_H__
#endif
