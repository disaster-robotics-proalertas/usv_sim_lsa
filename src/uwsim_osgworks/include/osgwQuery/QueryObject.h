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

#ifndef __OSGWQUERY_QUERY_OBJECT_H__
#define __OSGWQUERY_QUERY_OBJECT_H__ 1


#include <osgwQuery/Export.h>
#include <osg/Object>
#include <osg/buffered_value>
#include <osg/GL>

#include <vector>



namespace osgwQuery
{


/** \class QueryObject QueryObject.h <osgwQuery/QueryObject.h>
\brief A conveniece wrapper class for OpenGL query objects.
*/
class OSGWQUERY_EXPORT QueryObject : public osg::Object
{
public:
    QueryObject( unsigned int numIDs=1 );
    QueryObject( const QueryObject& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwQuery,QueryObject);

    GLuint getID( unsigned int contextID, unsigned int queryIDIndex=0 );

    void deleteIDs( unsigned int contextID );

protected:
    ~QueryObject();

    unsigned int _numIDs;
    typedef std::vector< GLuint > QueryIDVector;
    QueryIDVector _ids;
};


// osgwQuery
}

// __OSGWQUERY_QUERY_OBJECT_H__
#endif
