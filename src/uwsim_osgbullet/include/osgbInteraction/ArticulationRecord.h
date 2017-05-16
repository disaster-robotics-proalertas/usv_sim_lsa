/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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

#ifndef __OSGBINTERACTION_ARTICULATION_RECORD_H__
#define __OSGBINTERACTION_ARTICULATION_RECORD_H__


#include <osgbInteraction/Export.h>
#include <osg/Object>
#include <osg/Vec3d>


namespace osgbInteraction
{


/** \class ArticulationRecord ArticulationRecord.h <osgbInteraction/ArticulationRecord.h>
\brief Support for HandNode articulations.

handpreprocess2 stores transformation axis and pivot point
data in this record, and attaches it as UserData to each
MatrixTransform node. HandNode loads this data and stores it
in ArticulationInfo to control how each articulation
transforms its subgraph.
*/
class OSGBINTERACTION_EXPORT ArticulationRecord : public osg::Object
{
public:
    ArticulationRecord();
    ArticulationRecord( const osg::Vec3d& axis, const osg::Vec3d& pivotPoint );

    ArticulationRecord( const ArticulationRecord& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

    // TBD Currently, dot OSG ArticulationRecord support is in the 
    // osgdb_osgbDynamics plugin. This should really be in a separate
    // osgdb_osgbInteraction plugin.
    META_Object(osgbDynamics,ArticulationRecord);

    osg::Vec3d _axis;
    osg::Vec3d _pivotPoint;


    unsigned int _version;

protected:
    ~ArticulationRecord();
};


// osgbInteraction
}


// __OSGBINTERACTION_ARTICULATION_RECORD_H__
#endif
