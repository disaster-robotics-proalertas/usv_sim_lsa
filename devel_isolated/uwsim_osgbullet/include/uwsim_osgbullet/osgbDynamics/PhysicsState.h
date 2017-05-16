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

#ifndef __OSGBDYNAMICS_PHYSICS_STATE_H__
#define __OSGBDYNAMICS_PHYSICS_STATE_H__ 1

#include <osgbDynamics/Export.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgwTools/RefID.h>
#include <osg/Object>
#include <osg/Group>
#include <osgDB/Output>

#include <btBulletDynamicsCommon.h>


namespace osgbDynamics {


/** \class PhysicsData PhysicsState.h <osgbDynamics\PhysicsState.h>
\brief TBD

*/
class OSGBDYNAMICS_EXPORT PhysicsData : public osg::Object
{
public:
    PhysicsData();
    PhysicsData( const PhysicsData& rhs, osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );

    PhysicsData& operator=( const PhysicsData& rhs );

    META_Object(osgbDynamics,PhysicsData);

    /** \brief Obtains rigid body information from the physics simulation.

    If \c _body is non-NULL, this function stores the rigid body's current world
    transform, linear velocity, and angulat velocity in the analogous fields. */
    void loadState();

    /** \brief Restore rigid body information to the physics simultation.

    If \c _body is non-NULL, this function restores the saves \c _bodyWorldTransform,
    |c _angularVelocity, and \c _linearVelocity. */
    void restoreState() const;

    std::string _fileName;
    osg::ref_ptr< osgbDynamics::CreationRecord > _cr;
    btRigidBody* _body;

    // For save / restore use only
    osg::Matrix _osgTransform;
    osg::Matrix _bodyWorldTransform;
    osg::Vec3 _linearVelocity;
    osg::Vec3 _angularVelocity;

    /** \deprecated Existed in \c _version 2 only.
    Superceded by CreationRecord::_friction. */
    double _friction;
    /** \deprecated Existed in \c _version 2 only.
    Superceded by CreationRecord::_restitution. */
    double _restitution;

    unsigned int getVersion() const { return( _version ); }

protected:
    ~PhysicsData();

    unsigned int _version;
};

/** \class PhysicsState PhysicsState.h <osgbDynamics\PhysicsState.h>
\brief TBD

*/
class OSGBDYNAMICS_EXPORT PhysicsState : public osg::Object
{
public:
    PhysicsState();
    PhysicsState( const osgbDynamics::PhysicsState& rhs, osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );
    ~PhysicsState();

    META_Object(osgbDynamics,PhysicsState);

    void addPhysicsData( const osgwTools::RefID* id, PhysicsData* pd );
    void addPhysicsData( const osgwTools::RefID* id, const btRigidBody* body );
    void addPhysicsData( const osgwTools::RefID* id, const osgbDynamics::CreationRecord* cr );
    void addPhysicsData( const osgwTools::RefID* id, const std::string& fileName );
    //void addPhysicsData( const osgwTools::RefID* id, const btConstraint& constraint );

    void addPhysicsData( const std::string& id, PhysicsData* pd );
    void addPhysicsData( const std::string& id, const btRigidBody* body );
    void addPhysicsData( const std::string& id, const osgbDynamics::CreationRecord* cr );
    void addPhysicsData( const std::string& id, const std::string& fileName );
    //void addPhysicsData( const std::string& id, const btConstraint& constraint );

    unsigned int getNumEntries() const;
    const PhysicsData* getPhysicsData( const osgwTools::RefID* id ) const;
    const PhysicsData* getPhysicsData( const std::string& id ) const;
    PhysicsData* getPhysicsData( const std::string& id );

    void removePhysicsData( const osgwTools::RefID* id );
    void removePhysicsData( const std::string& id );

    typedef std::map< std::string, osg::ref_ptr< PhysicsData > > DataMap;
    void exportEntired( osgDB::Output& out ) const;

    /** \brief Load current state from the physics simultation.

    This function calls PhysicsData::loadState() for each element of \c _dataMap.
    Use this function to take a "snapshot" of the current physics simultaion.
    osgBullet uses this functionality to save physics state to disk. */
    void loadState();

    /** \brief Restore physics state to the physics simulation.

    This function calls PhysicsData::restoreState() for each element of \c _dataMap.
    Use this function to reset the physics simulation to a previous state. For example,
    this function enalbes a "reset" button in your application.

    Together, loadState() and restoreState() support the ability for an app to save
    a physics simulation to disk and restore it later in mid-simulation. */
    void restoreState() const;

protected:
    DataMap _dataMap;
};


// osgbDynamics
}

// __OSGBDYNAMICS_PHYSICS_STATE_H__
#endif
