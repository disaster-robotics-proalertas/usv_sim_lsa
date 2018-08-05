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

#include <osgbDynamics/PhysicsState.h>
#include <osgbCollision/Utils.h>
#include <osg/Notify>
#include <osg/Object>
#include <osg/Group>
#include <osgDB/Output>
#include <osgwTools/RefID.h>

#include <btBulletDynamicsCommon.h>

namespace osgbDynamics
{


PhysicsData::PhysicsData()
    : 
    _fileName( std::string( "" ) ),
    _cr( NULL ),
    _body( NULL ),
    _friction( 0.1 ),
    _restitution( 1 ),
    _version( 3 )
{
    ;
}
PhysicsData::PhysicsData( const PhysicsData& rhs, osg::CopyOp copyop )
{
    (*this) = rhs;
}
PhysicsData::~PhysicsData()
{
}

PhysicsData&
PhysicsData::operator=( const PhysicsData& rhs )
{
    _version = rhs._version;
    _fileName = rhs._fileName;
    _cr = rhs._cr;
    _body = rhs._body;

    return( *this );
}


void PhysicsData::loadState()
{
    if( _body == NULL )
        return;

    _bodyWorldTransform = osgbCollision::asOsgMatrix( _body->getWorldTransform() );
    _linearVelocity = osgbCollision::asOsgVec3( _body->getLinearVelocity() );
    _angularVelocity = osgbCollision::asOsgVec3( _body->getAngularVelocity() );
}

void PhysicsData::restoreState() const
{
    if( _body == NULL )
        return;

    _body->setWorldTransform( osgbCollision::asBtTransform( _bodyWorldTransform ) );
    _body->setLinearVelocity( osgbCollision::asBtVector3( _linearVelocity ) );
    _body->setAngularVelocity( osgbCollision::asBtVector3( _angularVelocity ) );
}



////////////////////////////////////////////////////////////////////////////////

PhysicsState::PhysicsState()
{
}
PhysicsState::PhysicsState( const osgbDynamics::PhysicsState& rhs, osg::CopyOp copyop )
{
}
PhysicsState::~PhysicsState()
{
}

void PhysicsState::addPhysicsData( const osgwTools::RefID* id, PhysicsData* pd )
{
    addPhysicsData( id->str(), pd );
}
void PhysicsState::addPhysicsData( const osgwTools::RefID* id, const btRigidBody* body )
{
    addPhysicsData( id->str(), body );
}
void PhysicsState::addPhysicsData( const osgwTools::RefID* id, const osgbDynamics::CreationRecord* cr )
{
    addPhysicsData( id->str(), cr );
}
void PhysicsState::addPhysicsData( const osgwTools::RefID* id, const std::string& fileName )
{
    addPhysicsData( id->str(), fileName );
}
//void PhysicsState::addPhysicsData( const osgwTools::RefID* id, const btConstraint& constraint )
//{
//    addPhysicsData( id, constraint );
//}

void PhysicsState::addPhysicsData( const std::string& id, PhysicsData* pd )
{
    if( _dataMap.find( id ) != _dataMap.end() )
        osg::notify( osg::WARN ) << "Overwriting physics data for \"" << id << "\"" << std::endl;

    _dataMap[ id ] = pd;
}
void PhysicsState::addPhysicsData( const std::string& id, const btRigidBody* body )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
    {
        osg::ref_ptr< PhysicsData > pd = new PhysicsData;
        pd->_body = const_cast< btRigidBody* >( body );
        _dataMap[ id ] = pd.get();
    }
    else
    {
        it->second->_body = const_cast< btRigidBody* >( body );
    }
}
void PhysicsState::addPhysicsData( const std::string& id, const osgbDynamics::CreationRecord* cr )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
    {
        osg::ref_ptr< PhysicsData > pd = new PhysicsData;
        pd->_cr = const_cast< CreationRecord* >( cr );
        _dataMap[ id ] = pd.get();
    }
    else
    {
        it->second->_cr = const_cast< CreationRecord* >( cr );
    }
}
void PhysicsState::addPhysicsData( const std::string& id, const std::string& fileName )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
    {
        osg::ref_ptr< PhysicsData > pd = new PhysicsData;
        pd->_fileName = fileName;
        _dataMap[ id ] = pd.get();
    }
    else
    {
        it->second->_fileName = fileName;
    }
}
//void PhysicsState::addPhysicsData( const std::string& id, const btConstraint& constraint )
//{
//}

unsigned int PhysicsState::getNumEntries() const
{
    return( _dataMap.size() );
}
void PhysicsState::exportEntired( osgDB::Output& out ) const
{
    DataMap::const_iterator it;
    for( it = _dataMap.begin(); it != _dataMap.end(); ++it )
    {
        osg::ref_ptr< osgwTools::RefID > rid = new osgwTools::RefID( it->first );
        out.writeObject( *rid );
        out.writeObject( *( it->second ) );
    }
}

const PhysicsData* PhysicsState::getPhysicsData( const osgwTools::RefID* id ) const
{
    return( getPhysicsData( id->str() ) );
}
const PhysicsData* PhysicsState::getPhysicsData( const std::string& id ) const
{
    DataMap::const_iterator it = _dataMap.find( id );
    if( it != _dataMap.end() )
        return( it->second.get() );
    else
        return( NULL );
}
PhysicsData* PhysicsState::getPhysicsData( const std::string& id )
{
    DataMap::const_iterator it = _dataMap.find( id );
    if( it != _dataMap.end() )
        return( it->second.get() );
    else
        return( NULL );
}

void PhysicsState::removePhysicsData( const osgwTools::RefID* id )
{
    removePhysicsData( id->str() );
}
void PhysicsState::removePhysicsData( const std::string& id )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
        osg::notify( osg::WARN ) << "Can't erase non-extant RefID (RefID::operator<<() TBD)." << std::endl;
    else
        _dataMap.erase( it );
}


void PhysicsState::loadState()
{
    DataMap::iterator it;
    for( it = _dataMap.begin(); it != _dataMap.end(); ++it )
        it->second->loadState();
}

void PhysicsState::restoreState() const
{
    DataMap::const_iterator it;
    for( it = _dataMap.begin(); it != _dataMap.end(); ++it )
        it->second->restoreState();
}



// osgbDynamics
}
