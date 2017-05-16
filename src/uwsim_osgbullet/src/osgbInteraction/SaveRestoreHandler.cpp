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

#include <osgbInteraction/SaveRestoreHandler.h>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgGA/GUIEventHandler>
#include <osgbDynamics/PhysicsState.h>
#include <osgbDynamics/PhysicsThread.h>
#include <osgbInteraction/LaunchHandler.h>

#include <btBulletDynamicsCommon.h>

#include <sstream>


namespace osgbInteraction
{


SaveRestoreHandler::SaveRestoreHandler()
  : _state( new osgbDynamics::PhysicsState ),
    _fileName( "osgbullet-save.sgb" ),
    _lh( NULL ),
    _pt( NULL )
{
}
SaveRestoreHandler::~SaveRestoreHandler()
{
}

bool SaveRestoreHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    if( ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN )
        return( false );
    const bool ctrl( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) != 0 );

    if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Insert )
    {
        if( _pt != NULL )
            _pt->pause( true );
        capture();
        if( _pt != NULL )
            _pt->pause( false );
        return( true );
    }
    else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Delete )
    {
        if( _pt != NULL )
            _pt->pause( true );
        reset();
        if( _pt != NULL )
            _pt->pause( false );
        return( true );
    }
    else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_F1 )
    {
        save();
        return( true );
    }
    else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_F2 )
    {
        if( _pt != NULL )
            _pt->pause( true );
        capture();
        if( _pt != NULL )
            _pt->pause( false );
        save();
        return( true );
    }
    return( false );
}

void SaveRestoreHandler::setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt )
{
    _pt = pt;
}

void SaveRestoreHandler::add( const std::string& id, btRigidBody* rb )
{
    _state->addPhysicsData( id, rb );
}
void SaveRestoreHandler::add( const std::string& id, osgbDynamics::CreationRecord* cr )
{
    _state->addPhysicsData( id, cr );
}

void SaveRestoreHandler::add( btDynamicsWorld* dw )
{
    btCollisionObjectArray& coa = dw->getCollisionObjectArray();
    int idx;
    for( idx=0; idx < dw->getNumCollisionObjects(); idx++ )
    {
        btRigidBody* rb = static_cast< btRigidBody* >( coa[ idx ] );
        if( rb != NULL )
        {
            std::ostringstream ostr;
            ostr << idx;
            _state->addPhysicsData( ostr.str(), rb );
        }
    }
}

osgbDynamics::PhysicsData* SaveRestoreHandler::getPhysicsData( const std::string& id )
{
    return( _state->getPhysicsData( id ) );
}


void SaveRestoreHandler::capture()
{
    _state->loadState();
}

void SaveRestoreHandler::reset()
{
    _state->restoreState();

    if( _lh != NULL )
        _lh->reset();
}

void SaveRestoreHandler::setSaveRestoreFileName( const std::string& fileName )
{
    if( osgDB::getLowerCaseFileExtension( fileName ) != std::string( "sgb" ) )
    {
        osg::notify( osg::WARN ) << "SaveRestoreHandler::setSaveRestoreFileName(): Invalid file extension." << std::endl;
        osg::notify( osg::WARN ) << "\tFile name: \"" << fileName << "\" must have extension .sgb" << std::endl;
        return;
    }

    _fileName = fileName;
}
std::string SaveRestoreHandler::getSaveRestoreFileName() const
{
    return( _fileName );
}

void SaveRestoreHandler::save( const std::string& fileName )
{
    std::string fName( fileName );
    if( fName.empty() )
        fName = _fileName;

    osgDB::writeObjectFile( *_state, fName );
}

void SaveRestoreHandler::restore( const std::string& fileName )
{
    osg::notify( osg::WARN ) << "SaveRestoreHandler::restore() not currently implemented." << std::endl;
    return;

    std::string fName( fileName );
    if( fName.empty() )
        fName = _fileName;

    osg::Object* state = osgDB::readObjectFile( fileName );
    _state = dynamic_cast< osgbDynamics::PhysicsState* >( state );
    if( state == NULL )
        osg::notify( osg::WARN ) << "SaveRestoreHandler::restore(): Unable to read data from \"" << fName << "\"." << std::endl;
}


// osgbInteraction
}
