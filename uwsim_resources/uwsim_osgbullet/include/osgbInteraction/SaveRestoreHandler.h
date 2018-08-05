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

#ifndef __OSGBINTERACTION_SAVE_RESTORE_HANDLER_H__
#define __OSGBINTERACTION_SAVE_RESTORE_HANDLER_H__ 1


#include <osgbInteraction/Export.h>
#include <osgGA/GUIEventHandler>
#include <osgbDynamics/PhysicsState.h>
#include <btBulletDynamicsCommon.h>


// Forward
namespace osgbDynamics {
    class PhysicsThread;
}


namespace osgbInteraction
{


// Forward:
class LaunchHandler;


/** \class SaveRestoreHandler SaveRestoreHandler.h <osgbInteraction\SaveRestoreHandler.h>
\brief An event handler to support physics save and restore.

TBD full descrip.
*/
class OSGBINTERACTION_EXPORT SaveRestoreHandler : public osgGA::GUIEventHandler
{
public:
    SaveRestoreHandler();

    /** \brief Handle events

    Controls:
    \li Insert Capture the current physics state.
    \li Delete Reset the last captured physics state.
    \li F1 Save last captured state to disk.
    \li F2 Capture and save current physics state to disk.
    */
    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );

    /** \brief Support for running the Bullet physics simultation in a separate thread.

    Call this function to specify the osgbDynamics::PhysicsThread. SaveRestoreHandler pauses
    and unpauses the thread during accesses to the dynamics world. */
    void setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt );

    /** \brief Add a rigid body for save / restore management.

    SaveRestoreHandler only captures and modifies the state of rigid bodies that have
    been explicitly added.
    \param id A unique string identifier, used for associating rigid bodies with
    scene graph locations during a restore from disk.
    \param rb Rigid body to manage. */
    void add( const std::string& id, btRigidBody* rb );

    /** \brief Add a CreationRecord for saving to disk.

    To support later restore from disk, use this function to associate a CreationRecord
    with a particular ID. For an example of how to use this information to restore from disk,
    see the saverestore example.
    \param id A unique string identifier, used for associating rigid bodies with
    scene graph locations during a restore from disk.
    \param cr CreationRecord rigid body creation information. */
    void add( const std::string& id, osgbDynamics::CreationRecord* cr );

    /** \brief Add all rigid bodies to save / restore management.

    This convenience routine loads all rigid bodies in the Bullet dynamics world.
    It's not particularly useful if you hope to save to disk and later restore, as
    the string identifiers are sutomatically generated, rather than uniquely
    identifying a scene graph location. */
    void add( btDynamicsWorld* dw );

    /** \brief Get PhysicsData for the specified id.
    */
    osgbDynamics::PhysicsData* getPhysicsData( const std::string& id );

    /** \brief Capture the current physics state for managed rigid bodies.
    */
    void capture();

    /** \brief Reset the last captured physics state for managed rigid bodies.
    */
    void reset();

    /** \brief Access the default file name used in save and restore operations.

    If save() or restore() and called explicitly with no parameter, or are
    called from handle() in restore to 's' or 'r' key activation, SaveRestoreHandler
    saves to (or loads from) a default file name. The initial value for the
    file name is "osgbullet-save.sgb". Override that default with the parameter
    to setSaveRestoreFileName().

    Note: The file name extension must be ".sgb".
    */
    void setSaveRestoreFileName( const std::string& fileName );
    std::string getSaveRestoreFileName() const;

    /** \brief Save to disk.
    */
    void save( const std::string& fileName=std::string( "" ) );

    /** \brief Restore from disk. Not currently implemented.

    Do not use this function. See saverestore for an example of restoreing
    from disk. */
    void restore( const std::string& fileName=std::string( "" ) );

    /** \brief Specify a LaunchHandler to reset during the reset() call.

    By default, no LaunchHandler is specified. Specify one with this function
    call, and reset() will call LaunchHandler::reset() to erase all launched
    models. */
    void setLaunchHandler( osgbInteraction::LaunchHandler* lh ) { _lh = lh; }

protected:
    virtual ~SaveRestoreHandler();

    osg::ref_ptr< osgbDynamics::PhysicsState > _state;
    std::string _fileName;

    osgbInteraction::LaunchHandler* _lh;

    osgbDynamics::PhysicsThread* _pt;
};


// osgbInteraction
}


// __OSGBINTERACTION_SAVE_RESTORE_HANDLER_H__
#endif
