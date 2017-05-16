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

#ifndef __OSGBDYNAMICS_PHYSICSTHREAD_H__
#define __OSGBDYNAMICS_PHYSICSTHREAD_H__ 1

#include <osgbDynamics/Export.h>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <OpenThreads/Barrier>
#include <osg/Timer>
#include <iostream>
#include <stdlib.h>

#include <btBulletDynamicsCommon.h>


// Forward declaraction
class btDynamicsWorld;

namespace osgbDynamics {


// Forward declaraction
class TripleBuffer;


/** \class PhysicsThread PhysicsThread.h <osgbDynamics/PhysicsThread.h>
\brief An OSG / OpenThreads class for asynchronous physics simulation.

*/
class OSGBDYNAMICS_EXPORT PhysicsThread : public OpenThreads::Thread
{
public:
    PhysicsThread( btDynamicsWorld* bw, osgbDynamics::TripleBuffer* tb=NULL );
    ~PhysicsThread();

    /** Specify the elapsed time parameter, used in call to stepSimulation.
    If value is <= 0.0, PhysicsThread uses the elapsed time.
    Default is 0.0 (use elapsed time). */
    void setTimeStep( btScalar timeStep );
    btScalar getTimeStep() const;

    /** Call Thread::start() to launch the thread. */
    virtual void run();

    /** Cause the thread to exit. Call Thread::isRunning() to verify that the
    thread has actually exited. */
    void stopPhysics();


    /** Temporarily pause the physics thread (to add a new rigid body,
    or move a static body, for example). You could also call
    stopPhysics then restart the thread, but this would incur the
    overhead of stopping and starting a thread. Use pause to temporarily
    halt the running thread. */
    void pause( bool pause );

    /** After telling the thread to pause, call isPaused() to ensure the
    thread has reached the pause gate and is idle. */
    bool isPaused() const;


    /** Allows access to the Bullet dynamics world. If the calling code
    intends to modify the dynamics world, it
    is responsible for ensuring that the thread is not running (and
    therefore not asynchronously modifying) the physics sim). */
    btDynamicsWorld* getDynamicsWorld() const { return( _bw ); }

protected:
    bool isStopping() const;

    btScalar _timeStep;

    btDynamicsWorld* _bw;
    osg::Timer _timer;
    bool _stopped;
    int _pauseCount;
    osg::Timer_t _lastTime;

    osgbDynamics::TripleBuffer* _tb;

    mutable OpenThreads::Mutex _stopMutex;
    mutable OpenThreads::Mutex _pauseMutex;
    mutable OpenThreads::Barrier _pauseGate;
};


// osgbDynamics
}

// __OSGBDYNAMICS_PHYSICSTHREAD_H__
#endif
