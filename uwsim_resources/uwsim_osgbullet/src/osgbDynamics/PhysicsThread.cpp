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

#include <osgbDynamics/PhysicsThread.h>
#include <osgbDynamics/TripleBuffer.h>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <OpenThreads/Block>

#include <btBulletDynamicsCommon.h>

#include <osg/Notify>
#include <iostream>
#include <string>


namespace osgbDynamics
{


PhysicsThread::PhysicsThread( btDynamicsWorld* bw, osgbDynamics::TripleBuffer* tb )
  : _timeStep( btScalar( 0.0 ) ),
    _bw( bw ),
    _stopped( true ),
    _pauseCount( 0 ),
    _tb( tb )
{
    ;
}
PhysicsThread::~PhysicsThread()
{
    ;
}


void
PhysicsThread::setTimeStep( btScalar timeStep )
{
    _timeStep = timeStep;
    _lastTime = _timer.tick();
}
btScalar
PhysicsThread::getTimeStep() const
{
    return( _timeStep );
}


void
PhysicsThread::run()
{
    _stopped = false;

    osg::Timer_t currentTime;

    _timer.setStartTick();
    _lastTime = _timer.tick();

    osg::notify( osg::INFO ) << "PhysicsThread: Starting" << std::endl;

    while( !isStopping() )
    {
        currentTime = _timer.tick();
        const btScalar deltaTime =
            ( ( _timeStep > 0.0 ) ?
                _timeStep :
                ( btScalar )( _timer.delta_s( _lastTime, currentTime ) ) );

        bool localPause;
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _pauseMutex );
            localPause = (_pauseCount > 0);
        }
        if( localPause )
        {
            osg::notify( osg::INFO ) << "PT: Pausing..." << std::endl;
            // Wait to be released.
            _pauseGate.block();
            osg::notify( osg::INFO ) << "PT: Released." << std::endl;

            // We were just released. Reset the block.
            _pauseGate.release();

            // Yawn! That was a nice nap. What time is it?
            currentTime = _timer.tick();
        }

        if( _tb != NULL )
        {
            // Run with triple buffering.
            _tb->beginWrite();
            _bw->stepSimulation( deltaTime );
            _tb->endWrite();
        }
        else
        {
            // Run normally. (Not sure if this is useful.)
            _bw->stepSimulation( deltaTime );
        }

        _lastTime = currentTime;
    }

    osg::notify( osg::INFO ) << "PhysicsThread: Stopping" << std::endl;
}

void
PhysicsThread::stopPhysics()
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _stopMutex );
    _stopped = true;
}
bool
PhysicsThread::isStopping() const
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _stopMutex );
    return( _stopped );
}

void
PhysicsThread::pause( bool pause )
{
    if( _stopped )
        return;

    bool block( false );
    bool unblock( false );
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _pauseMutex );
        if( pause )
        {
            _pauseCount++;
            block = ( _pauseCount == 1 );
        }
        else
        {
            _pauseCount--;
            unblock = ( _pauseCount == 0 );
        }
    }

    if( block )
    {
        // Give physics thread a change to hit the gate.
        YieldCurrentThread();

        // Block until physics thread hits the gate.
        while( !( isPaused() ) )
            OpenThreads::Thread::microSleep( 10 );
    }
    else if( unblock )
        _pauseGate.release();
}
bool
PhysicsThread::isPaused() const
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _pauseMutex );
    return( _pauseGate.numThreadsCurrentlyBlocked() > 0 );
}


// osgbDynamics
}
