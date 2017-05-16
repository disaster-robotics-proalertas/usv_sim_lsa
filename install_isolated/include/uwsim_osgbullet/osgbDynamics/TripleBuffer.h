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


#ifndef __OSGBDYNAMICS_TRIPLEBUFFER_H__
#define __OSGBDYNAMICS_TRIPLEBUFFER_H__ 1

#include <osgbDynamics/Export.h>
#include <OpenThreads/Mutex>
#include <iostream>
#include <stdlib.h>


namespace osgbDynamics {


/** \class TripleBuffer TripleBuffer.h <osgbDynamics/TripleBuffer.h>
\brief A triple buffer mechanism to support physics and rendering in separate threads.

This class allows the read
thread to always access the latest complete set of updated
data without blocking, and allows the write thread to always 
switch to an available buffer to write the next set of data
without blocking.

Multiple heterogeneous chunks of data may be stored in buffers
by using reserve() to specify a data size and retrieve back the
unsigned int index into the TripleBuffer storage. This index is a count of
least-resolvable machine units (usually bytes).

In a typical usage case, each MotionState has a matrix
stored in the triple buffer object. The physics sim thread wraps the
stepSimulation call with beginWrite() and endWrite(). When Bullet sets the
world transform of the collision objects, MotionState intercepts the
transforms and stores them in the current write buffer. Concurrently,
during the scene graph update phase, the rendering thread calls beginRead(),
uses the data to update matrices in the scene graph, then calls endRead().
*/
class OSGBDYNAMICS_EXPORT TripleBuffer
{
public:
    TripleBuffer( unsigned int initialSize=8192 );
    ~TripleBuffer();

    /** \brief Sets the maximum size of all three buffers.

    This function reallocates the TripleBuffer storage by calling reallocate()
    on all three buffers so that
    they contain \c size bytes of contiguous memory. Because reallocation
    can be expensive, applications should avoid repeated resize() calls.
    In typical usage, applications call resize() once at init time. */
    void resize( unsigned int size );

    /** \brief Reserve a chunk of memory.

    Applications should call reserve() whenever they want to access a chunk of
    data by two threads. For example, MotionState calls reserve() to allocate
    space for a btTransform. The \c size parameter is set to sizeof(btTransform).
    The return value is used by the Physics thread to write the transform, and
    used by the MotionState to update the matrix of the OSG Transform node it manages.

    \param size The size of memory to reserve. If the currently allocated
        mrmory plus \c size exceeds the total storage specified with resize(),
        this function reallocates the TripleBuffer storage to accomodate the request.
    \param data If specified, \c size bytes of data from this address are
        copied into the allocation location.
    \return The byte offset into the TripleBuffer storage.
    */
    unsigned int reserve( unsigned int size, char* data=NULL );

    /** \brief Mark an INVALID buffer as WRITE and return its address. */
    char* beginWrite();
    /** \brief Return the address of the current WRITE buffer. */
    char* writeAddress();
    /** \brief Mark the current WRITE buffer as UPDATED. */
    void endWrite();

    /** \brief Mark an UPDATED buffer as READ and return its address. */
    char* beginRead();
    /** \brief Return the address of the current READ buffer. */
    char* readAddress();
    /** \brief Mark the current READ buffer as INVALID. */
    void endRead();

    /** \brief Useful during development. */
    void debugDump( const std::string& msg, std::ostream& oStr ) const;

protected:
    unsigned int _currentSize;
    unsigned int _nextFree;

    typedef enum {
        INVALID = 0,
        UPDATED = 1,
        READ = 2,
        WRITE = 3
    } BufferStatus;
    BufferStatus _status[ 3 ];
    char* _buf[ 3 ];

    char* _writeAddress;
    char* _readAddress;

    /** \brief Returns the buffer with the specified status.
    \return 0, 1, or 2, if a buffer is found. -1 otherwise. */
    int get( BufferStatus status );

    /** \brief reallocate a single buffer.

    Uses \c new to allocate the buffer, \c memcpy to copy date from the old
    location to the new, then uses \c delete to deallocate the old buffer.
    */
    void reallocate( unsigned int index, unsigned int size );

    OpenThreads::Mutex _lock;
};


// osgbDynamics
}

// __OSGBDYNAMICS_TRIPLEBUFFER_H__
#endif
