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


#ifndef __OSGWTOOLS_DATA_LOADER_H__
#define __OSGWTOOLS_DATA_LOADER_H__ 1

#include <osgwTools/Export.h>
#include <osgDB/ReadFile>

#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include <string>


namespace osgwTools
{



// Use a thread to call osgDB::readNodeFile.
class OSGWTOOLS_EXPORT LoadNodeThread : public OpenThreads::Thread
{
public:
    LoadNodeThread( const std::string& fileName )
      : _fileName( fileName )
    {}

    ~LoadNodeThread()
    {}

    void run()
    {
        if( _fileName.empty() )
            _node = NULL;
        else
            _node = osgDB::readNodeFile( _fileName );
    }

    const std::string _fileName;
    osg::ref_ptr< osg::Node > _node;
};

// Use a thread to call osgDB::readObjectFile.
class OSGWTOOLS_EXPORT LoadObjectThread : public OpenThreads::Thread
{
public:
    LoadObjectThread( const std::string& fileName )
      : _fileName( fileName )
    {}

    ~LoadObjectThread()
    {}

    void run()
    {
        if( _fileName.empty() )
            _object = NULL;
        else
            _object = osgDB::readObjectFile( _fileName );
    }

    const std::string _fileName;
    osg::ref_ptr< osg::Object > _object;
};


// osgwTools
}


// __OSGWTOOLS_DATA_LOADER_H__
#endif
