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

#ifndef __OSGBBULLET_OSGDB_SGB__
#define __OSGBBULLET_OSGDB_SGB__ 1


#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>


class ReaderWriterSGB : public osgDB::ReaderWriter
{
public:
    ReaderWriterSGB();
    ~ReaderWriterSGB();

    const char* className() const;

    virtual osgDB::ReaderWriter::ReadResult readObject( const std::string& fileName, const Options* options=NULL ) const;
    virtual osgDB::ReaderWriter::WriteResult writeObject( const osg::Object& obj, const std::string& fileName, const Options* options=NULL ) const;

protected:
};


// __OSGBBULLET_OSGDB_SGB__
#endif
