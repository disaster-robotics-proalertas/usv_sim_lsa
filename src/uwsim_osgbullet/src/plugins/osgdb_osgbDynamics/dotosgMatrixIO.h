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

#ifndef __DOTOSG_OSGBDYNAMICS_MATRIX_IO_H__
#define __DOTOSG_OSGBDYNAMICS_MATRIX_IO_H__ 1


#include <osgDB/Input>
#include <osgDB/Output>
#include <osg/Matrix>



// OSG Cuts and pastes these functions all over the dot osg plugin support code.
// I've declared them in this header and define them once, which makes it
// callable by any code in the osgdb_osgbDynamics plugin. These functions
// should be in am osgWorks shared library, as they are unfortunately not
// available in shared form in OSG itself.

bool readMatrix( osg::Matrix& matrix, osgDB::Input& fr, const char* keyword="Matrix" );

bool writeMatrix( const osg::Matrix& matrix, osgDB::Output& fw, const char* keyword="Matrix" );


// __DOTOSG_OSGBDYNAMICS_MATRIX_IO_H__
#endif
