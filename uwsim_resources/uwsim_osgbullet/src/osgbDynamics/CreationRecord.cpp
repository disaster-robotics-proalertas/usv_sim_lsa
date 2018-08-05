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

#include <osgbDynamics/CreationRecord.h>

#include <osgbCollision/CollisionShapes.h>

#include <osg/Object>
#include <osg/Vec3>

#include <btBulletDynamicsCommon.h>

#include <string>


namespace osgbDynamics
{


CreationRecord::CreationRecord()
  : _sceneGraph( NULL ),
    _version( 3 ),
    _com( 0., 0., 0. ),
    _comSet( false ),
    _margin( 0.f ),
    _marginSet( false ),
    _scale( osg::Vec3( 1., 1., 1. ) ),
    _parentTransform( osg::Matrix::identity() ),
    _shapeType( BOX_SHAPE_PROXYTYPE ),
    _mass( 1.f ),
    _restitution( 0.f ),
    _friction( 1.f ),
    _axis( osgbCollision::Z ),
    _reductionLevel( CreationRecord::NONE ),
    _overall( false )
{
}
CreationRecord::CreationRecord( const CreationRecord& rhs, osg::CopyOp copyop )
  : _sceneGraph( rhs._sceneGraph ),
    _version( rhs._version ),
    _com( rhs._com ),
    _comSet( rhs._comSet ),
    _margin( rhs._margin ),
    _marginSet( rhs._marginSet ),
    _scale( rhs._scale ),
    _parentTransform( rhs._parentTransform ),
    _shapeType( rhs._shapeType ),
    _mass( rhs._mass ),
    _restitution( rhs._restitution ),
    _friction( rhs._friction ),
    _axis( rhs._axis ),
    _reductionLevel( rhs._reductionLevel ),
    _overall( rhs._overall )
{
}

void CreationRecord::setMargin( const float margin )
{
    _margin = margin;
    _marginSet = true;
}

void CreationRecord::setCenterOfMass( const osg::Vec3& com )
{
    _com = com;
    _comSet = true;
}


// osgbDynamics
}
