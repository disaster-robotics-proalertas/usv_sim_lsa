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
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/Utils.h>
#include "dotosgMatrixIO.h"

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osg/MatrixTransform>

#include <osgwTools/AbsoluteModelTransform.h>

#include <iostream>
#include <string>

#include <osg/io_utils>


bool PhysicsData_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool PhysicsData_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy PhysicsData_Proxy
(
    new osgbDynamics::PhysicsData,
    "PhysicsData",
    "Object PhysicsData",
    PhysicsData_readLocalData,
    PhysicsData_writeLocalData
);




bool PhysicsData_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::PhysicsData& pd = static_cast< osgbDynamics::PhysicsData& >( obj );
    bool advance( false );

    unsigned int version( 0 );
    if( fr.matchSequence( "Version %i" ) )
    {
        fr[1].getUInt( version );
        fr+=2;
        advance = true;

        osg::notify( osg::INFO ) << "OSGB: Found version " << version << std::endl;

        pd._cr = static_cast< osgbDynamics::CreationRecord* >( fr.readObject() );
        osg::notify( osg::INFO ) << "OSGB: CreationRecord " << pd._cr.get() << std::endl;

        bool readM = readMatrix( pd._osgTransform, fr, "OSGTransform" );
        osg::notify( osg::INFO ) << "OSGB: OSGTransform " << readM << std::endl;

        readM = readMatrix( pd._bodyWorldTransform, fr, "BodyWorldTransform" );
        osg::notify( osg::INFO ) << "OSGB: BodyWorldTransform " << readM << std::endl;
        if( fr.matchSequence( "Linear velocity %f %f %f" ) )
        {
            osg::Vec3& v( pd._linearVelocity );
            fr[2].getFloat( (v[0]) );
            fr[3].getFloat( (v[1]) );
            fr[4].getFloat( (v[2]) );
            fr += 5;

            osg::notify( osg::INFO ) << "OSGB: Found linear velocity " << v << std::endl;
        }
        if( fr.matchSequence( "Angular velocity %f %f %f" ) )
        {
            osg::Vec3& v( pd._angularVelocity );
            fr[2].getFloat( (v[0]) );
            fr[3].getFloat( (v[1]) );
            fr[4].getFloat( (v[2]) );
            fr += 5;

            osg::notify( osg::INFO ) << "OSGB: Found angular velocity " << v << std::endl;
        }
        if( version == 2 )
        {
            if( fr.matchSequence( "Friction %f" ) )
            {
                fr[1].getFloat( pd._friction );
                fr += 2;
                
                osg::notify( osg::INFO ) << "OSGB: Found friction " << pd._friction << std::endl;
            }
            if( fr.matchSequence( "Restitution %f" ) )
            {
                fr[1].getFloat( pd._restitution );
                fr += 2;
                
                osg::notify( osg::INFO ) << "OSGB: Found restitution " << pd._restitution << std::endl;
            }
        }
    }
    else if( fr.matchSequence( "FileName" ) )
    {
        pd._fileName = fr[1].getStr();
        fr+=2;
        advance = true;

        osg::notify( osg::INFO ) << "OSGB: Found fileName " << pd._fileName << std::endl;
    }

    osg::notify( osg::INFO ) << "OSGB: advance " << advance << std::endl;
    return( advance );
}

bool PhysicsData_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::PhysicsData& pd = static_cast< const osgbDynamics::PhysicsData& >( obj );

    fw.indent() << "Version " << pd.getVersion() << std::endl;

    if( pd._cr == NULL )
        osg::notify( osg::WARN ) << "PhysicsData_writeLocalData: Can't write NULL CreationRecord." << std::endl;
    else
        fw.writeObject( *( pd._cr ) );

    // The AMT matrix is different from the RB matrix. We need to save it
    // separately here so that we can display the OSG subgraph transformed
    // correctly while waiting for physics data to load.
    btMotionState* motion = pd._body->getMotionState();
    osgbDynamics::MotionState* ms = dynamic_cast< osgbDynamics::MotionState* >( motion );
    if( ms != NULL )
    {
        osg::Transform* trans = ms->getTransform();
        if( trans->asMatrixTransform() != NULL )
        {
            const osg::Matrix& mt( trans->asMatrixTransform()->getMatrix() );
            writeMatrix( mt, fw, "OSGTransform" );
        }
        else
        {
            osgwTools::AbsoluteModelTransform* amt = dynamic_cast< osgwTools::AbsoluteModelTransform* >( trans );
            if( amt != NULL )
            {
                const osg::Matrix& mt( amt->getMatrix() );
                writeMatrix( mt, fw, "OSGTransform" );
            }
        }
    }

    // Save rigid body state.
    osg::Matrix m( osgbCollision::asOsgMatrix( pd._body->getWorldTransform() ) );
    writeMatrix( m, fw, "BodyWorldTransform" );
    osg::Vec3 lv( osgbCollision::asOsgVec3( pd._body->getLinearVelocity() ) );
    fw.indent() << "Linear velocity " << lv << std::endl;
    osg::Vec3 av( osgbCollision::asOsgVec3( pd._body->getAngularVelocity() ) );
    fw.indent() << "Angular velocity " << av << std::endl;
    fw.indent() << "Friction " << pd._body->getFriction() << std::endl;
    fw.indent() << "Restitution " << pd._body->getRestitution() << std::endl;

    if( !pd._fileName.empty() )
        fw.indent() << "FileName \"" << pd._fileName << "\"" << std::endl;

    return( true );
}
