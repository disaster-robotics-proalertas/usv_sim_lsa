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

#include <BulletDynamics/ConstraintSolver/btContactConstraint.h>
#include <osgbDynamics/Constraints.h>
#include "dotosgMatrixIO.h"

#include <iostream>
#include <string>

#include <osg/io_utils>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>


bool Constraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool Constraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy Constraint_Proxy
(
    new osgbDynamics::Constraint,
    "Constraint",
    "Object Constraint",
    Constraint_readLocalData,
    Constraint_writeLocalData
);


bool SliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool SliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy SliderConstraint_Proxy
(
    new osgbDynamics::SliderConstraint,
    "SliderConstraint",
    "Object Constraint SliderConstraint",
    SliderConstraint_readLocalData,
    SliderConstraint_writeLocalData
);


bool TwistSliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool TwistSliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy TwistSliderConstraint_Proxy
(
    new osgbDynamics::TwistSliderConstraint,
    "TwistSliderConstraint",
    "Object Constraint TwistSliderConstraint",
    TwistSliderConstraint_readLocalData,
    TwistSliderConstraint_writeLocalData
);


bool InternalSpringData_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool InternalSpringData_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy InternalSpringData_Proxy
(
    new osgbDynamics::InternalSpringData,
    "InternalSpringData",
    "Object InternalSpringData",
    InternalSpringData_readLocalData,
    InternalSpringData_writeLocalData
);


bool LinearSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool LinearSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy LinearSpringConstraint_Proxy
(
    new osgbDynamics::LinearSpringConstraint,
    "LinearSpringConstraint",
    "Object Constraint LinearSpringConstraint",
    LinearSpringConstraint_readLocalData,
    LinearSpringConstraint_writeLocalData
);


bool AngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool AngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy AngleSpringConstraint_Proxy
(
    new osgbDynamics::AngleSpringConstraint,
    "AngleSpringConstraint",
    "Object Constraint AngleSpringConstraint",
    AngleSpringConstraint_readLocalData,
    AngleSpringConstraint_writeLocalData
);


bool LinearAngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool LinearAngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy LinearAngleSpringConstraint_Proxy
(
    new osgbDynamics::LinearAngleSpringConstraint,
    "LinearAngleSpringConstraint",
    "Object Constraint LinearAngleSpringConstraint",
    LinearAngleSpringConstraint_readLocalData,
    LinearAngleSpringConstraint_writeLocalData
);


bool FixedConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool FixedConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy FixedConstraint_Proxy
(
    new osgbDynamics::FixedConstraint,
    "FixedConstraint",
    "Object Constraint FixedConstraint",
    FixedConstraint_readLocalData,
    FixedConstraint_writeLocalData
);


bool PlanarConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool PlanarConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy PlanarConstraint_Proxy
(
    new osgbDynamics::PlanarConstraint,
    "PlanarConstraint",
    "Object Constraint PlanarConstraint",
    PlanarConstraint_readLocalData,
    PlanarConstraint_writeLocalData
);


bool BoxConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool BoxConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy BoxConstraint_Proxy
(
    new osgbDynamics::BoxConstraint,
    "BoxConstraint",
    "Object Constraint BoxConstraint",
    BoxConstraint_readLocalData,
    BoxConstraint_writeLocalData
);


bool HingeConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool HingeConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy HingeConstraint_Proxy
(
    new osgbDynamics::HingeConstraint,
    "HingeConstraint",
    "Object Constraint HingeConstraint",
    HingeConstraint_readLocalData,
    HingeConstraint_writeLocalData
);


bool CardanConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool CardanConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy CardanConstraint_Proxy
(
    new osgbDynamics::CardanConstraint,
    "CardanConstraint",
    "Object Constraint CardanConstraint",
    CardanConstraint_readLocalData,
    CardanConstraint_writeLocalData
);


bool BallAndSocketConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool BallAndSocketConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy BallAndSocketConstraint_Proxy
(
    new osgbDynamics::BallAndSocketConstraint,
    "BallAndSocketConstraint",
    "Object Constraint BallAndSocketConstraint",
    BallAndSocketConstraint_readLocalData,
    BallAndSocketConstraint_writeLocalData
);


bool RagdollConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool RagdollConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy RagdollConstraint_Proxy
(
    new osgbDynamics::RagdollConstraint,
    "RagdollConstraint",
    "Object Constraint RagdollConstraint",
    RagdollConstraint_readLocalData,
    RagdollConstraint_writeLocalData
);


bool WheelSuspensionConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool WheelSuspensionConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy WheelSuspensionConstraint_Proxy
(
    new osgbDynamics::WheelSuspensionConstraint,
    "WheelSuspensionConstraint",
    "Object Constraint WheelSuspensionConstraint",
    WheelSuspensionConstraint_readLocalData,
    WheelSuspensionConstraint_writeLocalData
);


bool Constraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::Constraint& cons = static_cast< osgbDynamics::Constraint& >( obj );

    osg::Matrix m;
    if( readMatrix( m, fr, "rbAXform" ) )
        cons.setAXform( m );
    else
    {
        osg::notify( osg::WARN ) << "Constraint_readLocalData: Bad input data at \"rbAXform\"." << std::endl;
        return( false );
    }

    if( readMatrix( m, fr, "rbBXform" ) )
        cons.setBXform( m );
    else
    {
        osg::notify( osg::WARN ) << "Constraint_readLocalData: Bad input data at \"rbBXform\"." << std::endl;
        return( false );
    }

    return( true );
}
bool Constraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::Constraint& cons = static_cast< const osgbDynamics::Constraint& >( obj );

    writeMatrix( cons.getAXform(), fw, "rbAXform" );
    writeMatrix( cons.getBXform(), fw, "rbBXform" );

    return( true );
}


bool SliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::SliderConstraint& cons = static_cast< osgbDynamics::SliderConstraint& >( obj );

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "SliderConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Limit %f %f" ) )
    {
        osg::Vec2 limit;
        fr[1].getFloat( ( limit[0] ) );
        fr[2].getFloat( ( limit[1] ) );
        cons.setLimit( limit );
        fr += 3;
    }
    else
    {
        osg::notify( osg::WARN ) << "SliderConstraint_readLocalData: Bad input data at \"Limit\"." << std::endl;
        return( false );
    }

    return( true );
}
bool SliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::SliderConstraint& cons = static_cast< const osgbDynamics::SliderConstraint& >( obj );

    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.indent() << "Limit " << cons.getLimit() << std::endl;

    return( true );
}


bool TwistSliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::TwistSliderConstraint& cons = static_cast< osgbDynamics::TwistSliderConstraint& >( obj );

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( axis[0] );
        fr[2].getFloat( axis[1] );
        fr[3].getFloat( axis[2] );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "TwistSliderConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[1].getFloat( point[0] );
        fr[2].getFloat( point[1] );
        fr[3].getFloat( point[2] );
        cons.setPoint( point );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "TwistSliderConstraint_readLocalData: Bad input data at \"Point\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Slide limit %f %f" ) )
    {
        osg::Vec2 limit;
        fr[2].getFloat( ( limit[0] ) );
        fr[3].getFloat( ( limit[1] ) );
        cons.setSlideLimit( limit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "TwistSliderConstraint_readLocalData: Bad input data at \"Slide limit\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Twist limit %f %f" ) )
    {
        osg::Vec2 limit;
        fr[2].getFloat( ( limit[0] ) );
        fr[3].getFloat( ( limit[1] ) );
        cons.setTwistLimit( limit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "TwistSliderConstraint_readLocalData: Bad input data at \"Twist limit\"." << std::endl;
        return( false );
    }

    return( true );
}
bool TwistSliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::TwistSliderConstraint& cons = static_cast< const osgbDynamics::TwistSliderConstraint& >( obj );

    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.indent() << "Point " << cons.getPoint() << std::endl;
    fw.indent() << "Slide limit " << cons.getSlideLimit() << std::endl;
    fw.indent() << "Twist limit " << cons.getTwistLimit() << std::endl;

    return( true );
}


bool InternalSpringData_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::InternalSpringData& data = static_cast< osgbDynamics::InternalSpringData& >( obj );

    if( fr.matchSequence( "Linear lower limits %f %f %f" ) )
    {
        osg::Vec3 vec;
        fr[3].getFloat( ( vec[0] ) );
        fr[4].getFloat( ( vec[1] ) );
        fr[5].getFloat( ( vec[2] ) );
        data._linearLowerLimits = vec;
        fr += 6;
    }
    else
    {
        osg::notify( osg::WARN ) << "InternalSpringData_readLocalData: Bad input data at \"Linear lower limits\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Linear upper limits %f %f %f" ) )
    {
        osg::Vec3 vec;
        fr[3].getFloat( ( vec[0] ) );
        fr[4].getFloat( ( vec[1] ) );
        fr[5].getFloat( ( vec[2] ) );
        data._linearUpperLimits = vec;
        fr += 6;
    }
    else
    {
        osg::notify( osg::WARN ) << "InternalSpringData_readLocalData: Bad input data at \"Linear upper limits\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Angular lower limits %f %f %f" ) )
    {
        osg::Vec3 vec;
        fr[3].getFloat( ( vec[0] ) );
        fr[4].getFloat( ( vec[1] ) );
        fr[5].getFloat( ( vec[2] ) );
        data._angularLowerLimits = vec ;
        fr += 6;
    }
    else
    {
        osg::notify( osg::WARN ) << "InternalSpringData_readLocalData: Bad input data at \"Angular lower limits\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Angular upper limits %f %f %f" ) )
    {
        osg::Vec3 vec;
        fr[3].getFloat( ( vec[0] ) );
        fr[4].getFloat( ( vec[1] ) );
        fr[5].getFloat( ( vec[2] ) );
        data._angularUpperLimits = vec ;
        fr += 6;
    }
    else
    {
        osg::notify( osg::WARN ) << "InternalSpringData_readLocalData: Bad input data at \"Angular upper limits\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Enable" ) )
    {
        int idx, value;
        for( idx=0; idx<6; idx++ )
        {
            fr[ idx + 1 ].getInt( value );
            data._enable[ idx ] = ( value != 0 );
        }
        fr += 7;
    }
    else
    {
        osg::notify( osg::WARN ) << "InternalSpringData_readLocalData: Bad input data at \"Enable\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Stiffness %f %f %f %f %f %f" ) )
    {
        int idx;
        for( idx=0; idx<6; idx++ )
            fr[ idx + 1 ].getFloat( data._stiffness[ idx ] );
        fr += 7;
    }
    else
    {
        osg::notify( osg::WARN ) << "InternalSpringData_readLocalData: Bad input data at \"Stiffness\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Damping %f %f %f %f %f %f" ) )
    {
        int idx;
        for( idx=0; idx<6; idx++ )
            fr[ idx + 1 ].getFloat( data._damping[ idx ] );
        fr += 7;
    }
    else
    {
        osg::notify( osg::WARN ) << "InternalSpringData_readLocalData: Bad input data at \"Damping\"." << std::endl;
        return( false );
    }

    return( true );
}
bool InternalSpringData_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::InternalSpringData& data = static_cast< const osgbDynamics::InternalSpringData& >( obj );

    fw.indent() << "Linear lower limits " << data._linearLowerLimits << std::endl;
    fw.indent() << "Linear upper limits " << data._linearUpperLimits << std::endl;
    fw.indent() << "Angular lower limits " << data._angularLowerLimits << std::endl;
    fw.indent() << "Angular upper limits " << data._angularUpperLimits << std::endl;

    int idx;
    fw.indent() << "Enable ";
    for( idx=0; idx<6; idx++ )
        fw << ( data._enable[ idx ] ? 1 : 0 ) << " ";
    fw << std::endl;

    fw.indent() << "Stiffness ";
    for( idx=0; idx<6; idx++ )
        fw << data._stiffness[ idx ] << " ";
    fw << std::endl;

    fw.indent() << "Damping ";
    for( idx=0; idx<6; idx++ )
        fw << data._damping[ idx ] << " ";
    fw << std::endl;

    return( true );
}


bool LinearSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::LinearSpringConstraint& cons = static_cast< osgbDynamics::LinearSpringConstraint& >( obj );

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "LinearSpringConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    osg::ref_ptr< osgbDynamics::InternalSpringData > isd =
        static_cast< osgbDynamics::InternalSpringData* >( fr.readObject() );
    cons.setSpringData( isd.get() );

    return( true );
}
bool LinearSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::LinearSpringConstraint& cons = static_cast< const osgbDynamics::LinearSpringConstraint& >( obj );

    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.writeObject( *( cons.getSpringData() ) );

    return( true );
}


bool AngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::AngleSpringConstraint& cons = static_cast< osgbDynamics::AngleSpringConstraint& >( obj );

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "AngleSpringConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Pivot point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[2].getFloat( ( point[0] ) );
        fr[3].getFloat( ( point[1] ) );
        fr[4].getFloat( ( point[2] ) );
        cons.setPivotPoint( point );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "AngleSpringConstraint_readLocalData: Bad input data at \"Pivot point\"." << std::endl;
        return( false );
    }

    osg::ref_ptr< osgbDynamics::InternalSpringData > isd =
        static_cast< osgbDynamics::InternalSpringData* >( fr.readObject() );
    cons.setSpringData( isd.get() );

    return( true );
}
bool AngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::AngleSpringConstraint& cons = static_cast< const osgbDynamics::AngleSpringConstraint& >( obj );

    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.indent() << "Pivot point " << cons.getPivotPoint() << std::endl;
    fw.writeObject( *( cons.getSpringData() ) );

    return( true );
}


bool LinearAngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::LinearAngleSpringConstraint& cons = static_cast< osgbDynamics::LinearAngleSpringConstraint& >( obj );

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "LinearAngleSpringConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Pivot point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[2].getFloat( ( point[0] ) );
        fr[3].getFloat( ( point[1] ) );
        fr[4].getFloat( ( point[2] ) );
        cons.setPivotPoint( point );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "LinearAngleSpringConstraint_readLocalData: Bad input data at \"Pivot point\"." << std::endl;
        return( false );
    }

    osg::ref_ptr< osgbDynamics::InternalSpringData > isd =
        static_cast< osgbDynamics::InternalSpringData* >( fr.readObject() );
    cons.setSpringData( isd.get() );

    return( true );
}
bool LinearAngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::LinearAngleSpringConstraint& cons = static_cast< const osgbDynamics::LinearAngleSpringConstraint& >( obj );

    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.indent() << "Pivot point " << cons.getPivotPoint() << std::endl;
    fw.writeObject( *( cons.getSpringData() ) );

    return( true );
}


bool FixedConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // No-op, but this function must exist to supprt the TwistSliderConstraint object.
    return( true );
}
bool FixedConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // No-op, but this function must exist to supprt the TwistSliderConstraint object.
    return( true );
}


bool PlanarConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::PlanarConstraint& cons = static_cast< osgbDynamics::PlanarConstraint& >( obj );

    if( fr.matchSequence( "Low limit %f %f" ) )
    {
        osg::Vec2 loLimit;
        fr[2].getFloat( ( loLimit[0] ) );
        fr[3].getFloat( ( loLimit[1] ) );
        cons.setLowLimit( loLimit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "PlanarConstraint_readLocalData: Bad input data at \"Low limit\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "High limit %f %f" ) )
    {
        osg::Vec2 hiLimit;
        fr[2].getFloat( ( hiLimit[0] ) );
        fr[3].getFloat( ( hiLimit[1] ) );
        cons.setHighLimit( hiLimit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "PlanarConstraint_readLocalData: Bad input data at \"High limit\"." << std::endl;
        return( false );
    }

    osg::Matrix m;
    if( readMatrix( m, fr, "Orient" ) )
        cons.setOrientation( m );
    else
    {
        osg::notify( osg::WARN ) << "PlanarConstraint_readLocalData: Bad input data at \"Orient\"." << std::endl;
        return( false );
    }

    return( true );
}
bool PlanarConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::PlanarConstraint& cons = static_cast< const osgbDynamics::PlanarConstraint& >( obj );

    fw.indent() << "Low limit " << cons.getLowLimit() << std::endl;
    fw.indent() << "High limit " << cons.getHighLimit() << std::endl;
    writeMatrix( cons.getOrientation(), fw, "Orient" );

    return( true );
}


bool BoxConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::BoxConstraint& cons = static_cast< osgbDynamics::BoxConstraint& >( obj );

    if( fr.matchSequence( "Low limit %f %f %f" ) )
    {
        osg::Vec3 loLimit;
        fr[2].getFloat( ( loLimit[0] ) );
        fr[3].getFloat( ( loLimit[1] ) );
        fr[4].getFloat( ( loLimit[2] ) );
        cons.setLowLimit( loLimit );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "BoxConstraint_readLocalData: Bad input data at \"Low limit\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "High limit %f %f %f" ) )
    {
        osg::Vec3 hiLimit;
        fr[2].getFloat( ( hiLimit[0] ) );
        fr[3].getFloat( ( hiLimit[1] ) );
        fr[4].getFloat( ( hiLimit[2] ) );
        cons.setHighLimit( hiLimit );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "BoxConstraint_readLocalData: Bad input data at \"High limit\"." << std::endl;
        return( false );
    }

    osg::Matrix m;
    if( readMatrix( m, fr, "Orient" ) )
        cons.setOrientation( m );
    else
    {
        osg::notify( osg::WARN ) << "BoxConstraint_readLocalData: Bad input data at \"Orient\"." << std::endl;
        return( false );
    }

    return( true );
}
bool BoxConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::BoxConstraint& cons = static_cast< const osgbDynamics::BoxConstraint& >( obj );

    fw.indent() << "Low limit " << cons.getLowLimit() << std::endl;
    fw.indent() << "High limit " << cons.getHighLimit() << std::endl;
    writeMatrix( cons.getOrientation(), fw, "Orient" );

    return( true );
}


bool HingeConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::HingeConstraint& cons = static_cast< osgbDynamics::HingeConstraint& >( obj );

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "HingeConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Pivot point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[2].getFloat( point[0] );
        fr[3].getFloat( point[1] );
        fr[4].getFloat( point[2] );
        cons.setPivotPoint( point );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "HingeConstraint_readLocalData: Bad input data at \"Pivot point\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Limit %f %f" ) )
    {
        osg::Vec2 limit;
        fr[1].getFloat( limit[0] );
        fr[2].getFloat( limit[1] );
        cons.setLimit( limit );
        fr += 3;
    }
    else
    {
        osg::notify( osg::WARN ) << "HingeConstraint_readLocalData: Bad input data at \"Limit\"." << std::endl;
        return( false );
    }

    return( true );
}
bool HingeConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::HingeConstraint& cons = static_cast< const osgbDynamics::HingeConstraint& >( obj );

    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.indent() << "Pivot point " << cons.getPivotPoint() << std::endl;
    fw.indent() << "Limit " << cons.getLimit() << std::endl;

    return( true );
}


bool CardanConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::CardanConstraint& cons = static_cast< osgbDynamics::CardanConstraint& >( obj );

    if( fr.matchSequence( "AxisA %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxisA( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "CardanConstraint_readLocalData: Bad input data at \"AxisA\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "AxisB %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxisB( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "CardanConstraint_readLocalData: Bad input data at \"AxisB\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Anchor point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[2].getFloat( ( point[0] ) );
        fr[3].getFloat( ( point[1] ) );
        fr[4].getFloat( ( point[2] ) );
        cons.setAnchorPoint( point );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "CardanConstraint_readLocalData: Bad input data at \"Anchor point\"." << std::endl;
        return( false );
    }

    return( true );
}
bool CardanConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::CardanConstraint& cons = static_cast< const osgbDynamics::CardanConstraint& >( obj );

    fw.indent() << "AxisA " << cons.getAxisA() << std::endl;
    fw.indent() << "AxisB " << cons.getAxisB() << std::endl;
    fw.indent() << "Anchor point " << cons.getAnchorPoint() << std::endl;

    return( true );
}


bool BallAndSocketConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::BallAndSocketConstraint& cons = static_cast< osgbDynamics::BallAndSocketConstraint& >( obj );

    if( fr.matchSequence( "Point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[1].getFloat( ( point[0] ) );
        fr[2].getFloat( ( point[1] ) );
        fr[3].getFloat( ( point[2] ) );
        cons.setPoint( point );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "BallAndSocketConstraint_readLocalData: Bad input data at \"Point\"." << std::endl;
        return( false );
    }

    return( true );
}
bool BallAndSocketConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::BallAndSocketConstraint& cons = static_cast< const osgbDynamics::BallAndSocketConstraint& >( obj );

    fw.indent() << "Point " << cons.getPoint() << std::endl;

    return( true );
}


bool RagdollConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::RagdollConstraint& cons = static_cast< osgbDynamics::RagdollConstraint& >( obj );

    if( fr.matchSequence( "Point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[1].getFloat( ( point[0] ) );
        fr[2].getFloat( ( point[1] ) );
        fr[3].getFloat( ( point[2] ) );
        cons.setPoint( point );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "RagdollConstraint_readLocalData: Bad input data at \"Point\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "RagdollConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Angle %f" ) )
    {
        double angle;
        fr[1].getFloat( angle );
        cons.setAngle( angle );
    }
    else
    {
        osg::notify( osg::WARN ) << "RagdollConstraint_readLocalData: Bad input data at \"Angle\"." << std::endl;
        return( false );
    }

    return( true );
}
bool RagdollConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::RagdollConstraint& cons = static_cast< const osgbDynamics::RagdollConstraint& >( obj );

    fw.indent() << "Point " << cons.getPoint() << std::endl;
    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.indent() << "Angle " << cons.getAngle() << std::endl;

    return( true );
}


bool WheelSuspensionConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::WheelSuspensionConstraint& cons = static_cast< osgbDynamics::WheelSuspensionConstraint& >( obj );

    if( fr.matchSequence( "Spring axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[2].getFloat( ( axis[0] ) );
        fr[3].getFloat( ( axis[1] ) );
        fr[4].getFloat( ( axis[2] ) );
        cons.setSpringAxis( axis );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "WheelSuspensionConstraint_readLocalData: Bad input data at \"Spring Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Axle axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[2].getFloat( ( axis[0] ) );
        fr[3].getFloat( ( axis[1] ) );
        fr[4].getFloat( ( axis[2] ) );
        cons.setAxleAxis( axis );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "WheelSuspensionConstraint_readLocalData: Bad input data at \"Axle axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Linear limit %f %f" ) )
    {
        osg::Vec2 limit;
        fr[2].getFloat( ( limit[0] ) );
        fr[3].getFloat( ( limit[1] ) );
        cons.setLinearLimit( limit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "WheelSuspensionConstraint_readLocalData: Bad input data at \"Linear limit\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Angle limit %f %f" ) )
    {
        osg::Vec2 limit;
        fr[2].getFloat( ( limit[0] ) );
        fr[3].getFloat( ( limit[1] ) );
        cons.setAngleLimit( limit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "WheelSuspensionConstraint_readLocalData: Bad input data at \"Angle limit\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Anchor point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[2].getFloat( ( point[0] ) );
        fr[3].getFloat( ( point[1] ) );
        fr[4].getFloat( ( point[2] ) );
        cons.setAnchorPoint( point );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "WheelSuspensionConstraint_readLocalData: Bad input data at \"Anchor point\"." << std::endl;
        return( false );
    }

    return( true );
}
bool WheelSuspensionConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::WheelSuspensionConstraint& cons = static_cast< const osgbDynamics::WheelSuspensionConstraint& >( obj );

    fw.indent() << "Spring axis " << cons.getSpringAxis() << std::endl;
    fw.indent() << "Axle axis " << cons.getAxleAxis() << std::endl;
    fw.indent() << "Linear limit " << cons.getLinearLimit() << std::endl;
    fw.indent() << "Angle limit " << cons.getAngleLimit() << std::endl;
    fw.indent() << "Anchor point " << cons.getAnchorPoint() << std::endl;

    return( true );
}
