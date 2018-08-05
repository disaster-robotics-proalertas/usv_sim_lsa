/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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

#include <osgwTools/MultiCameraProjectionMatrix.h>

#include <osg/Matrixd>
#include <osg/Matrixf>
#include <osg/CullSettings>
#include <osg/Camera>
#include <osg/Uniform>
#include <osgUtil/CullVisitor>
#include <osgUtil/RenderStage>
#include <osg/Math>
#include <OpenThreads/Thread>
#include <OpenThreads/ScopedLock>

#include <osg/io_utils>


namespace osgwTools
{



/** This is a copy of the OSG function, but has been modified top
not be a template (operates only on Matrixd) and ignores the
CullSetting near/far ratio value (uses hardcoded 0.0005, the OSG
default). */
bool clampProjection( osg::Matrixd& projection, double& znear, double& zfar )
{
    const double epsilon = 1e-6;
    if( zfar < znear - epsilon )
    {
        osg::notify(osg::INFO)<<"_clampProjectionMatrix not applied, invalid depth range, znear = "<<znear<<"  zfar = "<<zfar<<std::endl;
        return( false );
    }
    
    if( zfar < znear + epsilon )
    {
        // znear and zfar are too close together and could cause divide by zero problems
        // late on in the clamping code, so move the znear and zfar apart.
        const double average = (znear+zfar)*0.5;
        znear = average - epsilon;
        zfar = average + epsilon;
        // osg::notify(osg::INFO) << "_clampProjectionMatrix widening znear and zfar to "<<znear<<" "<<zfar<<std::endl;
    }

    if( ( fabs( projection( 0, 3 ) ) < epsilon ) &&
        ( fabs( projection( 1, 3 ) ) < epsilon ) &&
        ( fabs( projection( 2, 3 ) ) < epsilon ) )
    {
        // osg::notify(osg::INFO) << "Orthographic matrix before clamping"<<projection<<std::endl;

        double delta_span = (zfar-znear)*0.02;
        if( delta_span < 1. )
            delta_span = 1.0;
        const double desired_znear = znear - delta_span;
        const double desired_zfar = zfar + delta_span;

        // assign the clamped values back to the computed values.
        znear = desired_znear;
        zfar = desired_zfar;

        projection(2,2)=-2.0f/(desired_zfar-desired_znear);
        projection(3,2)=-(desired_zfar+desired_znear)/(desired_zfar-desired_znear);

        // osg::notify(osg::INFO) << "Orthographic matrix after clamping "<<projection<<std::endl;
    }
    else
    {
        // osg::notify(osg::INFO) << "Persepective matrix before clamping"<<projection<<std::endl;
        //std::cout << "_computed_znear"<<_computed_znear<<std::endl;
        //std::cout << "_computed_zfar"<<_computed_zfar<<std::endl;

        const double zfarPushRatio = 1.02;
        const double znearPullRatio = 0.98;

        //znearPullRatio = 0.99; 

        double desired_znear = znear * znearPullRatio;
        double desired_zfar = zfar * zfarPushRatio;

        // near plane clamping.
        const double nearFarRatio( 0.0005 );
        const double min_near_plane = zfar*nearFarRatio;
        if( desired_znear < min_near_plane )
            desired_znear = min_near_plane;

        // assign the clamped values back to the computed values.
        znear = desired_znear;
        zfar = desired_zfar;

        const double trans_near_plane = (-desired_znear*projection(2,2)+projection(3,2))/(-desired_znear*projection(2,3)+projection(3,3));
        const double trans_far_plane = (-desired_zfar*projection(2,2)+projection(3,2))/(-desired_zfar*projection(2,3)+projection(3,3));

        const double ratio = fabs(2.0/(trans_near_plane-trans_far_plane));
        const double center = -(trans_near_plane+trans_far_plane)/2.0;

        projection.postMult(osg::Matrixd( 1., 0., 0., 0.,
                                          0., 1., 0., 0.,
                                          0., 0., ratio, 0.,
                                          0., 0., center*ratio, 1. ) );

        // osg::notify(osg::INFO) << "Persepective matrix after clamping"<<projection<<std::endl;
    }

    return( true );
}



/** \brief Save the subordinate Camera near & far values.
\details Near and far values are kept in a map indexed by current
thread ID. This class uses a Mutex for thread safety. */
class SubCameraClampCB : public osg::Camera::ClampProjectionMatrixCallback
{
public:
    SubCameraClampCB()
    {
    }
    ~SubCameraClampCB()
    {
    }

    virtual bool clampProjectionMatrixImplementation( osg::Matrixf& projection, double& znear, double& zfar ) const
    {
        // Support float by computing in double precision.
        osg::Matrixd m( projection );
        const bool result = clampProjectionMatrixImplementation( m, znear, zfar );
        projection = m;
        return( result );
    }
    virtual bool clampProjectionMatrixImplementation( osg::Matrixd& projection, double& znear, double& zfar ) const
    {
        // Do normal clamping first. This only encompasses the subgraph below this
        // subordinate Camera, so the FFP projection matrix should not be used.
        // We really only want the near and far values.
        const bool result = osgwTools::clampProjection( projection, znear, zfar );

        // Get the current thread ID for our map index. Use 0 if unavailable.
        OpenThreads::Thread* thread( OpenThreads::Thread::CurrentThread() );
        int id( 0 );
        if( thread != NULL )
            id = thread->getThreadId();

        osg::notify( osg::DEBUG_FP ) << "Thread: " << id << std::endl;
        osg::notify( osg::DEBUG_FP ) << "Sub Camera near/far: " << znear << " " << zfar << std::endl;

        // Store the near and far values in the map for later retrieval by
        // RootCameraClampCB (via our getNearFar() method).
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _mutex );
        _values[ id ] = DoublePair( znear, zfar );

        return( result );
    }

    void getNearFar( double& znear, double& zfar ) const
    {
        // Get the current thread ID for our map index. Use 0 if unavailable.
        OpenThreads::Thread* thread( OpenThreads::Thread::CurrentThread() );
        int id( 0 );
        if( thread != NULL )
            id = thread->getThreadId();

        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _mutex );
        ValueMap::const_iterator iter( _values.find( id ) );
        if( iter == _values.end() )
        {
            // Can't find the current thread ID. This might be a bug (wrong threading model),
            // or it might not (sub Camera's children might require paging and haven't
            // been loaded yet, in which case the sub Camera was culled).
            znear = zfar = 0.;
            return;
        }
        znear = iter->second.first;
        zfar = iter->second.second;
    }

protected:
    typedef std::pair< double, double > DoublePair;
    typedef std::map< int, DoublePair > ValueMap;
    mutable ValueMap _values;
    mutable OpenThreads::Mutex _mutex;
};


/** \brief Clamp projection matrix around two Cameras.
\details This class gets the near & far values from SubCameraClampCB.
It uses the maximum far value and the minimum near value to clamp its
projection matrix. Then it stores that projection matrix (and its inverse)
in the given StateSet (on the StateSet of the root Camera). */
class RootCameraClampCB : public osg::Camera::ClampProjectionMatrixCallback
{
public:
    RootCameraClampCB( SubCameraClampCB* subCB, osg::StateSet* stateSet )
      : _subCB( subCB ),
        _stateSet( stateSet )
    {
    }
    ~RootCameraClampCB()
    {
    }

    virtual bool clampProjectionMatrixImplementation( osg::Matrixf& projection, double& znear, double& zfar ) const
    {
        // Support float by computing in double precision.
        osg::Matrixd m( projection );
        const bool result = clampProjectionMatrixImplementation( m, znear, zfar );
        projection = m;
        return( result );
    }
    virtual bool clampProjectionMatrixImplementation( osg::Matrixd& projection, double& znear, double& zfar ) const
    {
        // Get the subordinate Camera's near and far values. getNearFar() uses
        // the current thread ID as an index to retrieve those values, which
        // will different between contexts in a tiled rendering situation.
        double zn, zf;
        _subCB->getNearFar( zn, zf );
        if( ( zn != 0. ) || ( zf != 0. ) )
        {
            // SubCameraClampCB returns near==0. and far=0. if it is unable to
            // retrieve the values for any reason, so we only use the returned
            // values if at least one of them is not 0.

            // Minimum near and maximum far will encompass both Camera's scenes.
            znear = osg::minimum< double >( zn, znear );
            zfar = osg::maximum< double >( zf, zfar );
        }

        // Clamp as usual.
        const bool result = osgwTools::clampProjection( projection, znear, zfar );

        // Set uniforms. Note that the shader uniforms are floats.
        const osg::Matrixf projf( projection );
        const osg::Matrixf projInv( osg::Matrixf::inverse( projf ) );
        _stateSet->getOrCreateUniform( "osgw_ProjectionMatrix", osg::Uniform::FLOAT_MAT4 )->set( projf );
        _stateSet->getOrCreateUniform( "osgw_ProjectionMatrixInverse", osg::Uniform::FLOAT_MAT4 )->set( projInv );

        return( result );
    }

protected:
    osg::ref_ptr< SubCameraClampCB > _subCB;
    osg::ref_ptr< osg::StateSet > _stateSet;
};




void MultiCameraProjectionMatrix::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _mutex );

    NVMap::iterator iter = _nvs.find( nv );
    if( iter == _nvs.end() )
    {
        // First time we've seen this NodeVisitor (CullVisitor, actually, as
        // we are a cull callback). Save it in the set so we don't do extra
        // work in the future.
        _nvs.insert( nv );

        // 'node' is the subordinate Camera. If it doesn't already have a
        // SubCameraClampCB, create one and add it. This should only occur
        // the first time through this cade; there is only one subordinate
        // Camera and it has only one SubCameraClampCB clamp projection callback.
        osg::Camera* cam( static_cast< osg::Camera* >( node ) );
        osg::CullSettings::ClampProjectionMatrixCallback* clampCB(
            cam->getClampProjectionMatrixCallback() );
        SubCameraClampCB* subClamp( dynamic_cast< SubCameraClampCB* >( clampCB ) );
        if( subClamp == NULL )
        {
            subClamp = new SubCameraClampCB();
            cam->setClampProjectionMatrixCallback( subClamp );
        }

        // Add a RootCameraClampCB to the root Camera to compute the
        // projection matrix uniform that encompasses both Camera's scenes.
        osgUtil::CullVisitor* cv( static_cast< osgUtil::CullVisitor* >( nv ) );
        osg::Camera* rootCam( cv->getRenderStage()->getCamera() );
        if( rootCam->getClampProjectionMatrixCallback() == NULL )
        {
            osg::StateSet* stateSet( node->getOrCreateStateSet() );
            RootCameraClampCB* rootClamp( new RootCameraClampCB( subClamp, stateSet ) );
            rootCam->setClampProjectionMatrixCallback( rootClamp );
        }
    }

    traverse( node, nv );
}



// namespace osgwTools
}
