/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <osgOcean/GodRays>
#include <osgOcean/ShaderManager>

using namespace osgOcean;

GodRays::GodRays(void)
    :_isDirty         (true)
    ,_isStateDirty    (true)
    ,_numOfRays       (10)
    ,_sunDirection    (0.f,0.f,-1.f)
    ,_extinction      (0.1f,0.1f,0.1f)
    ,_baseWaterHeight (0.f)
{
    setUserData( new GodRayDataType(*this) );
    setUpdateCallback( new GodRayAnimationCallback );
    setCullCallback( new GodRayAnimationCallback );
    setCullingActive(false);
}

GodRays::GodRays(unsigned int numOfRays, const osg::Vec3f& sunDir, float baseWaterHeight )
    :_isDirty         (true)
    ,_isStateDirty    (true)
    ,_numOfRays       (numOfRays)
    ,_sunDirection    (sunDir)
    ,_extinction      (0.1f,0.1f,0.1f)
    ,_baseWaterHeight (baseWaterHeight)
{
    setUserData( new GodRayDataType(*this) );
    setUpdateCallback( new GodRayAnimationCallback );
    setCullCallback( new GodRayAnimationCallback );
    setCullingActive(false);
}

GodRays::GodRays(const GodRays& copy, const osg::CopyOp& copyop)
    :osg::Geode       (copy,copyop)
    ,_isDirty         (copy._isDirty)
    ,_isStateDirty    (copy._isStateDirty)
    ,_numOfRays       (copy._numOfRays)
    ,_sunDirection    (copy._sunDirection)
    ,_extinction      (copy._extinction)
    ,_baseWaterHeight (copy._baseWaterHeight)
    ,_stateSet        (copy._stateSet)
    ,_constants       (copy._constants)
    ,_trochoids       (copy._trochoids)
{
}


void GodRays::build(void)
{
    removeDrawables( 0, getNumDrawables() );

    osg::ref_ptr<osg::Geometry> shafts = createRayShafts();

    addDrawable( shafts.get() );

    osg::ref_ptr<osg::Geometry> glare = createGlareQuad();

    if( glare.valid() )
        addDrawable( glare.get() );

    _isDirty = false;
}

void GodRays::buildStateSet(void)
{
    _constants = new osg::FloatArray();

    // reset, create and pack trochoids
    _trochoids = WaterTrochoids(0.05f, 0.25f, 18.f, 1.2f, 1.f, 0.2f );
    _trochoids.createWaves();
    _trochoids.packWaves( _constants.get() );

    _stateSet = new osg::StateSet;

    osg::BlendFunc *blend = new osg::BlendFunc;
    blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE);

    osg::Uniform* waveUniform = new osg::Uniform( osg::Uniform::FLOAT, "osgOcean_Waves", (int)_constants->size() );
    waveUniform->setArray( _constants.get() );

    _stateSet->addUniform( new osg::Uniform( "osgOcean_Origin",       osg::Vec3() ) );
    _stateSet->addUniform( new osg::Uniform( "osgOcean_Extinction_c", _extinction ) );
    _stateSet->addUniform( new osg::Uniform( "osgOcean_Eye",          osg::Vec3() ) );
    _stateSet->addUniform( new osg::Uniform( "osgOcean_Spacing",      1.f ) );
    _stateSet->addUniform( new osg::Uniform( "osgOcean_SunDir",       _sunDirection ) );    
    
    _stateSet->addUniform( waveUniform );

    _stateSet->setAttributeAndModes( blend, osg::StateAttribute::ON );
    _stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );
    _stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    setStateSet( _stateSet.get() );

    _isStateDirty=false;
}

osg::Geometry* GodRays::createRayShafts(void)
{
    osg::Geometry* geom = new osg::Geometry();

    int gridSize = _numOfRays;

    osg::Vec3Array* vertices = new osg::Vec3Array(gridSize*gridSize*2);
    osg::Vec2Array* texcoords = new osg::Vec2Array(gridSize*gridSize*2);

    int rowLen = gridSize*2;
    float disp = ((float)gridSize-1.f)/2.f;

    // The two sets of vertices are set side to side.
    // columns 0-9 upper set
    // columns 10-19 lower set
    // Assign length of ray to the tex coord for use in the vertex shader.
    for(int r = 0; r < gridSize; r++)
    {
        for(int c = 0; c < gridSize; c++)
        {
            float pos_x = (float)c-disp;
            float pos_y = (float)r-disp;

            int i_0 = idx(c,r,rowLen);

            (*vertices) [i_0] = osg::Vec3( pos_x, pos_y, 0.f );
            (*texcoords)[i_0] = osg::Vec2( 0.f, 0.f );

            int i_1 = idx(c+gridSize,r,rowLen);

            (*vertices) [i_1] = osg::Vec3( pos_x, pos_y, 0.f );
            (*texcoords)[i_1] = osg::Vec2( 40.f, 40.f );
        }
    }

    geom->setVertexArray( vertices );
    geom->setTexCoordArray(0, texcoords);

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back( osg::Vec4(1.0,1.0,1.0,1.0) );
    geom->setColorArray( colors );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    for(int r = 0; r < gridSize-1; r+=2)
    {
        for(int c = 0; c < gridSize-1; c+=2)
        {
            osg::DrawElementsUInt* shaft = 
                new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);

            shaft->push_back( idx(c,            r+1, rowLen) );
            shaft->push_back( idx(c+gridSize,   r+1, rowLen) );

            shaft->push_back( idx(c+1,          r+1, rowLen) );
            shaft->push_back( idx(c+gridSize+1, r+1, rowLen) );

            shaft->push_back( idx(c+1,          r,   rowLen) );
            shaft->push_back( idx(c+gridSize+1, r,   rowLen) );

            geom->addPrimitiveSet( shaft );
        }
    }

    osg::StateSet* ss = new osg::StateSet;
    
    osg::ref_ptr<osg::Program> program = createGodRayProgram();

    if( program.valid() )
        ss->setAttributeAndModes( program.get(), osg::StateAttribute::ON );

    // set bounding box as the vertices are displaced in the vertex shader
    // HACK: bounds are set ridiculously big, but could still get culled
    // when we don't want it to.
    osg::BoundingBox box(-2000.f, -2000.f, -2000.f, 2000.f, 2000.f, 0.f);
    geom->setInitialBound(box);
    geom->setComputeBoundingBoxCallback( new ComputeBoundsCallback(*this) );
            
    geom->setStateSet(ss);

    return geom;
}

osg::Geometry* GodRays::createGlareQuad(void)
{
    osg::ref_ptr<osg::Image> glareImage = osgDB::readImageFile("sun_glare.png");

    if( !glareImage.valid() )
        return NULL;

    osg::Texture2D* glareTexture = new osg::Texture2D(glareImage.get());
    glareTexture->setInternalFormat(GL_RGB);
    glareTexture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    glareTexture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    glareTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP );
    glareTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP );

    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* vertices = new osg::Vec3Array;

    float size = 15.f;

    vertices->push_back( osg::Vec3f(-15.f, -15.f, 0.f) );
    vertices->push_back( osg::Vec3f(-15.f,  15.f, 0.f) );
    vertices->push_back( osg::Vec3f( 15.f,  15.f, 0.f) );
    vertices->push_back( osg::Vec3f( 15.f, -15.f, 0.f) );

    osg::Vec2Array* texCoords = new osg::Vec2Array;

    texCoords->push_back( osg::Vec2f(0.f,0.f) );
    texCoords->push_back( osg::Vec2f(0.f,1.f) );
    texCoords->push_back( osg::Vec2f(1.f,1.f) );
    texCoords->push_back( osg::Vec2f(1.f,0.f) );

    osg::Vec3Array* normals = new osg::Vec3Array;

    normals->push_back( osg::Vec3f(0.f, 0.f, -1.f) );

    osg::Vec4Array* colors = new osg::Vec4Array;

    colors->push_back( osg::Vec4f(1.f,1.f,1.f,1.f) );

    osg::DrawElementsUInt* prim = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS);

    prim->push_back(0);
    prim->push_back(1);
    prim->push_back(2);
    prim->push_back(3);

    osg::StateSet* ss = new osg::StateSet;
    ss->setTextureAttributeAndModes( 0, glareTexture );
    ss->addUniform( new osg::Uniform( "osgOcean_GlareTexture",   0) );
    
    osg::ref_ptr<osg::Program> program = createGodRayGlareProgram();

    if( program.valid() )
        ss->setAttributeAndModes( program.get(), osg::StateAttribute::ON );

    // set bounding box as the vertices are displaced in the vertex shader
    // HACK: bounds are set ridiculously big, but could still get culled
    // when we don't want it to.
    osg::BoundingBox box(-2000.f, -2000.f, -30.f, 2000.f, 2000.f, 0.f);
    geom->setInitialBound(box);
    geom->setComputeBoundingBoxCallback( new ComputeBoundsCallback(*this) );

    geom->setVertexArray(vertices);
    geom->setTexCoordArray(0,texCoords);
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(prim);
    
    geom->setStateSet(ss);

    return geom;
}

void GodRays::update(float time, const osg::Vec3f& eye, const double& fov)
{
    if(_isDirty)
        build();

    if(_isStateDirty)
        buildStateSet();

    // if eye is below water surface do the updates
    if( eye.z() < _baseWaterHeight )
    {
        float tanFOVOver2 = tan( osg::inDegrees( fov / 2.f ) );
        float depth = -eye.z() * 2.f;

        float spacing = 0.2 * ( ( depth * tanFOVOver2 ) / (float)_numOfRays );

        osg::Vec3f refracted = refract( 0.75f /* 1/1.333 */, _sunDirection, osg::Vec3(0.f, 0.f, 1.f) );
        refracted.normalize();
        osg::Vec3f sunPos = eye + refracted * ( _baseWaterHeight-eye.z() ) / refracted.z();

        _stateSet->getUniform("osgOcean_Eye")->set(eye);
        _stateSet->getUniform("osgOcean_Spacing")->set(spacing);
        _stateSet->getUniform("osgOcean_Origin")->set(sunPos);

        _trochoids.updateWaves( time/2.0 );
        _trochoids.packWaves( _constants.get() );

        _stateSet->getUniform("osgOcean_Waves")->setArray( _constants.get() );

        // If the eye isn't contained withing the god ray volume, 
        // we need to recompute the bounds or they get clipped.
        if(!getDrawable(0)->getBound().contains( eye )){
            getDrawable(0)->dirtyBound();
            getDrawable(1)->dirtyBound();
        }
    }
}

osg::Vec3f GodRays::refract( const float ratio, const osg::Vec3f& I, const osg::Vec3f& N )
{
    float n = ratio;
    float n_2 = n*n;

    osg::Vec3f nI = I * n;

    float nIN = nI*N;

    float IN_2 = I*N;
    IN_2 = IN_2 * IN_2;

    return ( N * ( -nIN - sqrt( 1.f - ( n_2*(1.f-IN_2) )  ) ) ) + nI;
}

#include <osgOcean/shaders/osgOcean_godrays_vert.inl>
#include <osgOcean/shaders/osgOcean_godrays_frag.inl>

osg::Program* GodRays::createGodRayProgram( void )
{
	static const char osgOcean_godrays_vert_file[] = "osgOcean_godrays.vert";
	static const char osgOcean_godrays_frag_file[] = "osgOcean_godrays.frag";

	return ShaderManager::instance().createProgram("godrays_shader", 
                                                   osgOcean_godrays_vert_file, osgOcean_godrays_frag_file, 
                                                   osgOcean_godrays_vert,      osgOcean_godrays_frag);
}

#include <osgOcean/shaders/osgOcean_godray_glare_vert.inl>
#include <osgOcean/shaders/osgOcean_godray_glare_frag.inl>

osg::Program* GodRays::createGodRayGlareProgram( void )
{
	static const char osgOcean_godray_glare_vert_file[] = "osgOcean_godray_glare.vert";
	static const char osgOcean_godray_glare_frag_file[] = "osgOcean_godray_glare.frag";

	return ShaderManager::instance().createProgram("godray_glare", 
                                                   osgOcean_godray_glare_vert_file, osgOcean_godray_glare_frag_file,
                                                   osgOcean_godray_glare_vert,      osgOcean_godray_glare_frag);
}

// --------------------------------------------
//          Callback implementations
// --------------------------------------------

GodRays::GodRayDataType::GodRayDataType(GodRays& godRays)
    :_godRays( godRays )
    ,_fov    ( 0.0 )
{}

GodRays::GodRayDataType::GodRayDataType( const GodRayDataType& copy, const osg::CopyOp& copyop )
    :_godRays (copy._godRays)
    ,_eye     (copy._eye)
    ,_fov     (copy._fov)
{}

void GodRays::GodRayDataType::update( float time )
{
    _godRays.update(time, _eye, _fov);
}

void GodRays::GodRayAnimationCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::ref_ptr<GodRayDataType> data = dynamic_cast<GodRayDataType*> ( node->getUserData() );

    if(data.valid())
    {
        // If cull visitor update the current eye position
        if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osg::Vec3f eye, centre, up;
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            
            cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye,centre,up);
            data->setEye( eye );

            double fov,aspectRatio,near,far;
            cv->getRenderStage()->getCamera()->getProjectionMatrixAsPerspective(fov,aspectRatio,near,far);
            data->setFOV( fov );
        }
        else if( nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR ){
            data->update( nv->getFrameStamp()->getSimulationTime() );
        }
    }

    traverse(node, nv); 
}

GodRays::ComputeBoundsCallback::ComputeBoundsCallback( GodRays& rays )
    :_rays(rays)
{}

osg::BoundingBox GodRays::ComputeBoundsCallback::computeBound(const osg::Drawable& draw) const
{
    GodRays::GodRayDataType* data = static_cast<GodRays::GodRayDataType*> (_rays.getUserData() );

    osg::BoundingBox bb = draw.getInitialBound();

    bb.xMin() += data->getEye().x();
    bb.xMax() += data->getEye().x();

    bb.yMin() += data->getEye().y();
    bb.yMax() += data->getEye().y();

    return bb;
}
