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

#include <osgOcean/OceanScene>
#include <osgOcean/ShaderManager>
#include <osgOcean/FFTOceanTechnique>

#include <osg/Depth>

using namespace osgOcean;

namespace
{
    // CameraTrackCallback used by osgOcean with the undersea cylinder.
    // Note: only set on MatrixTransform.
    class CameraTrackCallback: public osg::NodeCallback
    {
    public:
        CameraTrackCallback(OceanScene* oceanScene)
            : _oceanScene (oceanScene)
            , _currentMatrix(0)
            , _traversalNumber(-1)
        {
        }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
            {
                osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);

                if (nv->getTraversalNumber() > _traversalNumber)
                {
                    // Rendering new frame, reuse matrices used in the last frame.
                    _currentMatrix = 0;
                }

                osg::MatrixTransform* mt = static_cast<osg::MatrixTransform*>(node);
                bool follow = true;

                osgOcean::FFTOceanTechnique* oceanTechnique = dynamic_cast<osgOcean::FFTOceanTechnique*>(_oceanScene->getOceanTechnique());
                if (oceanTechnique && !oceanTechnique->isEndlessOceanEnabled())
                {
                    // Do not follow the eye when the ocean is constrained 
                    // to an area.
                    follow = false;

                    // The ocean cylinder should not be visible if the ocean 
                    // is constrained to an area, since the ocean will often 
                    // be in a pool or a river which already has walls.
                    mt->getChild(0)->setNodeMask(0);
                }

                if (follow)
                {
                    osg::Vec3f centre,up,eye;
                    // get MAIN camera eye,centre,up
                    cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye,centre,up);

                    bool eyeAboveWater = _oceanScene->isEyeAboveWater(eye);
                    float mult = 1.0;
                    if (eyeAboveWater) mult = -1.0;

                    // Translate the ocean cylinder down by the surface height 
                    // if the eye went from below to above the surface (so the 
                    // cylinder doesn't peek through the waves) and inversely
                    // when the eye goes from above to below the surface (so
                    // we don't see cracks between the cylinder's edge and the
                    // waves).

                    // Note, we set the ocean cylinder's own matrixTransform 
                    // to identity, and push the relevant matrix onto the 
                    // modelView matrix stack, because it is viewpoint 
                    // dependent.
                    mt->setMatrix(osg::Matrix::identity());

                    osg::Camera* currentCamera = cv->getCurrentRenderBin()->getStage()->getCamera();

                    double z = -_oceanScene->getOceanCylinder()->getHeight() +                      // So the cylinder is underwater
                                _oceanScene->getOceanSurfaceHeight() +                              // Follow the ocean surface's height
                                mult * _oceanScene->getOceanTechnique()->getMaximumHeight();        // Offset either up or down by a bit.
                    osg::RefMatrix* cylinderMatrix = createOrReuseMatrix(osg::Matrix::translate(eye.x(), eye.y(), z) * currentCamera->getViewMatrix());
                    cv->pushModelViewMatrix(cylinderMatrix, osg::Transform::ABSOLUTE_RF);
                }

                traverse(node, nv);

                if (follow)
                {
                    cv->popModelViewMatrix();
                }
            }
            else
            {
                traverse(node, nv); 
            }
        }

        // See osg::CullStack::createOrReuseMatrix()
        osg::RefMatrix* createOrReuseMatrix(const osg::Matrix& value)
        {
            // skip of any already reused matrix.
            while (_currentMatrix < _matrices.size() && 
                   _matrices[_currentMatrix]->referenceCount()>1)
            {
                ++_currentMatrix;
            }

            // if still within list, element must be singularly referenced
            // there return it to be reused.
            if (_currentMatrix < _matrices.size())
            {
                osg::RefMatrix* matrix = _matrices[_currentMatrix++].get();
                matrix->set(value);
                return matrix;
            }

            // otherwise need to create new matrix.
            osg::RefMatrix* matrix = new osg::RefMatrix(value);
            _matrices.push_back(matrix);
            ++_currentMatrix;
            return matrix;
        }

        osgOcean::OceanScene* _oceanScene;
        std::vector< osg::ref_ptr<osg::RefMatrix> > _matrices;
        unsigned int _currentMatrix;

        int _traversalNumber;
    };

    static const float OCEAN_CYLINDER_HEIGHT = 4000.f;

}

OceanScene::OceanScene( void )
    :_oceanSurface               ( 0 )
    ,_isDirty                    ( true )
    ,_enableReflections          ( false )
    ,_enableRefractions          ( false )
    ,_enableHeightmap            ( false )
    ,_enableGodRays              ( false )
    ,_enableSilt                 ( false )
    ,_enableDOF                  ( false )
    ,_enableGlare                ( false )
    ,_enableDistortion           ( false )
    ,_enableUnderwaterScattering ( false )
    ,_enableDefaultShader        ( true )
    ,_reflectionTexSize          ( 512,512 )
    ,_refractionTexSize          ( 512,512 )
    ,_screenDims                 ( 1920,1080 )
    ,_sunDirection               ( 0,0,-1 )
    ,_reflectionUnit             ( 1 )
    ,_refractionUnit             ( 2 )
    ,_refractionDepthUnit        ( 3 )
    ,_heightmapUnit              ( 7 )
    ,_reflectionSceneMask        ( 0x1 )  // 1
    ,_refractionSceneMask        ( 0x2 )  // 2
    ,_normalSceneMask            ( 0x4 )  // 4
    ,_surfaceMask                ( 0x8 )  // 8
    ,_siltMask                   ( 0x10 ) // 16
    ,_heightmapMask              ( 0x20 ) // 32
    ,_ARMask                     ( 0x40 ) // 64
    ,_lightID                    ( 0 )
    ,_dofNear                    ( 0.f )
    ,_dofFar                     ( 160.f )
    ,_dofFocus                   ( 30.f )
    ,_dofFarClamp                ( 1.f )
    ,_glareThreshold             ( 0.9f )
    ,_glareAttenuation           ( 0.75f )
    ,_underwaterFogColor         ( 0.2274509f, 0.4352941f, 0.7294117f, 1.f )
    ,_underwaterAttenuation      ( 0.015f, 0.0075f, 0.005f)
    ,_underwaterFogDensity       ( 0.01f )
    ,_aboveWaterFogDensity       ( 0.0012f )
    ,_eyeHeightReflectionCutoff  ( FLT_MAX )
    ,_eyeHeightRefractionCutoff  (-FLT_MAX )
    ,_surfaceHeight              ( 0.0f )
    ,_oceanTransform             ( new osg::MatrixTransform )
    ,_oceanCylinder              ( new Cylinder(1900.f, OCEAN_CYLINDER_HEIGHT, 16, false, true) )
    ,_oceanCylinderMT            ( new osg::MatrixTransform )
{
    //-----------------------------------------------------------------------
    // _oceanCylinder follows the camera underwater, so that the clear
    // color is not visible past the far plane - it will be the fog color.
    _oceanCylinder->setColor( _underwaterFogColor );
    _oceanCylinder->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _oceanCylinder->getOrCreateStateSet()->setMode(GL_FOG, osg::StateAttribute::OFF);

    osg::Geode* cylinderGeode = new osg::Geode;
    cylinderGeode->addDrawable( _oceanCylinder.get() );

    _oceanCylinderMT->setMatrix( osg::Matrix::translate(0, 0, -OCEAN_CYLINDER_HEIGHT) );
    _oceanCylinderMT->setDataVariance( osg::Object::DYNAMIC ),
    _oceanCylinderMT->setCullCallback( new CameraTrackCallback(this) );
    _oceanCylinderMT->setNodeMask( getNormalSceneMask() | getRefractedSceneMask() );
    _oceanCylinderMT->addChild( cylinderGeode );

    _oceanTransform->addChild( _oceanCylinderMT.get() );
    //-----------------------------------------------------------------------

    _oceanTransform->setNodeMask( _normalSceneMask | _surfaceMask );
    addChild( _oceanTransform.get() );

    addResourcePaths();

    setNumChildrenRequiringUpdateTraversal(1);
    
    _defaultSceneShader = createDefaultSceneShader();
    ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", _lightID);
}

OceanScene::OceanScene( OceanTechnique* technique )
    :_oceanSurface               ( technique )
    ,_isDirty                    ( true )
    ,_enableReflections          ( false )
    ,_enableRefractions          ( false )
    ,_enableHeightmap            ( false )
    ,_enableGodRays              ( false )
    ,_enableSilt                 ( false )
    ,_enableDOF                  ( false )
    ,_enableGlare                ( false )
    ,_enableDistortion           ( false )
    ,_enableUnderwaterScattering ( false )
    ,_enableDefaultShader        ( true )
    ,_reflectionTexSize          ( 512,512 )
    ,_refractionTexSize          ( 512,512 )
    ,_screenDims                 ( 1920, 1080 )
    ,_sunDirection               ( 0,0,-1 )
    ,_reflectionUnit             ( 1 )
    ,_refractionUnit             ( 2 )
    ,_refractionDepthUnit        ( 3 )
    ,_heightmapUnit              ( 7 )
    ,_reflectionSceneMask        ( 0x1 )
    ,_refractionSceneMask        ( 0x2 )
    ,_normalSceneMask            ( 0x4 )
    ,_surfaceMask                ( 0x8 )
    ,_siltMask                   ( 0x10 )
    ,_heightmapMask              ( 0x20 ) 
    ,_ARMask                     ( 0x40 ) // 64
    ,_lightID                    ( 0 )
    ,_dofNear                    ( 0.f )
    ,_dofFar                     ( 160.f )
    ,_dofFocus                   ( 30.f )
    ,_dofFarClamp                ( 1.f )
    ,_glareThreshold             ( 0.9f )
    ,_glareAttenuation           ( 0.75f )
    ,_underwaterFogColor         ( 0.2274509f, 0.4352941f, 0.7294117f, 1.f )
    ,_underwaterAttenuation      ( 0.015f, 0.0075f, 0.005f)
    ,_underwaterFogDensity       ( 0.01f )
    ,_aboveWaterFogDensity       ( 0.0012f )
    ,_eyeHeightReflectionCutoff  ( FLT_MAX)
    ,_eyeHeightRefractionCutoff  (-FLT_MAX)
    ,_surfaceHeight              ( 0.0f )
    ,_oceanTransform             ( new osg::MatrixTransform )
    ,_oceanCylinder              ( new Cylinder(1900.f, OCEAN_CYLINDER_HEIGHT, 16, false, true) )
    ,_oceanCylinderMT            ( new osg::MatrixTransform )
{
    //-----------------------------------------------------------------------
    // _oceanCylinder follows the camera underwater, so that the clear
    // color is not visible past the far plane - it will be the fog color.
    _oceanCylinder->setColor( _underwaterFogColor );
    _oceanCylinder->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _oceanCylinder->getOrCreateStateSet()->setMode(GL_FOG, osg::StateAttribute::OFF);

    osg::Geode* cylinderGeode = new osg::Geode;
    cylinderGeode->addDrawable( _oceanCylinder.get() );

    _oceanCylinderMT->setMatrix( osg::Matrix::translate(0, 0, -OCEAN_CYLINDER_HEIGHT) );
    _oceanCylinderMT->setDataVariance( osg::Object::DYNAMIC ),
    _oceanCylinderMT->setCullCallback( new CameraTrackCallback(this) );
    _oceanCylinderMT->setNodeMask( getNormalSceneMask() | getRefractedSceneMask() );
    _oceanCylinderMT->addChild( cylinderGeode );

    _oceanTransform->addChild( _oceanCylinderMT.get() );
    //-----------------------------------------------------------------------

    _oceanTransform->setNodeMask( _normalSceneMask | _surfaceMask );
    addChild( _oceanTransform.get() );

    _oceanSurface->setNodeMask( _surfaceMask );
    _oceanTransform->addChild( _oceanSurface.get() );

    addResourcePaths();

    setNumChildrenRequiringUpdateTraversal(1);

    ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", _lightID);
    _defaultSceneShader = createDefaultSceneShader();
}

OceanScene::OceanScene( const OceanScene& copy, const osg::CopyOp& copyop )
    :osg::Group                  ( copy, copyop )
    ,_oceanSurface               ( copy._oceanSurface )
    ,_isDirty                    ( copy._isDirty )
    ,_enableReflections          ( copy._enableReflections )
    ,_enableRefractions          ( copy._enableRefractions )
    ,_enableGodRays              ( copy._enableGodRays )
    ,_enableSilt                 ( copy._enableSilt )
    ,_enableDOF                  ( copy._enableDOF )
    ,_enableGlare                ( copy._enableGlare )
    ,_enableDistortion           ( copy._enableDistortion )
    ,_enableUnderwaterScattering ( copy._enableUnderwaterScattering )
    ,_enableDefaultShader        ( copy._enableDefaultShader )
    ,_reflectionTexSize          ( copy._reflectionTexSize )
    ,_refractionTexSize          ( copy._refractionTexSize )
    ,_screenDims                 ( copy._screenDims )
    ,_reflectionUnit             ( copy._reflectionUnit )
    ,_refractionUnit             ( copy._refractionUnit )
    ,_refractionDepthUnit        ( copy._refractionDepthUnit )
    ,_heightmapUnit              ( copy._heightmapUnit )
    ,_reflectionSceneMask        ( copy._reflectionSceneMask )
    ,_refractionSceneMask        ( copy._refractionSceneMask )
    ,_siltMask                   ( copy._siltMask )
    ,_surfaceMask                ( copy._surfaceMask )
    ,_normalSceneMask            ( copy._normalSceneMask )
    ,_heightmapMask              ( copy._heightmapMask )
    ,_ARMask                     ( copy._ARMask )
    ,_godrayPreRender            ( copy._godrayPreRender )
    ,_godrayPostRender           ( copy._godrayPostRender )
    ,_godrays                    ( copy._godrays )
    ,_godRayBlendSurface         ( copy._godRayBlendSurface )
    ,_siltClipNode               ( copy._siltClipNode )
    ,_reflectionClipNode         ( copy._reflectionClipNode )
    ,_lightID                    ( copy._lightID )
    ,_underwaterFogDensity       ( copy._underwaterFogDensity )
    ,_aboveWaterFogDensity       ( copy._aboveWaterFogDensity ) 
    ,_underwaterFogColor         ( copy._underwaterFogColor )
    ,_aboveWaterFogColor         ( copy._aboveWaterFogColor )
    ,_underwaterDiffuse          ( copy._underwaterDiffuse )
    ,_underwaterAttenuation      ( copy._underwaterAttenuation )
    ,_sunDirection               ( copy._sunDirection )
    ,_dofNear                    ( copy._dofNear )
    ,_dofFar                     ( copy._dofFar )
    ,_dofFocus                   ( copy._dofFocus )
    ,_dofFarClamp                ( copy._dofFarClamp )
    ,_dofPasses                  ( copy._dofPasses )
    ,_glarePasses                ( copy._glarePasses )
    ,_dofStateSet                ( copy._dofStateSet )
    ,_glareThreshold             ( copy._glareThreshold )
    ,_glareAttenuation           ( copy._glareAttenuation )
    ,_glareStateSet              ( copy._glareStateSet )
    ,_distortionSurface          ( copy._distortionSurface)
    ,_globalStateSet             ( copy._globalStateSet )
    ,_defaultSceneShader         ( copy._defaultSceneShader )
    ,_eyeHeightReflectionCutoff  ( copy._eyeHeightReflectionCutoff )
    ,_eyeHeightRefractionCutoff  ( copy._eyeHeightRefractionCutoff )
    ,_surfaceHeight              ( copy._surfaceHeight )
    ,_oceanTransform             ( copy._oceanTransform )
    ,_oceanCylinder              ( copy._oceanCylinder )
    ,_oceanCylinderMT            ( copy._oceanCylinderMT )
{
}

OceanScene::~OceanScene( void )
{

}

void OceanScene::init( void )
{
    osg::notify(osg::INFO) << "OceanScene::init()" << std::endl;

    _godrayPreRender  = NULL;
    _godrayPostRender = NULL;

    if( _reflectionClipNode.valid() ){
        removeChild( _reflectionClipNode.get() );
        _reflectionClipNode = NULL;
    }

    _dofPasses.clear();
    _dofStateSet = NULL;

    _glarePasses.clear();
    _glareStateSet = NULL;
    
    _distortionSurface = NULL;

    if( _siltClipNode.valid() ){
        removeChild( _siltClipNode.get() );
        _siltClipNode = NULL;
    }

    if( _oceanSurface.valid() )
    {
        const float LOG2E = 1.442695;

        _globalStateSet = new osg::StateSet;

        // This is now a #define, added by the call to 
        // ShaderManager::setGlobalDefinition() in the constructors above. 
        // Note that since _lightID can change, we will need to change the
        // global definition and reload all the shaders that depend on its
        // value when it does. This is not implemented yet.
        //_globalStateSet->addUniform( new osg::Uniform("osgOcean_LightID", _lightID ) );

        _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableDOF", _enableDOF ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableGlare", _enableGlare ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableUnderwaterScattering", _enableUnderwaterScattering ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_WaterHeight", float(getOceanSurfaceHeight()) ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterFogColor", _underwaterFogColor ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_AboveWaterFogColor", _aboveWaterFogColor ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterFogDensity", -_underwaterFogDensity*_underwaterFogDensity*LOG2E ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_AboveWaterFogDensity", -_aboveWaterFogDensity*_aboveWaterFogDensity*LOG2E ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterDiffuse", _underwaterDiffuse ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterAttenuation", _underwaterAttenuation ) );

        if(_enableDefaultShader)
        {
            _globalStateSet->setAttributeAndModes( _defaultSceneShader.get(), osg::StateAttribute::ON );
        }

        if( _enableReflections )
        {
            osg::ClipPlane* reflClipPlane = new osg::ClipPlane();
            reflClipPlane->setClipPlaneNum(0);
            reflClipPlane->setClipPlane( 0.0, 0.0, 1.0, -getOceanSurfaceHeight() );
            _reflectionClipNode = new osg::ClipNode;
            _reflectionClipNode->addClipPlane( reflClipPlane );

            addChild( _reflectionClipNode.get() );
        }

        if( _enableGodRays )
        {
            osg::TextureRectangle* godRayTexture = createTextureRectangle( _screenDims/2, GL_RGB );

            _godrays = new GodRays(10,_sunDirection, getOceanSurfaceHeight() );

            _godrayPreRender=renderToTexturePass( godRayTexture );
            _godrayPreRender->setClearColor( osg::Vec4(0.0745098, 0.10588235, 0.1529411, 1.0) );
            _godrayPreRender->addChild( _godrays.get() );

            _godRayBlendSurface = new GodRayBlendSurface( osg::Vec3f(-1.f,-1.f,-1.f), osg::Vec2f(2.f,2.f), godRayTexture );

            _godRayBlendSurface->setSunDirection(_sunDirection);
            _godRayBlendSurface->setEccentricity(0.3f);
            _godRayBlendSurface->setIntensity(0.1f);

            _godrayPostRender=godrayFinalPass();
            _godrayPostRender->addChild( _godRayBlendSurface.get() );
        }

        if( _enableDOF )
        {
            _dofPasses.clear();

            osg::Vec2s lowResDims = _screenDims/4;

            _dofStateSet = new osg::StateSet;
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Near",  _dofNear ) );
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Far",   _dofFar ) );
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Clamp", _dofFarClamp ) );
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Focus", _dofFocus ) );

            // First capture screen color buffer and a luminance buffer used for a custom depth map
            osg::TextureRectangle* fullScreenTexture   = createTextureRectangle( _screenDims, GL_RGBA );
            osg::TextureRectangle* fullScreenLuminance = createTextureRectangle( _screenDims, GL_LUMINANCE );

            osg::Camera* fullPass = multipleRenderTargetPass( fullScreenTexture,   osg::Camera::COLOR_BUFFER0,
                                                              fullScreenLuminance, osg::Camera::COLOR_BUFFER1 );

            fullPass->setCullCallback( new PrerenderCameraCullCallback(this) );
            fullPass->setStateSet(_dofStateSet.get());
            _dofPasses.push_back( fullPass );

            // Downsize image
            osg::TextureRectangle* downsizedTexture = createTextureRectangle( lowResDims, GL_RGBA );
            _dofPasses.push_back( downsamplePass( fullScreenTexture, NULL, downsizedTexture, false ) );
            
            // Gaussian blur 1
            osg::TextureRectangle* gaussianTexture_1 = createTextureRectangle( lowResDims, GL_RGBA );
            _dofPasses.push_back( gaussianPass(downsizedTexture, gaussianTexture_1, true ) );

            // Gaussian blur 2
            osg::TextureRectangle* gaussianTexture_2 = createTextureRectangle( lowResDims, GL_RGBA );
            _dofPasses.push_back( gaussianPass(gaussianTexture_1, gaussianTexture_2, false ) );

            // Combiner
            osg::TextureRectangle* combinedTexture = createTextureRectangle( _screenDims, GL_RGBA );
            _dofPasses.push_back( dofCombinerPass(fullScreenTexture, fullScreenLuminance, gaussianTexture_2, combinedTexture ) );

            // Post render pass
            _dofPasses.push_back( dofFinalPass( combinedTexture ) );
        }
    
        if( _enableGlare )
        {
            _glarePasses.clear();

            osg::Vec2s lowResDims = _screenDims/4;

            _glareStateSet = new osg::StateSet;
            _glareStateSet->addUniform( new osg::Uniform("osgOcean_EnableGlare", _enableGlare ) );

            // First capture screen
            osg::TextureRectangle* fullScreenTexture = createTextureRectangle( _screenDims, GL_RGBA );
            osg::TextureRectangle* luminanceTexture  = createTextureRectangle( _screenDims, GL_LUMINANCE );
            
            osg::Camera* fullPass = multipleRenderTargetPass( 
                fullScreenTexture, osg::Camera::COLOR_BUFFER0,
                luminanceTexture,  osg::Camera::COLOR_BUFFER1 );

            fullPass->setCullCallback( new PrerenderCameraCullCallback(this) );
            fullPass->setStateSet(_glareStateSet.get());
            _glarePasses.push_back( fullPass );

            // Downsize image
            osg::TextureRectangle* downsizedTexture = createTextureRectangle( lowResDims, GL_RGBA );
            _glarePasses.push_back( downsamplePass( fullScreenTexture, luminanceTexture, downsizedTexture, true ) );

            // Streak filter top Right
            osg::TextureRectangle* streakBuffer1 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer1, 1, osg::Vec2f(0.5f,0.5f) ) );

            osg::TextureRectangle* streakBuffer2 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer1,streakBuffer2, 2, osg::Vec2f(0.5f,0.5f) ) );

            // Streak filter Bottom left
            osg::TextureRectangle* streakBuffer3 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer3, 1, osg::Vec2f(-0.5f,-0.5f) ) );

            osg::TextureRectangle* streakBuffer4 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer3,streakBuffer4, 2, osg::Vec2f(-0.5f,-0.5f) ) );

            // Streak filter Bottom right
            osg::TextureRectangle* streakBuffer5 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer5, 1, osg::Vec2f(0.5f,-0.5f) ) );

            osg::TextureRectangle* streakBuffer6 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer5,streakBuffer6, 2, osg::Vec2f(0.5f,-0.5f) ) );

            // Streak filter Top Left
            osg::TextureRectangle* streakBuffer7 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer7,1, osg::Vec2f(-0.5f,0.5f) ) );

            osg::TextureRectangle* streakBuffer8 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer7,streakBuffer8, 2, osg::Vec2f(-0.5f,0.5f) ) );

            // Final pass - combine glare and blend into scene.
            _glarePasses.push_back( glareCombinerPass(fullScreenTexture, streakBuffer2, streakBuffer4, streakBuffer6, streakBuffer8 ) );
        }

        if( _enableSilt )
        {
            SiltEffect* silt = new SiltEffect;
            // Clip silt above water level
            silt->getOrCreateStateSet()->setMode( GL_CLIP_PLANE0+1, osg::StateAttribute::ON );
            silt->setIntensity(0.07f);
            silt->setParticleSpeed(0.025f);
            silt->setNodeMask(_siltMask);

            osg::ClipPlane* siltClipPlane = new osg::ClipPlane();
            siltClipPlane->setClipPlaneNum(1);
            siltClipPlane->setClipPlane( 0.0, 0.0, -1.0, -getOceanSurfaceHeight() );

            _siltClipNode = new osg::ClipNode;
            _siltClipNode->addClipPlane( siltClipPlane );
            _siltClipNode->addChild( silt );

            addChild( _siltClipNode.get() );
        }
    }
    
    _isDirty = false;
}

OceanScene::ViewData* OceanScene::getViewDependentData( osgUtil::CullVisitor * cv )
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_viewDataMapMutex);
    return _viewDataMap[ cv ].get();
}

void OceanScene::setViewDependentData( osgUtil::CullVisitor * cv, ViewData * data )
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_viewDataMapMutex);
    _viewDataMap[ cv ] = data;
}

OceanScene::ViewData * OceanScene::initViewDependentData( osgUtil::CullVisitor *cv, OceanScene::ViewData * vd )
{
    ViewData* viewData = vd;
    if (!viewData) viewData = new ViewData;
    viewData->init( this, cv );
    return viewData;
}

void OceanScene::ViewData::dirty( bool flag )
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
    _dirty = flag;
}

#include <osgOcean/shaders/osgOcean_heightmap_vert.inl>
#include <osgOcean/shaders/osgOcean_heightmap_frag.inl>

void OceanScene::ViewData::init( OceanScene *oceanScene, osgUtil::CullVisitor * cv )
{
    _cv = cv;
    _oceanScene = oceanScene;

    _globalStateSet = new osg::StateSet;
    _surfaceStateSet = new osg::StateSet;

    _globalStateSet->addUniform( new osg::Uniform("osgOcean_EyeUnderwater", false ) );
    _globalStateSet->addUniform( new osg::Uniform("osgOcean_Eye", osg::Vec3f() ) );

    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableReflections",  _oceanScene->_enableReflections ) );
    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_ReflectionMap",      _oceanScene->_reflectionUnit ) );

    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableRefractions",  _oceanScene->_enableRefractions ) );
    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_RefractionMap",      _oceanScene->_refractionUnit ) );
    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_RefractionDepthMap", _oceanScene->_refractionDepthUnit ) );

    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableHeightmap",    _oceanScene->_enableHeightmap ) );
    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_Heightmap",          _oceanScene->_heightmapUnit ) );

    _surfaceStateSet->addUniform( new osg::Uniform(osg::Uniform::FLOAT_MAT4, "osgOcean_RefractionInverseTransformation") );
    _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_ViewportDimensions", osg::Vec2(_oceanScene->_screenDims.x(), _oceanScene->_screenDims.y()) ) );

    _fog = new osg::Fog;
    _fog->setMode(osg::Fog::EXP2);
    _fog->setDensity(_oceanScene->_aboveWaterFogDensity);
    _fog->setColor(_oceanScene->_aboveWaterFogColor);
    _globalStateSet->setAttributeAndModes(_fog.get(), osg::StateAttribute::ON);

    if( _oceanScene->_enableReflections )
    {
        // Update the reflection matrix's translation to take into account
        // the ocean surface height. The translation we need is 2*h.
        // See http://www.gamedev.net/columns/hardcore/rnerwater1/page3.asp
        _reflectionMatrix = osg::Matrixf(  1,  0,  0,  0,
                                           0,  1,  0,  0,
                                           0,  0, -1,  0,    
                                           0,  0,  2 * _oceanScene->getOceanSurfaceHeight(),  1 );

        osg::ref_ptr<osg::Texture2D> reflectionTexture = _oceanScene->createTexture2D( _oceanScene->_reflectionTexSize, GL_RGBA );
        
        // clip everything below water line
        _reflectionCamera = _oceanScene->renderToTexturePass( reflectionTexture.get() );
        _reflectionCamera->setClearColor( osg::Vec4( 0.0, 0.0, 0.0, 0.0 ) );
        _reflectionCamera->setComputeNearFarMode( osg::Camera::DO_NOT_COMPUTE_NEAR_FAR );
        _reflectionCamera->setCullMask( _oceanScene->_reflectionSceneMask | _oceanScene->_ARMask);
        _reflectionCamera->setCullCallback( new CameraCullCallback(_oceanScene.get()) );
        _reflectionCamera->getOrCreateStateSet()->setMode( GL_CLIP_PLANE0+0, osg::StateAttribute::ON );
        _reflectionCamera->getOrCreateStateSet()->setMode( GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

        _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_reflectionUnit, reflectionTexture.get(), osg::StateAttribute::ON );
    }

    if( _oceanScene->_enableRefractions )
    {
        osg::Texture2D* refractionTexture = _oceanScene->createTexture2D( _oceanScene->_refractionTexSize, GL_RGBA );
        osg::Texture2D* refractionDepthTexture = _oceanScene->createTexture2D( _oceanScene->_refractionTexSize, GL_DEPTH_COMPONENT );

        refractionTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST );
        refractionTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST );

        _refractionCamera = _oceanScene->multipleRenderTargetPass( 
            refractionTexture, osg::Camera::COLOR_BUFFER, 
            refractionDepthTexture, osg::Camera::DEPTH_BUFFER );

        _refractionCamera->setClearDepth( 1.0 );
        _refractionCamera->setClearColor( osg::Vec4( 0.0, 0.0, 0.0, 0.0 ) );
        _refractionCamera->setComputeNearFarMode( osg::Camera::DO_NOT_COMPUTE_NEAR_FAR );
        _refractionCamera->setCullMask( _oceanScene->_refractionSceneMask | _oceanScene->_ARMask );
        _refractionCamera->setCullCallback( new CameraCullCallback(_oceanScene.get()) );

        _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_refractionUnit, refractionTexture, osg::StateAttribute::ON );
        _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_refractionDepthUnit, refractionDepthTexture, osg::StateAttribute::ON );
    }

    if ( _oceanScene->_enableHeightmap ) 
    {
        osg::Texture2D* heightmapTexture = _oceanScene->createTexture2D( _oceanScene->_refractionTexSize, GL_DEPTH_COMPONENT );

        _heightmapCamera = new osg::Camera;

        _heightmapCamera->setClearMask( GL_DEPTH_BUFFER_BIT );
        _heightmapCamera->setClearDepth( 1.0 );
        _heightmapCamera->setComputeNearFarMode( osg::Camera::DO_NOT_COMPUTE_NEAR_FAR );
        _heightmapCamera->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LESS, 0.0f, 1.0f, true));

        _heightmapCamera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
        _heightmapCamera->setViewport( 0,0, heightmapTexture->getTextureWidth(), heightmapTexture->getTextureHeight() );
        _heightmapCamera->setRenderOrder(osg::Camera::PRE_RENDER);
        _heightmapCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        _heightmapCamera->attach( osg::Camera::DEPTH_BUFFER, heightmapTexture );    

        _heightmapCamera->setCullMask( _oceanScene->_heightmapMask );
        _heightmapCamera->setCullCallback( new CameraCullCallback(_oceanScene.get()) );

        static const char osgOcean_heightmap_vert_file[] = "osgOcean_heightmap.vert";
        static const char osgOcean_heightmap_frag_file[] = "osgOcean_heightmap.frag";

        osg::ref_ptr<osg::Program> program = ShaderManager::instance().createProgram( "heightmap", 
                                                                                      osgOcean_heightmap_vert_file, osgOcean_heightmap_frag_file, 
                                                                                      osgOcean_heightmap_vert,      osgOcean_heightmap_frag );

        if(program.valid())
            _heightmapCamera->getOrCreateStateSet()->setAttributeAndModes( program.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

        _surfaceStateSet->setTextureAttributeAndModes( _oceanScene->_heightmapUnit, heightmapTexture, osg::StateAttribute::ON );
    }

    dirty( false );
}

void OceanScene::ViewData::updateStateSet( bool eyeAboveWater )
{
    osg::Camera* currentCamera = _cv->getCurrentRenderBin()->getStage()->getCamera();

    _globalStateSet->getUniform("osgOcean_EyeUnderwater")->set(!eyeAboveWater);
    _globalStateSet->getUniform("osgOcean_Eye")->set( _cv->getEyePoint() );

    // Switch the fog state from underwater to above water or vice versa if necessary.
    float requiredFogDensity = eyeAboveWater ? _oceanScene->_aboveWaterFogDensity : _oceanScene->_underwaterFogDensity;
    osg::Vec4 requiredFogColor = eyeAboveWater ? _oceanScene->_aboveWaterFogColor   : _oceanScene->_underwaterFogColor;
    if (requiredFogDensity != _fog->getDensity() || requiredFogColor != _fog->getColor())
    {
        _fog->setDensity(requiredFogDensity);
        _fog->setColor  (requiredFogColor);
    }

    // Update viewport dimensions
    osg::Viewport* viewport = currentCamera->getViewport();
    _surfaceStateSet->getUniform("osgOcean_ViewportDimensions")->set( osg::Vec2(viewport->width(), viewport->height()) );

    // Check if the RTT passes should be enabled for this view.
    bool enabled = (_oceanScene->_viewsWithRTTEffectsDisabled.find(currentCamera->getView()) == _oceanScene->_viewsWithRTTEffectsDisabled.end());

    bool reflectionEnabled = _oceanScene->_enableReflections && eyeAboveWater && enabled && 
                             ( _cv->getEyePoint().z() < _oceanScene->_eyeHeightReflectionCutoff - _oceanScene->getOceanSurfaceHeight() );
    _surfaceStateSet->getUniform("osgOcean_EnableReflections")->set(reflectionEnabled);

    if (reflectionEnabled)
    {
        // Update the reflection matrix's translation to take into account
        // the ocean surface height. The translation we need is 2*h.
        // See http://www.gamedev.net/columns/hardcore/rnerwater1/page3.asp
        _reflectionMatrix = osg::Matrixf(  1,  0,  0,  0,
                                           0,  1,  0,  0,
                                           0,  0, -1,  0,    
                                           0,  0,  2 * _oceanScene->getOceanSurfaceHeight(),  1 );
    }

    // Refractions need to be calculated even when the eye is above water 
    // for the shoreline foam effect and translucency.
    bool refractionEnabled = _oceanScene->_enableRefractions && enabled;
    _surfaceStateSet->getUniform("osgOcean_EnableRefractions")->set(refractionEnabled);

    bool heightmapEnabled = _oceanScene->_enableHeightmap && eyeAboveWater && enabled;
    _surfaceStateSet->getUniform("osgOcean_EnableHeightmap")->set(heightmapEnabled);
}

void OceanScene::ViewData::cull( bool eyeAboveWater, bool surfaceVisible )
{
    // Assume _stateSet has been pushed before we get here.

    osg::Camera* currentCamera = _cv->getCurrentRenderBin()->getStage()->getCamera();

    bool reflectionEnabled;
    _surfaceStateSet->getUniform("osgOcean_EnableReflections")->get(reflectionEnabled);
    bool refractionEnabled;
    _surfaceStateSet->getUniform("osgOcean_EnableRefractions")->get(refractionEnabled);
    bool heightmapEnabled;
    _surfaceStateSet->getUniform("osgOcean_EnableHeightmap")->get(heightmapEnabled);

    _cv->pushStateSet(_oceanScene->_globalStateSet.get());

    // Render refraction if ocean surface is visible.
    if( surfaceVisible && refractionEnabled && _refractionCamera )
    {
        // update refraction camera and render refracted scene
        _refractionCamera->setViewMatrix( currentCamera->getViewMatrix() );
        _refractionCamera->setProjectionMatrix( currentCamera->getProjectionMatrix() );

        _refractionCamera->accept( *_cv );

        // Update inverse view and projection matrix
        osg::Matrixd viewMatrix = _refractionCamera->getViewMatrix();
        osg::Matrixd projectionMatrix = _refractionCamera->getProjectionMatrix();
        osg::Matrixd inverseViewProjectionMatrix = osg::Matrixd::inverse(viewMatrix * projectionMatrix);
        _surfaceStateSet->getUniform("osgOcean_RefractionInverseTransformation")->set(inverseViewProjectionMatrix);
    }

    // Render reflection if ocean surface is visible.
    if( surfaceVisible && reflectionEnabled && _reflectionCamera )
    {
        // update reflection camera and render reflected scene
        _reflectionCamera->setViewMatrix( _reflectionMatrix * currentCamera->getViewMatrix() );
        _reflectionCamera->setProjectionMatrix( currentCamera->getProjectionMatrix() );
        
        _reflectionCamera->accept( *_cv );
    }

    // Render height map if ocean surface is visible.
    if ( surfaceVisible && heightmapEnabled && _heightmapCamera ) 
    {
        // update refraction camera and render refracted scene
        _heightmapCamera->setViewMatrix( currentCamera->getViewMatrix() );
        _heightmapCamera->setProjectionMatrix( currentCamera->getProjectionMatrix() );

        _heightmapCamera->accept( *_cv );    
    }

    _cv->popStateSet();
}


void OceanScene::enableRTTEffectsForView(osg::View* view, bool enable)
{
    ViewSet::iterator it = _viewsWithRTTEffectsDisabled.find(view);
    if (enable)
    {
        // Default is enabled for all views, so if we find it we 
        // remove it, if not then nothing to do.
        if (it != _viewsWithRTTEffectsDisabled.end())
            _viewsWithRTTEffectsDisabled.erase(it);
    }
    else
    {
        if (it == _viewsWithRTTEffectsDisabled.end())
            _viewsWithRTTEffectsDisabled.insert(view);
    }
}

void OceanScene::traverse( osg::NodeVisitor& nv )
{
    if( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        if( _isDirty )
            init();

        update(nv);

        osg::Group::traverse(nv);
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

        if (cv) 
        {
            osg::Camera* currentCamera = cv->getCurrentRenderBin()->getStage()->getCamera();
            if (currentCamera->getName() == "ShadowCamera" ||
                currentCamera->getName() == "AnalysisCamera" ||
		currentCamera->getName() == "CamViewCamera")
            {
                // Do not do reflections and everything if we're in a shadow pass.
                osg::Group::traverse(nv);
            }
            else
            {
                bool eyeAboveWater  = isEyeAboveWater(cv->getEyePoint());

                // Push the stateset for view-dependent RTT effects.
                ViewData * vd = getViewDependentData( cv );

                if ( !vd || vd->_dirty || vd->_cv != cv || vd->_oceanScene != this )
                {
                    vd = initViewDependentData( cv, vd );
                    setViewDependentData( cv, vd );
                }

                if (vd)
                {
                    vd->updateStateSet(eyeAboveWater);
                    cv->pushStateSet( vd->_globalStateSet.get() );
                }

                bool surfaceVisible = _oceanSurface->isVisible(*cv, eyeAboveWater);

                (*_oceanSurface->getCullCallback())(_oceanSurface.get(), &nv);

                preRenderCull(*cv, eyeAboveWater, surfaceVisible);     // reflections/refractions
                
                // Above water
                if( eyeAboveWater )
                {
                    if(!_enableGlare)
                        cull(*cv, eyeAboveWater, surfaceVisible);        // normal scene render
                }
                // Below water passes
                else 
                {
                    if(!_enableDOF)
                        cull(*cv, eyeAboveWater, surfaceVisible);        // normal scene render
                }

                postRenderCull(*cv, eyeAboveWater, surfaceVisible);    // god rays/dof/glare

                if (vd)
                {
                    cv->popStateSet();
                }
            }
        }
        else
            osg::Group::traverse(nv);
    }
    else
        osg::Group::traverse(nv);
}

void OceanScene::update( osg::NodeVisitor& nv )
{
    if( _enableGodRays && _godrays.valid() )
        _godrays->accept(nv);

    if( _enableGodRays && _godRayBlendSurface.valid() )
        _godRayBlendSurface->accept(nv);

    if( _enableDistortion && _distortionSurface.valid() )
        _distortionSurface->accept(nv);
}

void OceanScene::preRenderCull( osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible )
{
    osg::Camera* currentCamera = cv.getCurrentRenderBin()->getStage()->getCamera();

    // Render all view-dependent RTT effects.
    ViewData * vd = getViewDependentData( &cv );

    if( vd )
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(vd->_mutex);
        vd->cull(eyeAboveWater, surfaceVisible);
    }

    // Now render other effects.

    // Above water
    if( eyeAboveWater )
    {
        if( _enableGlare )
        {
            // set view and projection to match main camera
            _glarePasses.at(0)->setViewMatrix( currentCamera->getViewMatrix() );
            _glarePasses.at(0)->setProjectionMatrix( currentCamera->getProjectionMatrix() );

            for( unsigned int i=0; i<_glarePasses.size()-1; ++i )
            {
                _glarePasses.at(i)->accept(cv);
            }
        }
    }
    // Below water
    else
    {
        if( _enableGodRays && _godrayPreRender.valid() )
        {
            // Render the god rays to texture
            _godrayPreRender->setViewMatrix( currentCamera->getViewMatrix() );
            _godrayPreRender->setProjectionMatrix( currentCamera->getProjectionMatrix() );
            _godrayPreRender->accept( cv );
        }

        if( _enableDOF )
        {
            // set view and projection to match main camera
            _dofPasses.at(0)->setViewMatrix( currentCamera->getViewMatrix() );
            _dofPasses.at(0)->setProjectionMatrix( currentCamera->getProjectionMatrix() );

            // pass the cull visitor down the chain
            for(unsigned int i = 0; i<_dofPasses.size()-1; ++i)
            {
                _dofPasses.at(i)->accept(cv);
            }
        }
    }
}

void OceanScene::postRenderCull( osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible )
{
    if( eyeAboveWater )
    {
        if( _enableGlare )
        {
            _glarePasses.back()->accept(cv);
        }
    }
    else
    {
        // dof screen first
        if( _enableDOF )
        {
            _dofPasses.back()->accept(cv);
        }
        // blend godrays ontop
        if( _enableGodRays )
        {
            _godrayPostRender->accept(cv);
        }
    }
}

void OceanScene::cull(osgUtil::CullVisitor& cv, bool eyeAboveWater, bool surfaceVisible)
{
    unsigned int mask = cv.getTraversalMask();

    cv.pushStateSet(_globalStateSet.get());

    if ( _oceanSurface.valid() && _oceanSurface->getNodeMask() != 0 && surfaceVisible )
    {
        // HACK: Make sure masks are set correctly on children... This 
        // assumes that the ocean surface is the only child that should have
        // the _surfaceMask bit set, and the silt node is the only child that
        // should have the _siltMask bit set. Otherwise other children will be
        // rendered twice.
        for (unsigned int i = 0; i < getNumChildren(); ++i)
        {
            osg::Node* child = getChild(i);
            if (child->getNodeMask() != 0 && child != _oceanTransform.get() && child != _siltClipNode.get())
                child->setNodeMask((child->getNodeMask() & ~_surfaceMask & ~_siltMask) | _normalSceneMask | _reflectionSceneMask | _refractionSceneMask);
        }

        // Push the view-dependent surface stateset.
        ViewData * vd = getViewDependentData( &cv );
        if (vd)
        {
            cv.pushStateSet( vd->_surfaceStateSet.get() );
        }

        cv.setTraversalMask( mask & _surfaceMask );
        osg::Group::traverse(cv);

        if (vd)
        {
            cv.popStateSet();
        }
    }

    // render rest of scene
    cv.setTraversalMask( mask & _normalSceneMask | _ARMask);
    osg::Group::traverse(cv);

    // pop globalStateSet
    cv.popStateSet(); 

    if( !eyeAboveWater )
    {
        if( _enableSilt )
        {
            cv.setTraversalMask( mask & _siltMask );
            osg::Group::traverse(cv);
        }
    }

    // put original mask back
    cv.setTraversalMask( mask );
}

bool OceanScene::isEyeAboveWater( const osg::Vec3& eye )
{
    return (eye.z() >= getOceanSurfaceHeight());
}

osg::Camera* OceanScene::renderToTexturePass( osg::Texture* textureBuffer )
{
    osg::Camera* camera = new osg::Camera;

    camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
    camera->setClearDepth( 1.0 );
    camera->setClearColor( osg::Vec4f(0.f, 0.f, 0.f, 1.f) );
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    camera->setViewport( 0,0, textureBuffer->getTextureWidth(), textureBuffer->getTextureHeight() );
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setRenderOrder(osg::Camera::PRE_RENDER, 1);
    camera->attach( osg::Camera::COLOR_BUFFER, textureBuffer );

    return camera;
}

osg::Camera* OceanScene::multipleRenderTargetPass(osg::Texture* texture0, osg::Camera::BufferComponent buffer0, 
                                                  osg::Texture* texture1, osg::Camera::BufferComponent buffer1 )
{
    osg::Camera* camera = new osg::Camera;

    camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
    camera->setClearDepth( 1.0 );
    camera->setClearColor( osg::Vec4f(0.f, 0.f, 0.f, 1.f) );
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    camera->setViewport( 0,0, texture0->getTextureWidth(), texture0->getTextureHeight() );
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setRenderOrder(osg::Camera::PRE_RENDER, 1);
    camera->attach( buffer0, texture0 );
    camera->attach( buffer1, texture1 );

    return camera;
}

osg::Camera* OceanScene::godrayFinalPass( void )
{
    osg::Camera* camera = new osg::Camera;

    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    camera->setProjectionMatrixAsOrtho( -1.f, 1.f, -1.f, 1.f, 1.0, 500.f );
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );
    
    return camera;
}

#include <osgOcean/shaders/osgOcean_downsample_vert.inl>
#include <osgOcean/shaders/osgOcean_downsample_frag.inl>
#include <osgOcean/shaders/osgOcean_downsample_glare_frag.inl>

osg::Camera* OceanScene::downsamplePass(osg::TextureRectangle* colorBuffer, 
                                        osg::TextureRectangle* auxBuffer,
                                        osg::TextureRectangle* outputTexture,
                                        bool isGlareEffect )
{
    static const char osgOcean_downsample_vert_file[]       = "osgOcean_downsample.vert";
    static const char osgOcean_downsample_frag_file[]       = "osgOcean_downsample.frag";
    static const char osgOcean_downsample_glare_frag_file[] = "osgOcean_downsample_glare.frag";

    osg::Vec2s lowResDims = _screenDims/4;

    osg::StateSet* ss = new osg::StateSet;

    if (isGlareEffect)
    {
        ss->setAttributeAndModes( 
            ShaderManager::instance().createProgram("downsample_glare", 
                                                    osgOcean_downsample_vert_file, osgOcean_downsample_glare_frag_file,
                                                    osgOcean_downsample_vert,      osgOcean_downsample_glare_frag), 
                                                    osg::StateAttribute::ON );

        ss->setTextureAttributeAndModes( 1, auxBuffer,   osg::StateAttribute::ON );

        ss->addUniform( new osg::Uniform("osgOcean_GlareThreshold", _glareThreshold ) );
        ss->addUniform( new osg::Uniform("osgOcean_LuminanceTexture", 1 ) );
    }
    else
    {
        ss->setAttributeAndModes( 
            ShaderManager::instance().createProgram("downsample", 
                                                    osgOcean_downsample_vert_file,  osgOcean_downsample_frag_file,
                                                    osgOcean_downsample_vert,       osgOcean_downsample_frag ), 
                                                    osg::StateAttribute::ON );
    }

    ss->setTextureAttributeAndModes( 0, colorBuffer, osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform( "osgOcean_ColorTexture", 0 ) );

    osg::Geode* downSizedQuad = createScreenQuad( lowResDims, _screenDims );
    downSizedQuad->setStateSet(ss);

    osg::Camera* RTTCamera = renderToTexturePass( outputTexture );
    RTTCamera->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0, lowResDims.y(), 1, 10 );
    RTTCamera->setViewMatrix(osg::Matrix::identity());
    RTTCamera->addChild( downSizedQuad );

    return RTTCamera;
}

#include <osgOcean/shaders/osgOcean_gaussian_vert.inl>
#include <osgOcean/shaders/osgOcean_gaussian1_frag.inl>
#include <osgOcean/shaders/osgOcean_gaussian2_frag.inl>

osg::Camera* OceanScene::gaussianPass( osg::TextureRectangle* inputTexture, osg::TextureRectangle* outputTexture, bool isXAxis )
{
    static const char osgOcean_gaussian_vert_file[]  = "osgOcean_gaussian.vert";
    static const char osgOcean_gaussian1_frag_file[] = "osgOcean_gaussian1.frag";
    static const char osgOcean_gaussian2_frag_file[] = "osgOcean_gaussian2.frag";

    osg::Vec2s lowResDims = _screenDims/4.f;

    osg::StateSet* ss = new osg::StateSet;
    
    if (isXAxis)
    {
        ss->setAttributeAndModes( 
            ShaderManager::instance().createProgram("gaussian1", 
                                                    osgOcean_gaussian_vert_file, osgOcean_gaussian1_frag_file, 
                                                    osgOcean_gaussian_vert,      osgOcean_gaussian1_frag), 
                                                    osg::StateAttribute::ON );
    }
    else
    {
        ss->setAttributeAndModes( 
            ShaderManager::instance().createProgram("gaussian2", 
                                                    osgOcean_gaussian_vert_file, osgOcean_gaussian2_frag_file, 
                                                    osgOcean_gaussian_vert,      osgOcean_gaussian2_frag), 
                                                    osg::StateAttribute::ON );
    }

    ss->setTextureAttributeAndModes( 0, inputTexture, osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform( "osgOcean_GaussianTexture", 0 ) );

    osg::Geode* gaussianQuad = createScreenQuad( lowResDims, lowResDims );
    gaussianQuad->setStateSet(ss);

    osg::Camera* dofGaussianPass = renderToTexturePass( outputTexture );
    dofGaussianPass->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0, lowResDims.y(), 1, 10 );
    dofGaussianPass->addChild(gaussianQuad);

    return dofGaussianPass;
}


#include <osgOcean/shaders/osgOcean_dof_combiner_vert.inl>
#include <osgOcean/shaders/osgOcean_dof_combiner_frag.inl>

osg::Camera* OceanScene::dofCombinerPass(osg::TextureRectangle* fullscreenTexture, 
                                         osg::TextureRectangle* fullDepthTexture,
                                         osg::TextureRectangle* blurTexture,
                                         osg::TextureRectangle* outputTexture )
{
    static const char osgOcean_dof_combiner_vert_file[] = "osgOcean_dof_combiner.vert";
    static const char osgOcean_dof_combiner_frag_file[] = "osgOcean_dof_combiner.frag";

    osg::Vec2f screenRes( (float)_screenDims.x(), (float)_screenDims.y() );
    osg::Vec2f invScreenRes( 1.f / (float)_screenDims.x(), 1.f / (float)_screenDims.y() );
    osg::Vec2f lowRes( float(_screenDims.x())*0.25f, float(_screenDims.y())*0.25f );

    osg::StateSet* ss = new osg::StateSet;
    ss->setTextureAttributeAndModes( 0, fullscreenTexture, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes( 1, fullDepthTexture,  osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes( 2, blurTexture,       osg::StateAttribute::ON );

    ss->setAttributeAndModes( 
        ShaderManager::instance().createProgram("dof_combiner", 
                                                osgOcean_dof_combiner_vert_file, osgOcean_dof_combiner_frag_file,
                                                osgOcean_dof_combiner_vert,      osgOcean_dof_combiner_frag), 
                                                osg::StateAttribute::ON );

    ss->addUniform( new osg::Uniform( "osgOcean_FullColourMap", 0 ) );
    ss->addUniform( new osg::Uniform( "osgOcean_FullDepthMap",  1 ) );
    ss->addUniform( new osg::Uniform( "osgOcean_BlurMap",       2 ) );
    ss->addUniform( new osg::Uniform( "osgOcean_ScreenRes",     screenRes ) );
    ss->addUniform( new osg::Uniform( "osgOcean_ScreenResInv",  invScreenRes ) );
    ss->addUniform( new osg::Uniform( "osgOcean_LowRes",        lowRes ) );

    osg::Geode* combinedDOFQuad = createScreenQuad( _screenDims, osg::Vec2s(1,1) );
    combinedDOFQuad->setStateSet(ss);

    osg::Camera* dofCombineCamera = renderToTexturePass( outputTexture );
    dofCombineCamera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0, _screenDims.y(), 1, 10 );
    dofCombineCamera->addChild( combinedDOFQuad );

    return dofCombineCamera;
}

osg::Camera* OceanScene::dofFinalPass( osg::TextureRectangle* combinedTexture )
{
    _distortionSurface = new DistortionSurface(osg::Vec3f(0,0,-1), osg::Vec2f(_screenDims.x(),_screenDims.y()), combinedTexture);

    osg::Camera* camera = new osg::Camera;
    camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    camera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0.f, _screenDims.y(), 1.0, 500.f );
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );
    camera->addChild(_distortionSurface.get());

    return camera;
}

#include <osgOcean/shaders/osgOcean_streak_vert.inl>
#include <osgOcean/shaders/osgOcean_streak_frag.inl>

osg::Camera* OceanScene::glarePass(osg::TextureRectangle* streakInput, 
                                   osg::TextureRectangle* steakOutput, 
                                   int pass, 
                                   osg::Vec2f direction )
{
    static const char osgOcean_streak_vert_file[] = "osgOcean_streak.vert";
    static const char osgOcean_streak_frag_file[] = "osgOcean_streak.frag";

    osg::Vec2s lowResDims = _screenDims / 4;

    osg::Camera* glarePass = renderToTexturePass( steakOutput );
    glarePass->setClearColor( osg::Vec4f( 0.f, 0.f, 0.f, 0.f) );
    glarePass->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0.f, lowResDims.y(), 1.0, 500.f );
    glarePass->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    {
        osg::Program* program = 
            ShaderManager::instance().createProgram( "streak_shader", 
                                                     osgOcean_streak_vert_file, osgOcean_streak_frag_file, 
                                                     osgOcean_streak_vert,      osgOcean_streak_frag );

        osg::Geode* screenQuad = createScreenQuad(lowResDims, lowResDims);
        screenQuad->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        screenQuad->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Buffer", 0) );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Pass", float(pass)) );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Direction", direction) );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Attenuation", _glareAttenuation ) );
        screenQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0,streakInput,osg::StateAttribute::ON);
        glarePass->addChild( screenQuad ); 
    }

    return glarePass;
}

#include <osgOcean/shaders/osgOcean_glare_composite_vert.inl>
#include <osgOcean/shaders/osgOcean_glare_composite_frag.inl>

osg::Camera* OceanScene::glareCombinerPass( osg::TextureRectangle* fullscreenTexture,
                                            osg::TextureRectangle* glareTexture1,
                                            osg::TextureRectangle* glareTexture2,
                                            osg::TextureRectangle* glareTexture3,
                                            osg::TextureRectangle* glareTexture4 )
{
    osg::Camera* camera = new osg::Camera;

    camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    camera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0.f, _screenDims.y(), 1.0, 500.f );
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );

    osg::Geode* quad = createScreenQuad( _screenDims, _screenDims );

    static const char osgOcean_glare_composite_vert_file[] = "osgOcean_glare_composite.vert";
    static const char osgOcean_glare_composite_frag_file[] = "osgOcean_glare_composite.frag";

    osg::Program* program = 
        ShaderManager::instance().createProgram( "glare_composite", 
                                                 osgOcean_glare_composite_vert_file, osgOcean_glare_composite_frag_file, 
                                                 osgOcean_glare_composite_vert,      osgOcean_glare_composite_frag );

    osg::StateSet* ss = quad->getOrCreateStateSet();
    ss->setAttributeAndModes(program, osg::StateAttribute::ON);
    ss->setTextureAttributeAndModes(0, fullscreenTexture, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(1, glareTexture1, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(2, glareTexture2, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(3, glareTexture3, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(4, glareTexture4, osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform("osgOcean_ColorBuffer",   0 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer1", 1 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer2", 2 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer3", 3 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer4", 4 ) );

    camera->addChild( quad );

    return camera;
}

osg::Texture2D* OceanScene::createTexture2D( const osg::Vec2s& size, GLint format )
{
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setTextureSize(size.x(), size.y());
    texture->setInternalFormat(format);
    texture->setFilter(osg::Texture2D::MIN_FILTER, format == GL_DEPTH_COMPONENT ? osg::Texture2D::NEAREST : osg::Texture2D::LINEAR );
    texture->setFilter(osg::Texture2D::MAG_FILTER, format == GL_DEPTH_COMPONENT ? osg::Texture2D::NEAREST : osg::Texture2D::LINEAR );
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    texture->setDataVariance(osg::Object::DYNAMIC);
    return texture;
}

osg::TextureRectangle* OceanScene::createTextureRectangle( const osg::Vec2s& size, GLint format )
{
    osg::TextureRectangle* texture = new osg::TextureRectangle();
    texture->setTextureSize(size.x(), size.y());
    texture->setInternalFormat(format);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    texture->setDataVariance(osg::Object::DYNAMIC);
    return texture;
}

osg::Geode* OceanScene::createScreenQuad( const osg::Vec2s& dims, const osg::Vec2s& texSize )
{
    osg::Geode* geode = new osg::Geode;

    osg::Geometry* quad = osg::createTexturedQuadGeometry( 
        osg::Vec3f(0.f,0.f,0.f), 
        osg::Vec3f(dims.x(), 0.f, 0.f), 
        osg::Vec3f( 0.f,dims.y(), 0.0 ),
        (float)texSize.x(),
        (float)texSize.y() );

    geode->addDrawable(quad);

    return geode;
}

#include <osgOcean/shaders/osgOcean_ocean_scene_vert.inl>
#include <osgOcean/shaders/osgOcean_ocean_scene_frag.inl>

osg::Program* OceanScene::createDefaultSceneShader(void)
{
    static const char osgOcean_ocean_scene_vert_file[] = "osgOcean_ocean_scene.vert";
    static const char osgOcean_ocean_scene_frag_file[] = "osgOcean_ocean_scene.frag";

    return ShaderManager::instance().createProgram("scene_shader", 
                                                   osgOcean_ocean_scene_vert_file, osgOcean_ocean_scene_frag_file,
                                                   osgOcean_ocean_scene_vert,      osgOcean_ocean_scene_frag);
}

void OceanScene::addResourcePaths(void)
{
    const std::string shaderPath  = "resources/shaders/";
    const std::string texturePath = "resources/textures/";

    osgDB::FilePathList& pathList = osgDB::Registry::instance()->getDataFilePathList();

    bool shaderPathPresent = false;
    bool texturePathPresent = false;

    for(unsigned int i = 0; i < pathList.size(); ++i )
    {
        if( pathList.at(i).compare(shaderPath) == 0 )
            shaderPathPresent = true;

        if( pathList.at(i).compare(texturePath) == 0 )
            texturePathPresent = true;
    }

    if(!texturePathPresent)
        pathList.push_back(texturePath);

    if(!shaderPathPresent)
        pathList.push_back(shaderPath);
}

// -------------------------------
//     Callback implementations
// -------------------------------

OceanScene::CameraCullCallback::CameraCullCallback(OceanScene* oceanScene):
_oceanScene(oceanScene)
{
}

void OceanScene::CameraCullCallback::operator()(osg::Node*, osg::NodeVisitor* nv)
{
    _oceanScene->osg::Group::traverse(*nv);
}

OceanScene::PrerenderCameraCullCallback::PrerenderCameraCullCallback(OceanScene* oceanScene):
_oceanScene(oceanScene)
{
}

void OceanScene::PrerenderCameraCullCallback::operator()(osg::Node*, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*> (nv);

    if(cv)
    {
        bool eyeAboveWater  = _oceanScene->isEyeAboveWater(cv->getEyePoint());
        bool surfaceVisible = _oceanScene->getOceanTechnique()->isVisible(*cv, eyeAboveWater);
        _oceanScene->cull(*cv, eyeAboveWater, surfaceVisible);
    }
}


OceanScene::EventHandler::EventHandler(OceanScene* oceanScene):
_oceanScene(oceanScene)
{
}

bool OceanScene::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
{
    if (ea.getHandled()) return false;

    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYUP):
        {
            // Reflections
            if (ea.getKey() == 'r')
            {
                _oceanScene->enableReflections(!_oceanScene->areReflectionsEnabled());
                osg::notify(osg::NOTICE) << "Reflections " << (_oceanScene->areReflectionsEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Refractions
            if (ea.getKey() == 'R')
            {
                _oceanScene->enableRefractions(!_oceanScene->areRefractionsEnabled());
                osg::notify(osg::NOTICE) << "Refractions " << (_oceanScene->areRefractionsEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // DOF
            if (ea.getKey() == 'o')
            {
                _oceanScene->enableUnderwaterDOF(!_oceanScene->isUnderwaterDOFEnabled());
                osg::notify(osg::NOTICE) << "Depth of field " << (_oceanScene->isUnderwaterDOFEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Glare
            if (ea.getKey() == 'g')
            {
                _oceanScene->enableGlare(!_oceanScene->isGlareEnabled());
                osg::notify(osg::NOTICE) << "Glare " << (_oceanScene->isGlareEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // God rays
            if (ea.getKey() == 'G')
            {
                _oceanScene->enableGodRays(!_oceanScene->areGodRaysEnabled());
                osg::notify(osg::NOTICE) << "God rays " << (_oceanScene->areGodRaysEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Silt
            if (ea.getKey() == 't')
            {
                _oceanScene->enableSilt(!_oceanScene->isSiltEnabled());
                osg::notify(osg::NOTICE) << "Silt " << (_oceanScene->isSiltEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Underwater scattering
            if (ea.getKey() == 'T')
            {
                _oceanScene->enableUnderwaterScattering(!_oceanScene->isUnderwaterScatteringEnabled());
                osg::notify(osg::NOTICE) << "Underwater scattering " << (_oceanScene->isUnderwaterScatteringEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Height map
            if (ea.getKey() == 'H')
            {
                _oceanScene->enableHeightmap(!_oceanScene->isHeightmapEnabled());
                _oceanScene->getOceanTechnique()->dirty();      // Make it reload shaders.
                osg::notify(osg::NOTICE) << "Height lookup for shoreline foam and sine shape " << (_oceanScene->isHeightmapEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Ocean surface height
            if (ea.getKey() == '+')
            {
                _oceanScene->setOceanSurfaceHeight(_oceanScene->getOceanSurfaceHeight() + 1.0);
                osg::notify(osg::NOTICE) << "Ocean surface is now at z = " << _oceanScene->getOceanSurfaceHeight() << std::endl;
                return true;
            }
            if (ea.getKey() == '-')
            {
                _oceanScene->setOceanSurfaceHeight(_oceanScene->getOceanSurfaceHeight() - 1.0);
                osg::notify(osg::NOTICE) << "Ocean surface is now at z = " << _oceanScene->getOceanSurfaceHeight() << std::endl;
                return true;
            }

            break;
        }
    default:
        break;
    }

    return false;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void OceanScene::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
    usage.addKeyboardMouseBinding("r","Toggle reflections (above water)");
    usage.addKeyboardMouseBinding("R","Toggle refractions (underwater)");
    usage.addKeyboardMouseBinding("o","Toggle Depth of Field (DOF) (underwater)");
    usage.addKeyboardMouseBinding("g","Toggle glare (above water)");
    usage.addKeyboardMouseBinding("G","Toggle God rays (underwater)");
    usage.addKeyboardMouseBinding("t","Toggle silt (underwater)");
    usage.addKeyboardMouseBinding("T","Toggle scattering (underwater)");
    usage.addKeyboardMouseBinding("H","Toggle Height lookup for shoreline foam and sine shape (above water)");
    usage.addKeyboardMouseBinding("+","Raise ocean surface");
    usage.addKeyboardMouseBinding("-","Lower ocean surface");
}

// register the read and write functions with the osgDB::Registry.
REGISTER_DOTOSGWRAPPER(OceanScene)
(
    new osgOcean::OceanScene,
    "OceanScene",
    "Object Node OceanScene Group",
    NULL,
    NULL
);
