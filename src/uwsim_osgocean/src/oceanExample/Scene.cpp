#include "Scene.h"
#include "ScopedTimer.h"

#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osg/LightSource>

#include <osgOcean/ShaderManager>

// ----------------------------------------------------
//               Camera Track Callback
// ----------------------------------------------------

class CameraTrackCallback: public osg::NodeCallback
{
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            osg::Vec3f centre,up,eye;
            // get MAIN camera eye,centre,up
            cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye,centre,up);
            // update position
            osg::MatrixTransform* mt = static_cast<osg::MatrixTransform*>(node);
            mt->setMatrix( osg::Matrix::translate( eye.x(), eye.y(), mt->getMatrix().getTrans().z() ) );
        }

        traverse(node, nv); 
    }
};

// ----------------------------------------------------
//                       Scene 
// ----------------------------------------------------

Scene::Scene( 
    const osg::Vec2f& windDirection,
    float windSpeed,
    float depth,
    float reflectionDamping,
    float scale,
    bool  isChoppy,
    float choppyFactor,
    float crestFoamHeight,
    bool  useVBO,
    const std::string& terrain_shader_basename)
{
    _sceneType = CLEAR;
    _useVBO = useVBO;

    _cubemapDirs.push_back( "sky_clear" );
    _cubemapDirs.push_back( "sky_dusk" );
    _cubemapDirs.push_back( "sky_fair_cloudy" );

    _fogColors.push_back( intColor( 199,226,255 ) );
    _fogColors.push_back( intColor( 244,228,179 ) );
    _fogColors.push_back( intColor( 172,224,251 ) );

    _waterFogColors.push_back( intColor(27,57,109) );
    _waterFogColors.push_back( intColor(44,69,106 ) );
    _waterFogColors.push_back( intColor(84,135,172 ) );

    _underwaterAttenuations.push_back( osg::Vec3f(0.015f, 0.0075f, 0.005f) );
    _underwaterAttenuations.push_back( osg::Vec3f(0.015f, 0.0075f, 0.005f) );
    _underwaterAttenuations.push_back( osg::Vec3f(0.008f, 0.003f, 0.002f) );

    _underwaterDiffuse.push_back( intColor(27,57,109) );
    _underwaterDiffuse.push_back( intColor(44,69,106) );
    _underwaterDiffuse.push_back( intColor(84,135,172) );

    _lightColors.push_back( intColor( 105,138,174 ) );
    _lightColors.push_back( intColor( 105,138,174 ) );
    _lightColors.push_back( intColor( 105,138,174 ) );

    _sunPositions.push_back( osg::Vec3f(326.573, 1212.99 ,1275.19) );
    _sunPositions.push_back( osg::Vec3f(520.f, 1900.f, 550.f ) );
    _sunPositions.push_back( osg::Vec3f(-1056.89f, -771.886f, 1221.18f ) );

    _sunDiffuse.push_back( intColor( 191, 191, 191 ) );
    _sunDiffuse.push_back( intColor( 251, 251, 161 ) );
    _sunDiffuse.push_back( intColor( 191, 191, 191 ) );

    build(windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, crestFoamHeight, _useVBO, terrain_shader_basename);
}

void Scene::build( 
    const osg::Vec2f& windDirection,
    float windSpeed,
    float depth,
    float reflectionDamping,
    float waveScale,
    bool  isChoppy,
    float choppyFactor,
    float crestFoamHeight,
    bool  useVBO,
    const std::string& terrain_shader_basename )
{
    {
        ScopedTimer buildSceneTimer("Building scene... \n", osg::notify(osg::NOTICE));

        _scene = new osg::Group; 

        {
            ScopedTimer cubemapTimer("  . Loading cubemaps: ", osg::notify(osg::NOTICE));
            _cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );
        }

        // Set up surface
        {
            ScopedTimer oceanSurfaceTimer("  . Generating ocean surface: ", osg::notify(osg::NOTICE));

            if (useVBO)
            {
                _FFToceanSurface = 
                    new osgOcean::FFTOceanSurfaceVBO( 
                    64, 256, 17, 
                    windDirection, 
                    windSpeed, depth, 
                    reflectionDamping, 
                    waveScale, isChoppy, 
                    choppyFactor, 10.f, 256 );
            }
            else
            {
                _FFToceanSurface = 
                    new osgOcean::FFTOceanSurface( 
                    64, 256, 17, 
                    windDirection, 
                    windSpeed, depth, 
                    reflectionDamping, 
                    waveScale, isChoppy, 
                    choppyFactor, 10.f, 256 );
            }

            _FFToceanSurface->setEnvironmentMap( _cubemap.get() );
            _FFToceanSurface->setFoamBottomHeight( 2.2f );
            _FFToceanSurface->setFoamTopHeight( 3.0f );
            _FFToceanSurface->enableCrestFoam( true );
            _FFToceanSurface->setLightColor( _lightColors[_sceneType] );
            // Make the ocean surface track with the main camera position, giving the illusion
            // of an endless ocean surface.
            _FFToceanSurface->enableEndlessOcean(true);
        }

        // Set up ocean scene, add surface
        {
            ScopedTimer oceanSceneTimer("  . Creating ocean scene: ", osg::notify(osg::NOTICE));
            osg::Vec3f sunDir = -_sunPositions[_sceneType];
            sunDir.normalize();

            _oceanScene = new osgOcean::OceanScene( _FFToceanSurface.get() );
            _oceanScene->setLightID(0);
            _oceanScene->enableReflections(true);
            _oceanScene->enableRefractions(true);
            _oceanScene->enableHeightmap(true);

            // Set the size of _oceanCylinder which follows the camera underwater. 
            // This cylinder prevents the clear from being visible past the far plane 
            // instead it will be the fog color.
            // The size of the cylinder should be changed according the size of the ocean surface.
            _oceanScene->setCylinderSize( 1900.f, 4000.f );

            _oceanScene->setAboveWaterFog(0.0012f, _fogColors[_sceneType] );
            _oceanScene->setUnderwaterFog(0.002f,  _waterFogColors[_sceneType] );
            _oceanScene->setUnderwaterDiffuse( _underwaterDiffuse[_sceneType] );
            _oceanScene->setUnderwaterAttenuation( _underwaterAttenuations[_sceneType] );

            _oceanScene->setSunDirection( sunDir );
            _oceanScene->enableGodRays(true);
            _oceanScene->enableSilt(true);
            _oceanScene->enableUnderwaterDOF(true);
            _oceanScene->enableUnderwaterScattering(true);
            _oceanScene->enableDistortion(true);
            _oceanScene->enableGlare(true);
            _oceanScene->setGlareAttenuation(0.8f);

            // create sky dome and add to ocean scene
            // set masks so it appears in reflected scene and normal scene
            _skyDome = new SkyDome( 1900.f, 16, 16, _cubemap.get() );
            _skyDome->setNodeMask( _oceanScene->getReflectedSceneMask() | 
                                   _oceanScene->getNormalSceneMask()    | 
                                   _oceanScene->getRefractedSceneMask());

            // add a pat to track the camera
            osg::MatrixTransform* transform = new osg::MatrixTransform;
            transform->setDataVariance( osg::Object::DYNAMIC );
            transform->setMatrix( osg::Matrixf::translate( osg::Vec3f(0.f, 0.f, 0.f) ));
            transform->setCullCallback( new CameraTrackCallback );

            transform->addChild( _skyDome.get() );

            _oceanScene->addChild( transform );

            {
                // Create and add fake texture for use with nodes without any texture
                // since the OceanScene default scene shader assumes that texture unit 
                // 0 is used as a base texture map.
                osg::Image * image = new osg::Image;
                image->allocateImage( 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE );
                *(osg::Vec4ub*)image->data() = osg::Vec4ub( 0xFF, 0xFF, 0xFF, 0xFF );

                osg::Texture2D* fakeTex = new osg::Texture2D( image );
                fakeTex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
                fakeTex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
                fakeTex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
                fakeTex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);

                osg::StateSet* stateset = _oceanScene->getOrCreateStateSet();
                stateset->setTextureAttribute(0,fakeTex,osg::StateAttribute::ON);
                stateset->setTextureMode(0,GL_TEXTURE_1D,osg::StateAttribute::OFF);
                stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
                stateset->setTextureMode(0,GL_TEXTURE_3D,osg::StateAttribute::OFF);
            }

        }

        {
            ScopedTimer islandsTimer("  . Loading islands: ", osg::notify(osg::NOTICE));
            osg::ref_ptr<osg::Node> islandModel = loadIslands(terrain_shader_basename);

            if( islandModel.valid() )
            {
                _islandSwitch = new osg::Switch;
                _islandSwitch->addChild( islandModel.get(), true );
                
                _islandSwitch->setNodeMask( _oceanScene->getNormalSceneMask()    | 
                                            _oceanScene->getReflectedSceneMask() | 
                                            _oceanScene->getRefractedSceneMask() |
                                            _oceanScene->getHeightmapMask()      | 
                                            RECEIVE_SHADOW);

                _oceanScene->addChild( _islandSwitch.get() );
            }
        }

        {
            ScopedTimer lightingTimer("  . Setting up lighting: ", osg::notify(osg::NOTICE));
            osg::LightSource* lightSource = new osg::LightSource;
            lightSource->setNodeMask(lightSource->getNodeMask() & ~CAST_SHADOW & ~RECEIVE_SHADOW);
            lightSource->setLocalStateSetModes();

            _light = lightSource->getLight();
            _light->setLightNum(0);
            _light->setAmbient( osg::Vec4d(0.3f, 0.3f, 0.3f, 1.0f ));
            _light->setDiffuse( _sunDiffuse[_sceneType] );
            _light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
#ifdef POINT_LIGHT
            _light->setPosition( osg::Vec4f(_sunPositions[_sceneType], 1.f) ); // point light
#else
            osg::Vec3f direction(_sunPositions[_sceneType]);
            direction.normalize();
            _light->setPosition( osg::Vec4f(direction, 0.0) );  // directional light
#endif

            _scene->addChild( lightSource );
            _scene->addChild( _oceanScene.get() );
            //_scene->addChild( sunDebug(_sunPositions[CLOUDY]) );
        }

        osg::notify(osg::NOTICE) << "complete.\nTime Taken: ";
    }
}

void Scene::changeScene( SCENE_TYPE type )
{
    _sceneType = type;

    _cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );
    _skyDome->setCubeMap( _cubemap.get() );

    _FFToceanSurface->setEnvironmentMap( _cubemap.get() );
    _FFToceanSurface->setLightColor( _lightColors[type] );

    _oceanScene->setAboveWaterFog(0.0012f, _fogColors[_sceneType] );
    _oceanScene->setUnderwaterFog(0.002f,  _waterFogColors[_sceneType] );
    _oceanScene->setUnderwaterDiffuse( _underwaterDiffuse[_sceneType] );
    _oceanScene->setUnderwaterAttenuation( _underwaterAttenuations[_sceneType] );

    osg::Vec3f sunDir = -_sunPositions[_sceneType];
    sunDir.normalize();

    _oceanScene->setSunDirection( sunDir );

#ifdef POINT_LIGHT
    _light->setPosition( osg::Vec4f(_sunPositions[_sceneType], 1.f) );
#else
    _light->setPosition( osg::Vec4f(-sunDir, 0.f) );
#endif
    _light->setDiffuse( _sunDiffuse[_sceneType] ) ;

    if(_islandSwitch.valid() )
    {
        if(_sceneType == CLEAR || _sceneType == CLOUDY)
            _islandSwitch->setAllChildrenOn();
        else
            _islandSwitch->setAllChildrenOff();
    }
}

#define USE_CUSTOM_SHADER

// Load the islands model
// Here we attach a custom shader to the model.
// This shader overrides the default shader applied by OceanScene but uses uniforms applied by OceanScene.
// The custom shader is needed to add multi-texturing and bump mapping to the terrain.
osg::Node* Scene::loadIslands(const std::string& terrain_shader_basename)
{
    osgDB::Registry::instance()->getDataFilePathList().push_back("resources/island");
    const std::string filename = "islands.ive";
    osg::ref_ptr<osg::Node> island = osgDB::readNodeFile(filename);

    if(!island.valid()){
        osg::notify(osg::WARN) << "Could not find: " << filename << std::endl;
        return NULL;
    }

#ifdef USE_CUSTOM_SHADER
    const std::string terrain_vertex   = terrain_shader_basename + ".vert";
    const std::string terrain_fragment = terrain_shader_basename + ".frag";
    
    osg::Program* program = osgOcean::ShaderManager::instance().createProgram("terrain", terrain_vertex, terrain_fragment, "", "" );
    if(program) program->addBindAttribLocation("aTangent", 6);

#endif
    island->setNodeMask( _oceanScene->getNormalSceneMask()    | 
                         _oceanScene->getReflectedSceneMask() | 
                         _oceanScene->getRefractedSceneMask() | 
                         _oceanScene->getHeightmapMask()      | 
                         RECEIVE_SHADOW);
    island->getStateSet()->addUniform( new osg::Uniform( "uTextureMap", 0 ) );

#ifdef USE_CUSTOM_SHADER
    island->getOrCreateStateSet()->setAttributeAndModes(program,osg::StateAttribute::ON);
    island->getStateSet()->addUniform( new osg::Uniform( "uOverlayMap", 1 ) );
    island->getStateSet()->addUniform( new osg::Uniform( "uNormalMap",  2 ) );
#endif
    osg::PositionAttitudeTransform* islandpat = new osg::PositionAttitudeTransform;
    islandpat->setPosition(osg::Vec3f( -island->getBound().center()+osg::Vec3f(0.0, 0.0, -15.f) ) );
    islandpat->setScale( osg::Vec3f(4.f, 4.f, 3.f ) );
    islandpat->addChild(island.get());

    return islandpat;
}

osg::ref_ptr<osg::TextureCubeMap> Scene::loadCubeMapTextures( const std::string& dir )
{
    enum {POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z};

    std::string filenames[6];

    filenames[POS_X] = "resources/textures/" + dir + "/east.png";
    filenames[NEG_X] = "resources/textures/" + dir + "/west.png";
    filenames[POS_Z] = "resources/textures/" + dir + "/north.png";
    filenames[NEG_Z] = "resources/textures/" + dir + "/south.png";
    filenames[POS_Y] = "resources/textures/" + dir + "/down.png";
    filenames[NEG_Y] = "resources/textures/" + dir + "/up.png";

    osg::ref_ptr<osg::TextureCubeMap> cubeMap = new osg::TextureCubeMap;
    cubeMap->setInternalFormat(GL_RGBA);

    cubeMap->setFilter( osg::Texture::MIN_FILTER,    osg::Texture::LINEAR_MIPMAP_LINEAR);
    cubeMap->setFilter( osg::Texture::MAG_FILTER,    osg::Texture::LINEAR);
    cubeMap->setWrap  ( osg::Texture::WRAP_S,        osg::Texture::CLAMP_TO_EDGE);
    cubeMap->setWrap  ( osg::Texture::WRAP_T,        osg::Texture::CLAMP_TO_EDGE);

    cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, osgDB::readImageFile( filenames[NEG_X] ) );
    cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, osgDB::readImageFile( filenames[POS_X] ) );
    cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, osgDB::readImageFile( filenames[NEG_Y] ) );
    cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, osgDB::readImageFile( filenames[POS_Y] ) );
    cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, osgDB::readImageFile( filenames[NEG_Z] ) );
    cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, osgDB::readImageFile( filenames[POS_Z] ) );

    return cubeMap;
}

osg::Geode* Scene::sunDebug( const osg::Vec3f& position )
{
    osg::ShapeDrawable* sphereDraw = new osg::ShapeDrawable( new osg::Sphere( position, 15.f ) );
    sphereDraw->setColor(osg::Vec4f(1.f,0.f,0.f,1.f));

    osg::Geode* sphereGeode = new osg::Geode;
    sphereGeode->addDrawable( sphereDraw );

    return sphereGeode;
}
