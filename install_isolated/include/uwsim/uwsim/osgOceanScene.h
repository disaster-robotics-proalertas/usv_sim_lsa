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
 *
 * Adapted to UWSim by Mario Prats
 */

#pragma once
#include <osg/Switch>
#include <osg/TextureCubeMap>

#include <osgText/Text>

#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/FFTOceanSurfaceVBO>

#include "SkyDome.h"
#include "ConfigXMLParser.h"
#include <osgOcean/ShaderManager>

enum DrawMask
{
  CAST_SHADOW = (0x1 << 30), RECEIVE_SHADOW = (0x1 << 29),
};

class osgOceanScene : public osg::Referenced
{
public:
  enum SCENE_TYPE
  {
    CLEAR, DUSK, CLOUDY
  };
  osg::ref_ptr<osg::MatrixTransform> localizedWorld;

private:
  SCENE_TYPE _sceneType;
  bool _useVBO;

  osg::ref_ptr<osgText::Text> _modeText;
  osg::ref_ptr<osg::Group> _scene;

  osg::ref_ptr<osgOcean::OceanScene> _oceanScene;
  osg::ref_ptr<osgOcean::FFTOceanTechnique> _FFToceanSurface;
  osg::ref_ptr<osg::TextureCubeMap> _cubemap;
  osg::ref_ptr<SkyDome> _skyDome;

  std::vector<std::string> _cubemapDirs;
  std::vector<osg::Vec4f> _lightColors;
  std::vector<osg::Vec4f> _fogColors;
  std::vector<osg::Vec3f> _underwaterAttenuations;
  std::vector<osg::Vec4f> _underwaterDiffuse;

  osg::ref_ptr<osg::Light> _light;

  std::vector<osg::Vec3f> _sunPositions;
  std::vector<osg::Vec4f> _sunDiffuse;
  std::vector<osg::Vec4f> _waterFogColors;

  osg::ref_ptr<osg::Switch> _islandSwitch;

public:
  osgOceanScene(double offsetp[3], double offsetr[3], const osg::Vec2f& windDirection = osg::Vec2f(1.0f, 1.0f),
                float windSpeed = 12.f, float depth = 10000.f, float reflectionDamping = 0.35f, float scale = 1e-8,
                bool isChoppy = true, float choppyFactor = -2.5f, float crestFoamHeight = 2.2f, bool useVBO = false,
                const std::string& terrain_shader_basename = "terrain");

  void build(double offsetp[3], double offsetr[3], const osg::Vec2f& windDirection, float windSpeed, float depth,
             float reflectionDamping, float waveScale, bool isChoppy, float choppyFactor, float crestFoamHeight,
             bool useVBO, const std::string& terrain_shader_basename);

  void changeScene(SCENE_TYPE type);

  // Load the islands model
  // Here we attach a custom shader to the model.
  // This shader overrides the default shader applied by OceanScene but uses uniforms applied by OceanScene.
  // The custom shader is needed to add multi-texturing and bump mapping to the terrain.
  osg::Node* loadIslands(const std::string& terrain_shader_basename);

  osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures(const std::string& dir);

  osg::Geode* sunDebug(const osg::Vec3f& position);

  inline osg::Vec4f intColor(unsigned r, unsigned g, unsigned b, unsigned a = 255)
  {
    float div = 1.f / 255.f;
    return osg::Vec4f(div * (float)r, div * (float)g, div * float(b), div * (float)a);
  }

  inline osgOcean::OceanScene::EventHandler* getOceanSceneEventHandler()
  {
    return _oceanScene->getEventHandler();
  }

  inline osgOcean::OceanTechnique* getOceanSurface(void)
  {
    return _FFToceanSurface.get();
  }

  inline osg::Group* getScene(void)
  {
    return _scene.get();
  }

  inline osgOcean::OceanScene* getOceanScene()
  {
    return _oceanScene.get();
  }

  osg::Light* getLight()
  {
    return _light.get();
  }

  osg::Node* addObject(osg::Transform *transform, std::string filename, Object *o = NULL);

  void addObject(osg::Transform *transform);

};
