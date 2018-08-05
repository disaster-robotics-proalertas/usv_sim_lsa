/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */

#ifndef SCENE_EVENT_HANDLER
#define SCENE_EVENT_HANDLER

#include "SimulatedIAUV.h"
#include "ConfigXMLParser.h"
#include "TrajectoryVisualization.h"

class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
  osg::ref_ptr<osgOceanScene> _scene;
  SceneBuilder * _sceneBuilder;
  osg::ref_ptr<TextHUD> _textHUD;
  std::vector<osg::ref_ptr<osgWidget::Window> > _windows;
  ConfigFile *_config;

  bool draw_frames_;
public:
  //vehicle track indicates whether the camera must automatically track the vehicle node
  SceneEventHandler(std::vector<osg::ref_ptr<osgWidget::Window> > &windows, TextHUD* textHUD,
                    SceneBuilder * sceneBuilder, ConfigFile *config) :
      _scene(sceneBuilder->getScene()), _sceneBuilder(sceneBuilder), _textHUD(textHUD), _windows(windows), draw_frames_(false)
  {
    _textHUD->setSceneText("Clear Blue Sky");
    _config = config;
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
  {
    switch (ea.getEventType())
    {
      case (osgGA::GUIEventAdapter::KEYUP):
      {

        if (ea.getKey() == '1')
        {
          ROS_INFO("Switch to Clear Blue Sky");
          _scene->changeScene(osgOceanScene::CLEAR);
          _scene->getOceanScene()->setUnderwaterFog(
              _config->fogDensity, osg::Vec4f(_config->fogColor[0], _config->fogColor[1], _config->fogColor[2], 1));
          _scene->getOceanScene()->setUnderwaterDiffuse(
              osg::Vec4f(_config->color[0], _config->color[1], _config->color[2], 1));
          _scene->getOceanScene()->setUnderwaterAttenuation(
              osg::Vec3f(_config->attenuation[0], _config->attenuation[1], _config->attenuation[2]));

          _textHUD->setSceneText("Clear Blue Sky");
          return false;
        }
        else if (ea.getKey() == '2')
        {
          _scene->changeScene(osgOceanScene::DUSK);
          _textHUD->setSceneText("Dusk");
          _scene->getOceanScene()->setUnderwaterFog(
              _config->fogDensity, osg::Vec4f(_config->fogColor[0], _config->fogColor[1], _config->fogColor[2], 1));
          _scene->getOceanScene()->setUnderwaterDiffuse(
              osg::Vec4f(_config->color[0], _config->color[1], _config->color[2], 1));
          _scene->getOceanScene()->setUnderwaterAttenuation(
              osg::Vec3f(_config->attenuation[0], _config->attenuation[1], _config->attenuation[2]));
          return false;
        }
        else if (ea.getKey() == '3')
        {
          _scene->changeScene(osgOceanScene::CLOUDY);
          _textHUD->setSceneText("Pacific Cloudy");
          _scene->getOceanScene()->setUnderwaterFog(
              _config->fogDensity, osg::Vec4f(_config->fogColor[0], _config->fogColor[1], _config->fogColor[2], 1));
          _scene->getOceanScene()->setUnderwaterDiffuse(
              osg::Vec4f(_config->color[0], _config->color[1], _config->color[2], 1));
          _scene->getOceanScene()->setUnderwaterAttenuation(
              osg::Vec3f(_config->attenuation[0], _config->attenuation[1], _config->attenuation[2]));
          return false;
        }
        else if (ea.getKey() == 'c')
        {
          for (unsigned int i = 0; i < _windows.size(); i++)
            if (_windows[i]->isVisible())
              _windows[i]->hide();
            else
              _windows[i]->show();
          return false;
        }
        else if (ea.getKey() == 'f')
        {
          //Search for 'switch_frames' nodes and toggle their values
          findNodeVisitor finder("switch_frames");
          _scene->localizedWorld->accept(finder);
          std::vector<osg::Node*> node_list = finder.getNodeList();
          (draw_frames_) ? draw_frames_ = false : draw_frames_ = true;
          for (unsigned int i = 0; i < node_list.size(); i++)
            (draw_frames_) ? node_list[i]->asSwitch()->setAllChildrenOn() :
                node_list[i]->asSwitch()->setAllChildrenOff();
        }
        else if (ea.getKey() == 't')
        {
          //Search for 'switch_trajectory' nodes and toggle their values
          findNodeVisitor finder("switch_trajectory");
          _scene->localizedWorld->accept(finder);
          std::vector<osg::Node*> node_list = finder.getNodeList();
          (draw_frames_) ? draw_frames_ = false : draw_frames_ = true;
          for (unsigned int i = 0; i < node_list.size(); i++)
            (draw_frames_) ? node_list[i]->asSwitch()->setAllChildrenOn() :
                node_list[i]->asSwitch()->setAllChildrenOff();
        }
        else if (ea.getKey() == 'r')
        {
          //search catchable objects and get them back to their original positions
          for (unsigned int i = 0; i < _sceneBuilder->objects.size(); i++)
          {

            osg::ref_ptr<NodeDataType> data = dynamic_cast<NodeDataType*>(_sceneBuilder->objects[i]->getUserData());
            if(data->catchable)
            { //No need to restart static objects
              osg::Matrixd matrix;
              matrix.makeRotate(
                osg::Quat(data->originalRotation[0], osg::Vec3d(1, 0, 0), data->originalRotation[1],
                          osg::Vec3d(0, 1, 0), data->originalRotation[2], osg::Vec3d(0, 0, 1)));
              matrix.setTrans(data->originalPosition[0], data->originalPosition[1], data->originalPosition[2]);

              if(!data->rigidBody)
              { //No physics
                _sceneBuilder->objects[i]->getParent(0)->getParent(0)->asTransform()->asMatrixTransform()->setMatrix(matrix);
 
                //just in case an object picker picked it
                _scene->localizedWorld->addChild(_sceneBuilder->objects[i]->getParent(0)->getParent(0));
                _sceneBuilder->objects[i]->getParent(0)->getParent(0)->getParent(0)->removeChild(_sceneBuilder->objects[i]->getParent(0)->getParent(0));
              }
              if(data->rigidBody)
              {//Physics restart
                
                //Reset position for kinematic & static objects
                _sceneBuilder->objects[i]->getParent(0)->getParent(0)->asTransform()->asMatrixTransform()->setMatrix(matrix);
                _scene->localizedWorld->addChild(_sceneBuilder->objects[i]->getParent(0)->getParent(0));
                _sceneBuilder->objects[i]->getParent(0)->getParent(0)->getParent(0)->removeChild(_sceneBuilder->objects[i]->getParent(0)->getParent(0));

                //Get object position in OSG world to move it to Bullet
                boost::shared_ptr<osg::Matrix> mat = getWorldCoords(_sceneBuilder->objects[i]->getParent(0)->getParent(0));

                //Unset STATIC flag (catched objects)
                data->rigidBody->setCollisionFlags(
                    data->rigidBody->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT );

                //Reset dynamic properties
                data->rigidBody->setCenterOfMassTransform(osgbCollision::asBtTransform(*mat));
                data->rigidBody->clearForces();
                data->rigidBody->setLinearVelocity(btVector3(0,0,0));
                data->rigidBody->setAngularVelocity(btVector3(0,0,0));
              }
            }
          }

          //Search for vehicles and set them back to initial pose

          for (unsigned int i = 0; i < _sceneBuilder->iauvFile.size(); i++)
          {
            for (std::list<Vehicle>::iterator cfgVehicle = _config->vehicles.begin(); cfgVehicle != _config->vehicles.end(); cfgVehicle++)
              if (cfgVehicle->name == _sceneBuilder->iauvFile[i]->name){
                _sceneBuilder->iauvFile[i]->setVehiclePosition(cfgVehicle->position[0], cfgVehicle->position[1],
                  cfgVehicle->position[2], cfgVehicle->orientation[0],cfgVehicle->orientation[1], cfgVehicle->orientation[2]);

                _sceneBuilder->iauvFile[i]->urdf->setJointPosition(cfgVehicle->jointValues);
              }

            //restart object pickers
            for (unsigned int j = 0; j<_sceneBuilder->iauvFile[i]->object_pickers.size();j++){
              osg::ref_ptr<ObjectPickerUpdateCallback> callback =
                dynamic_cast<ObjectPickerUpdateCallback*>(_sceneBuilder->iauvFile[i]->object_pickers[j].trackNode->getUpdateCallback());
              callback->picked = false;
            }
          }

          //Search for trajectory updaters and clearwaypoints?
        }
        else if (ea.getKey() == 'y') 
        { //Clear trajectories
          for (unsigned int i = 0; i < _sceneBuilder->trajectories.size(); i++)
          {
            osg::ref_ptr<TrajectoryUpdateCallback> callback =
              dynamic_cast<TrajectoryUpdateCallback*>(_sceneBuilder->trajectories[i]->getUpdateCallback()); 
            callback->reset();
          }
        }

      }
      default:
        return false;
    }
  }

  void getUsage(osg::ApplicationUsage& usage) const
  {
  }

};

#endif
