#pragma once
#include <osgGA/GUIEventHandler>
#include <osgGA/FlightManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/DriveManipulator>

#include "TextHUD.h"
#include "Scene.h"

// ----------------------------------------------------
//                   Event Handler
// ----------------------------------------------------

class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
    osg::ref_ptr<Scene>   _scene;
    osg::ref_ptr<TextHUD> _textHUD;
    osgViewer::View*      _view;
    osg::Vec3             _initialCameraPosition;

    enum CameraMode
    {
        FIXED,
        FLIGHT,
        TRACKBALL
    };

    CameraMode _currentCameraMode;

public:
    SceneEventHandler( Scene* scene, TextHUD* textHUD, osgViewer::View* view, const osg::Vec3& initialCameraPosition )
        :_scene                (scene)
        ,_textHUD              (textHUD)
        ,_view                 (view)
        ,_currentCameraMode    (FIXED)
        ,_initialCameraPosition(initialCameraPosition)
      {
          _textHUD->setSceneText("Clear");
          _textHUD->setCameraText("FIXED");

          osg::Vec3f eye(_initialCameraPosition);
          osg::Vec3f centre = eye+osg::Vec3f(0.f,1.f,0.f);
          osg::Vec3f up(0.f, 0.f, 1.f);

          _view->getCamera()->setViewMatrixAsLookAt( eye, centre, up    );
      }

      virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
      {
          switch(ea.getEventType())
          {
          case(osgGA::GUIEventAdapter::KEYUP):
              {
                  if(ea.getKey() == '1')
                  {
                      _scene->changeScene( Scene::CLEAR );
                      _textHUD->setSceneText( "Clear Blue Sky" );
                      return false;
                  }
                  else if(ea.getKey() == '2')
                  {
                      _scene->changeScene( Scene::DUSK );
                      _textHUD->setSceneText( "Dusk" );
                      return false;
                  }
                  else if(ea.getKey() == '3' )
                  {
                      _scene->changeScene( Scene::CLOUDY );
                      _textHUD->setSceneText( "Pacific Cloudy" );
                      return false;
                  }
                  else if(ea.getKey() == 'C' || ea.getKey() == 'c' )
                  {
                      osg::Vec3 eye(_initialCameraPosition);
                      if (_currentCameraMode == FIXED)
                      {
                          _currentCameraMode = FLIGHT;
                          osgGA::FlightManipulator* flight = new osgGA::FlightManipulator;
                          flight->setHomePosition( eye, eye + osg::Vec3(0,1,0), osg::Vec3f(0,0,1) );
                          _view->setCameraManipulator( flight );
                          _textHUD->setCameraText("FLIGHT");
                      }
                      else if (_currentCameraMode == FLIGHT)
                      {
                          _currentCameraMode = TRACKBALL;
                          osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
                          tb->setHomePosition( eye, eye + osg::Vec3(0,20,0), osg::Vec3f(0,0,1) );
                          _view->setCameraManipulator( tb );
                          _textHUD->setCameraText("TRACKBALL");
                      }
                      else if (_currentCameraMode == TRACKBALL)
                      {
                          _currentCameraMode = FIXED;
                          _view->getCamera()->setViewMatrixAsLookAt( eye, eye + osg::Vec3(0,1,0), osg::Vec3f(0,0,1) );
                          _view->setCameraManipulator(NULL);
                          _textHUD->setCameraText("FIXED");
                      }
                  }
              }
          default:
              return false;
          }
      }

      void getUsage(osg::ApplicationUsage& usage) const
      {
          usage.addKeyboardMouseBinding("c","Camera type (cycle through Fixed, Flight, Trackball)");
          usage.addKeyboardMouseBinding("1","Select scene \"Clear Blue Sky\"");
          usage.addKeyboardMouseBinding("2","Select scene \"Dusk\"");
          usage.addKeyboardMouseBinding("3","Select scene \"Pacific Cloudy\"");
      }
};