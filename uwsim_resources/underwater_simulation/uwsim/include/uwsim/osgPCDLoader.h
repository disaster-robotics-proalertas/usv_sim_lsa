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

#ifndef OSGPCDLOADER_H_
#define OSGPCDLOADER_H_

#include "SimulatorConfig.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>

template<class T>
  class osgPCDLoader
  {
  private:
    osg::ref_ptr<osg::Geometry> geometry;
    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec4Array> colors;

  public:
    pcl::PointCloud<T> cloud;
    osg::ref_ptr<osg::Geode> geode;

    osgPCDLoader(std::string pcd_file)
    {
      if (pcl::io::loadPCDFile < T > (pcd_file, cloud) == -1) //* load the file
      {
        std::cerr << "Couldn't read file " << pcd_file << std::endl;
      }
      else
      {
        ROS_INFO_STREAM("Loaded " << cloud.width * cloud.height << " data points from " << pcd_file);
        createGeode(cloud);
      }
    }

    osgPCDLoader(const pcl::PointCloud<T> &c)
    {
      createGeode(c);
    }

    void createGeode(const pcl::PointCloud<T> &cloud);

    osg::Geode *getGeode()
    {
      return geode.get();
    }

    ~osgPCDLoader()
    {
    }

  };

template<>
  void osgPCDLoader<pcl::PointXYZRGB>::createGeode(const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
  {
    geode = osg::ref_ptr < osg::Geode > (new osg::Geode());
    geometry = osg::ref_ptr < osg::Geometry > (new osg::Geometry());
    vertices = osg::ref_ptr < osg::Vec3Array > (new osg::Vec3Array());
    colors = osg::ref_ptr < osg::Vec4Array > (new osg::Vec4Array());

    //Read vertex and color info from PCD
    for (int i = 0; i < cloud.points.size(); i++)
    {
      vertices->push_back(osg::Vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));

      uint32_t rgb_val_;
      memcpy(&rgb_val_, &(cloud.points[i].rgb), sizeof(uint32_t));

      uint32_t red, green, blue;
      blue = rgb_val_ & 0x000000ff;
      rgb_val_ = rgb_val_ >> 8;
      green = rgb_val_ & 0x000000ff;
      rgb_val_ = rgb_val_ >> 8;
      red = rgb_val_ & 0x000000ff;

      colors->push_back(osg::Vec4f((float)red / 255.0f, (float)green / 255.0f, (float)blue / 255.0f, 1.0f));
    }

    //Set OSG Geometry vertex and colors
    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

    geode->addDrawable(geometry.get());
    osg::StateSet* state = geometry->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::Point *point = new osg::Point;
    point->setSize(2);
    state->setAttribute(point);
  }

template<>
  void osgPCDLoader<pcl::PointXYZ>::createGeode(const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    geode = osg::ref_ptr < osg::Geode > (new osg::Geode());
    geometry = osg::ref_ptr < osg::Geometry > (new osg::Geometry());
    vertices = osg::ref_ptr < osg::Vec3Array > (new osg::Vec3Array());
    colors = osg::ref_ptr < osg::Vec4Array > (new osg::Vec4Array());

    //Read vertex and color info from PCD
    for (int i = 0; i < cloud.points.size(); i++)
    {
      vertices->push_back(osg::Vec3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));
      colors->push_back(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
    }

    //Set OSG Geometry vertex and colors
    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

    geode->addDrawable(geometry);
    osg::StateSet* state = geometry->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::Point *point = new osg::Point;
    point->setSize(2);
    state->setAttribute(point);
  }

#endif

