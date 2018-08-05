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

#include <uwsim/SphereSegment.h>

SphereSegment::SphereSegment(void)
{
}

SphereSegment::SphereSegment(float radius, unsigned int longitudeSteps, unsigned int lattitudeSteps, float longStart,
                             float longEnd, float latStart, float latEnd)
{
  compute(radius, longitudeSteps, lattitudeSteps, longStart, longEnd, latStart, latEnd);
}

SphereSegment::SphereSegment(const SphereSegment& copy, const osg::CopyOp& copyop) :
    osg::Geode(copy, copyop)
{
}

SphereSegment::~SphereSegment(void)
{
}

// 0 >= longStart/longEnd <= 180
// 0 >= latStart/latEnd <= 360

void SphereSegment::compute(float radius, unsigned int longitudeSteps, unsigned int lattitudeSteps, float longStart,
                            float longEnd, float latStart, float latEnd)

{
  removeDrawables(0, getNumDrawables());

  osg::Vec3Array* vertices = new osg::Vec3Array();
  osg::Vec2Array* texcoords = new osg::Vec2Array();

  double x, y, z, t, p, sin_t, cos_t;

  double longInc = (longEnd - longStart) / (double)longitudeSteps;
  double latInc = (latEnd - latStart) / (double)lattitudeSteps;

  double theta = longStart, phi = latStart;

  float uScale = 1.f / longitudeSteps;
  float vScale = 1.f / lattitudeSteps;

  for (unsigned int i = 0; i <= longitudeSteps; ++i)
  {
    t = osg::DegreesToRadians(theta);
    sin_t = sin(t);
    cos_t = cos(t);

    for (unsigned int j = 0; j <= lattitudeSteps; ++j)
    {
      p = osg::DegreesToRadians(phi);

      x = radius * sin_t * cos(p);
      y = radius * sin_t * sin(p);
      z = radius * cos_t;

      vertices->push_back(osg::Vec3(x, y, z));
      texcoords->push_back(osg::Vec2(j * vScale, i * uScale));

      phi += latInc;
    }

    theta -= longInc;
    phi = latStart;
  }

  osg::ref_ptr < osg::Geometry > geom = new osg::Geometry();

  for (unsigned int r = 0; r <= longitudeSteps - 1; r += 1)
  {
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);

    for (unsigned int c = 0; c <= lattitudeSteps; c += 1)
    {
      indices->push_back(idx(r, c, lattitudeSteps + 1));
      indices->push_back(idx(r + 1, c, lattitudeSteps + 1));
    }

    geom->addPrimitiveSet(indices);
  }

  osg::Vec4Array* colors = new osg::Vec4Array();
  colors->push_back(osg::Vec4(1.f, 1.f, 1.f, 1.f));

  geom->setVertexArray(vertices);
  geom->setTexCoordArray(0, texcoords);
  geom->setColorArray(colors);
  geom->setColorBinding(osg::Geometry::BIND_OVERALL);

  addDrawable(geom.get());
}

osg::Vec2 SphereSegment::sphereMap(osg::Vec3& vertex, float radius)
{
  float u, v;

  float TWOPI = osg::PI * 2.f;

  v = acos(vertex.y() / radius) / osg::PI;

  if (vertex.z() >= 0.f)
    u = acos(vertex.x() / (radius * sin(osg::PI * v))) / TWOPI;
  else
    u = (osg::PI + acos(vertex.x() / (radius * sin(osg::PI * v)))) / TWOPI;

  return osg::Vec2(u, v);
}
