/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
*
* This library is open source and may be redistributed and/or modified under  
* the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
* (at your option) any later version.  The full license is in LICENSE file
* included with this distribution, and on the openscenegraph.org website.
* 
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
* OpenSceneGraph Public License for more details.
*/

#include <osgOcean/SiltEffect>
#include <osgOcean/ShaderManager>

#include <stdlib.h>
#include <OpenThreads/ScopedLock>

#include <osg/Texture2D>
#include <osg/PointSprite>
#include <osgUtil/CullVisitor>
#include <osgUtil/GLObjectsVisitor>

#include <osg/Notify>
#include <osg/io_utils>
#include <osg/Timer>
#include <osg/Version>

using namespace osgOcean;

static float random(float min,float max) { return min + (max-min)*(float)rand()/(float)RAND_MAX; }

static void fillSpotLightImage(unsigned char* ptr, const osg::Vec4& centerColour, const osg::Vec4& backgroudColour, unsigned int size, float power)
{
    if (size==1)
    {
        float r = 0.5f;
        osg::Vec4 color = centerColour*r+backgroudColour*(1.0f-r);
        *ptr++ = (unsigned char)((color[0])*255.0f);
        *ptr++ = (unsigned char)((color[1])*255.0f);
        *ptr++ = (unsigned char)((color[2])*255.0f);
        *ptr++ = (unsigned char)((color[3])*255.0f);
        return;
    }

    float mid = (float(size)-1.0f)*0.5f;
    float div = 2.0f/float(size);
    for(unsigned int r=0;r<size;++r)
    {
        //unsigned char* ptr = image->data(0,r,0);
        for(unsigned int c=0;c<size;++c)
        {
            float dx = (float(c) - mid)*div;
            float dy = (float(r) - mid)*div;
            float r = powf(1.0f-sqrtf(dx*dx+dy*dy),power);
            if (r<0.0f) r=0.0f;
            osg::Vec4 color = centerColour*r+backgroudColour*(1.0f-r);
            *ptr++ = (unsigned char)((color[0])*255.0f);
            *ptr++ = (unsigned char)((color[1])*255.0f);
            *ptr++ = (unsigned char)((color[2])*255.0f);
            *ptr++ = (unsigned char)((color[3])*255.0f);
        }
    }
}

static osg::Image* createSpotLightImage(const osg::Vec4& centerColour, const osg::Vec4& backgroudColour, unsigned int size, float power)
{

#if 0
    osg::Image* image = new osg::Image;
    unsigned char* ptr = image->data(0,0,0);
    fillSpotLightImage(ptr, centerColour, backgroudColour, size, power);

    return image;
#else
    osg::Image* image = new osg::Image;
    osg::Image::MipmapDataType mipmapData;
    unsigned int s = size;
    unsigned int totalSize = 0;
    unsigned i;
    for(i=0; s>0; s>>=1, ++i)
    {
        if (i>0) mipmapData.push_back(totalSize);
        totalSize += s*s*4;
    }

    unsigned char* ptr = new unsigned char[totalSize];
    image->setImage(size, size, size, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, ptr, osg::Image::USE_NEW_DELETE,1);

    image->setMipmapLevels(mipmapData);

    s = size;
    for(i=0; s>0; s>>=1, ++i)
    {
        fillSpotLightImage(ptr, centerColour, backgroudColour, s, power);
        ptr += s*s*4;
    }

    return image;
#endif    
}


SiltEffect::SiltEffect()
{
    setNumChildrenRequiringUpdateTraversal(1);

    setUpGeometries(1920);

    setIntensity(0.5);
}

void SiltEffect::setIntensity(float intensity)
{
    _wind.set(0.0f,0.0f,0.0f);
    _particleSpeed = -0.75f - 0.25f*intensity;
    _particleSize = 0.02f + 0.03f*intensity;
    _particleColor = osg::Vec4(0.85f, 0.85f, 0.85f, 1.0f) -  osg::Vec4(0.1f, 0.1f, 0.1f, 1.0f)* intensity;
    _maximumParticleDensity = intensity * 8.2f;
    _cellSize.set(5.0f / (0.25f+intensity), 5.0f / (0.25f+intensity), 5.0f);
    _nearTransition = 25.f;
    _farTransition = 100.0f - 60.0f*sqrtf(intensity);

    if (!_fog) _fog = new osg::Fog;

    _fog->setMode(osg::Fog::EXP);
    _fog->setDensity(0.01f*intensity);
    _fog->setColor(osg::Vec4(0.6, 0.6, 0.6, 1.0));

    _dirty = true;

    update();
}

SiltEffect::SiltEffect(const SiltEffect& copy, const osg::CopyOp& copyop):
osg::Node(copy,copyop)
{
    setNumChildrenRequiringUpdateTraversal(getNumChildrenRequiringUpdateTraversal()+1);            
    _dirty = true;
    update();
}

void SiltEffect::compileGLObjects(osg::RenderInfo& renderInfo) const
{
    if (_quadGeometry.valid()) 
    {
        _quadGeometry->compileGLObjects(renderInfo);
        if (_quadGeometry->getStateSet()) _quadGeometry->getStateSet()->compileGLObjects(*renderInfo.getState());
    }

    if (_pointGeometry.valid()) 
    {
        _pointGeometry->compileGLObjects(renderInfo);
        if (_pointGeometry->getStateSet()) _pointGeometry->getStateSet()->compileGLObjects(*renderInfo.getState());
    }
}


void SiltEffect::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        if (_dirty) update();

        if (nv.getFrameStamp())
        {
            double currentTime = nv.getFrameStamp()->getSimulationTime();
            static double previousTime = currentTime;
            double delta = currentTime - previousTime;
            _origin += _wind * delta;
            previousTime = currentTime;
        }

        return;
    }

    if (nv.getVisitorType() == osg::NodeVisitor::NODE_VISITOR)
    {
        if (_dirty) update();

        osgUtil::GLObjectsVisitor* globjVisitor = dynamic_cast<osgUtil::GLObjectsVisitor*>(&nv);
        if (globjVisitor)
        {
            if (globjVisitor->getMode() & osgUtil::GLObjectsVisitor::COMPILE_STATE_ATTRIBUTES)
            {
                compileGLObjects(globjVisitor->getRenderInfo());
            }
        }

        return;
    }


    if (nv.getVisitorType() != osg::NodeVisitor::CULL_VISITOR)
    {
        return;
    }

    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
    if (!cv)
    {
        return;
    }

    ViewIdentifier viewIndentifier(cv, nv.getNodePath());
    {
        SiltDrawableSet* SiltDrawableSet = 0;

        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
            SiltDrawableSet = &(_viewDrawableMap[viewIndentifier]);

            if (!SiltDrawableSet->_quadSiltDrawable)
            {
                SiltDrawableSet->_quadSiltDrawable = new SiltDrawable;
                SiltDrawableSet->_quadSiltDrawable->setGeometry(_quadGeometry.get());
                SiltDrawableSet->_quadSiltDrawable->setStateSet(_quadStateSet.get());
                SiltDrawableSet->_quadSiltDrawable->setDrawType(GL_QUADS);

                SiltDrawableSet->_pointSiltDrawable = new SiltDrawable;
                SiltDrawableSet->_pointSiltDrawable->setGeometry(_pointGeometry.get());
                SiltDrawableSet->_pointSiltDrawable->setStateSet(_pointStateSet.get());
                SiltDrawableSet->_pointSiltDrawable->setDrawType(GL_POINTS);
            }
        }

        cull(*SiltDrawableSet, cv);

        cv->pushStateSet(_stateset.get());
        float depth = 0.0f;

        if (!SiltDrawableSet->_quadSiltDrawable->getCurrentCellMatrixMap().empty())
        {
            cv->pushStateSet(SiltDrawableSet->_quadSiltDrawable->getStateSet());
            cv->addDrawableAndDepth(SiltDrawableSet->_quadSiltDrawable.get(),cv->getModelViewMatrix(),depth);    
            cv->popStateSet();
        }

        if (!SiltDrawableSet->_pointSiltDrawable->getCurrentCellMatrixMap().empty())
        {
            cv->pushStateSet(SiltDrawableSet->_pointSiltDrawable->getStateSet());
            cv->addDrawableAndDepth(SiltDrawableSet->_pointSiltDrawable.get(),cv->getModelViewMatrix(),depth);    
            cv->popStateSet();
        }

        cv->popStateSet();
    }
}

void SiltEffect::update()
{
    _dirty = false;

    osg::notify(osg::INFO)<<"SiltEffect::update()"<<std::endl;

    float length_u = _cellSize.x();
    float length_v = _cellSize.y();
    float length_w = _cellSize.z();

    // time taken to get from start to the end of cycle
    _period = fabsf(_cellSize.z() / _particleSpeed);

    _du.set(length_u, 0.0f, 0.0f);
    _dv.set(0.0f, length_v, 0.0f);
    _dw.set(0.0f, 0.0f, length_w);

    _inverse_du.set(1.0f/length_u, 0.0f, 0.0f);
    _inverse_dv.set(0.0f, 1.0f/length_v, 0.0f);
    _inverse_dw.set(0.0f, 0.0f, 1.0f/length_w);

    osg::notify(osg::INFO)<<"Cell size X="<<length_u<<std::endl;
    osg::notify(osg::INFO)<<"Cell size Y="<<length_v<<std::endl;
    osg::notify(osg::INFO)<<"Cell size Z="<<length_w<<std::endl;

    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        _viewDrawableMap.clear();
    }    

    // set up state/
    {
        if (!_stateset)
        {
            _stateset = new osg::StateSet;
            _stateset->addUniform(new osg::Uniform("osgOcean_BaseTexture",0));

            _stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            _stateset->setMode(GL_BLEND, osg::StateAttribute::ON);

            osg::Texture2D* texture = new osg::Texture2D(createSpotLightImage(osg::Vec4(0.55f,0.55f,0.55f,0.65f),osg::Vec4(0.55f,0.55f,0.55f,0.0f),32,1.0));
            _stateset->setTextureAttribute(0, texture);
        }

        if (!_inversePeriodUniform)
        {
            _inversePeriodUniform = new osg::Uniform("osgOcean_InversePeriod",1.0f/_period);
            _stateset->addUniform(_inversePeriodUniform.get());
        }
        else _inversePeriodUniform->set(1.0f/_period);

        if (!_particleColorUniform)
        {
            _particleColorUniform = new osg::Uniform("osgOcean_ParticleColour", _particleColor);
            _stateset->addUniform(_particleColorUniform.get());
        }
        else _particleColorUniform->set(_particleColor);

        if (!_particleSizeUniform)
        {
            _particleSizeUniform = new osg::Uniform("osgOcean_ParticleSize", _particleSize);
            _stateset->addUniform(_particleSizeUniform.get());
        }
        else
            _particleSizeUniform->set(_particleSize);
    }
}

void SiltEffect::createGeometry(unsigned int numParticles, 
                                          osg::Geometry* quad_geometry, 
                                          osg::Geometry* point_geometry )
{
    // particle corner offsets
    osg::Vec2 offset00(0.0f,0.0f);
    osg::Vec2 offset10(1.0f,0.0f);
    osg::Vec2 offset01(0.0f,1.0f);
    osg::Vec2 offset11(1.0f,1.0f);

    osg::Vec2 offset0(0.5f,0.0f);
    osg::Vec2 offset1(0.5f,1.0f);

    osg::Vec2 offset(0.5f,0.5f);

    // configure quad_geometry;
    osg::Vec3Array* quad_vertices = 0;
    osg::Vec2Array* quad_offsets = 0;
    osg::Vec3Array* quad_vectors = 0;

    if (quad_geometry)
    {
        quad_geometry->setName("quad");

        quad_vertices = new osg::Vec3Array(numParticles*4);
        quad_offsets  = new osg::Vec2Array(numParticles*4);
        quad_vectors  = new osg::Vec3Array(numParticles*4);

        quad_geometry->setVertexArray(quad_vertices);
        quad_geometry->setTexCoordArray(0, quad_offsets);
        quad_geometry->setNormalArray(quad_vectors);
        quad_geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    }

    // configure point_geometry;
    osg::Vec3Array* point_vertices = 0;
    osg::Vec2Array* point_offsets = 0;
    osg::Vec3Array* point_vectors = 0;

    if (point_geometry)
    {
        point_geometry->setName("point");

        point_vertices = new osg::Vec3Array(numParticles);
        point_offsets = new osg::Vec2Array(numParticles);
        point_vectors = new osg::Vec3Array(numParticles);

        point_geometry->setVertexArray(point_vertices);
        point_geometry->setTexCoordArray(0, point_offsets);
        point_geometry->setNormalArray(point_vectors);
        point_geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    }

    // set up vertex attribute data.
    for(unsigned int i=0; i< numParticles; ++i)
    {
        osg::Vec3 pos( random(0.0f, 1.0f), random(0.0f, 1.0f), random(0.0f, 1.0f));
        osg::Vec3 dir( random(-1.f, 1.f), random(-1.f,1.f), random(-1.f,1.f) );

        // quad particles
        if (quad_vertices)
        {
            (*quad_vertices)[i*4]   = pos;
            (*quad_vertices)[i*4+1] = pos;
            (*quad_vertices)[i*4+2] = pos;
            (*quad_vertices)[i*4+3] = pos;

            (*quad_offsets)[i*4]   = offset00;
            (*quad_offsets)[i*4+1] = offset01;
            (*quad_offsets)[i*4+2] = offset11;
            (*quad_offsets)[i*4+3] = offset10;

            (*quad_vectors)[i*4]   = dir;
            (*quad_vectors)[i*4+1] = dir;
            (*quad_vectors)[i*4+2] = dir;
            (*quad_vectors)[i*4+3] = dir;
        }
        // point particles
        if (point_vertices)
        {
            (*point_vertices)[i] = pos;
            (*point_offsets)[i]  = offset;
            (*point_vectors)[i]  = dir;
        }
    }
}

#include <osgOcean/shaders/osgOcean_silt_quads_vert.inl>
#include <osgOcean/shaders/osgOcean_silt_quads_frag.inl>
#include <osgOcean/shaders/osgOcean_silt_points_vert.inl>
#include <osgOcean/shaders/osgOcean_silt_points_frag.inl>

void SiltEffect::setUpGeometries(unsigned int numParticles)
{
    unsigned int quadRenderBin = 12;
    unsigned int pointRenderBin = 11;

    osg::notify(osg::INFO)<<"SiltEffect::setUpGeometries("<<numParticles<<")"<<std::endl;

    bool needGeometryRebuild = false;

    if (!_quadGeometry || _quadGeometry->getVertexArray()->getNumElements() != 4*numParticles)
    {
        _quadGeometry = new osg::Geometry;
        _quadGeometry->setUseVertexBufferObjects(true);
        needGeometryRebuild = true;
    }

    if (!_pointGeometry || _pointGeometry->getVertexArray()->getNumElements() != numParticles)
    {
        _pointGeometry = new osg::Geometry;
        _pointGeometry->setUseVertexBufferObjects(true);
        needGeometryRebuild = true;
    }

    if (needGeometryRebuild) 
    {
        createGeometry(numParticles, _quadGeometry.get(), _pointGeometry.get());
    }

    if (!_quadStateSet)
    {
        _quadStateSet = new osg::StateSet;

        _quadStateSet->setRenderBinDetails(quadRenderBin,"DepthSortedBin");

        static const char osgOcean_silt_quads_vert_file[] = "osgOcean_silt_quads.vert";
	    static const char osgOcean_silt_quads_frag_file[] = "osgOcean_silt_quads.frag";

        osg::Program* program = 
            ShaderManager::instance().createProgram("silt_quads", 
                                                    osgOcean_silt_quads_vert_file, osgOcean_silt_quads_frag_file, 
                                                    osgOcean_silt_quads_vert,      osgOcean_silt_quads_frag );
        _quadStateSet->setAttribute(program);
    }

    if (!_pointStateSet)
    {
        _pointStateSet = new osg::StateSet;

        static const char osgOcean_silt_points_vert_file[] = "osgOcean_silt_points.vert";
	    static const char osgOcean_silt_points_frag_file[] = "osgOcean_silt_points.frag";

        osg::Program* program = 
            ShaderManager::instance().createProgram("silt_point", 
                                                    osgOcean_silt_points_vert_file, osgOcean_silt_points_frag_file, 
                                                    osgOcean_silt_points_vert,      osgOcean_silt_points_frag );
        _pointStateSet->setAttribute(program);

        /// Setup the point sprites
        osg::PointSprite *sprite = new osg::PointSprite();
        _pointStateSet->setTextureAttributeAndModes(0, sprite, osg::StateAttribute::ON);

        _pointStateSet->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
        _pointStateSet->setRenderBinDetails(pointRenderBin,"DepthSortedBin");
    }
}

void SiltEffect::cull(SiltDrawableSet& pds, osgUtil::CullVisitor* cv) const
{
#ifdef DO_TIMING
    osg::Timer_t startTick = osg::Timer::instance()->tick();
#endif

    float cellVolume = _cellSize.x() * _cellSize.y() * _cellSize.z();
    int numberOfParticles = (int)(_maximumParticleDensity * cellVolume);

    if (numberOfParticles==0) 
        return;

    pds._quadSiltDrawable->setNumberOfVertices(numberOfParticles*4);
    pds._pointSiltDrawable->setNumberOfVertices(numberOfParticles);

    pds._quadSiltDrawable->newFrame();
    pds._pointSiltDrawable->newFrame();

    osg::Matrix inverse_modelview;
    inverse_modelview.invert(*(cv->getModelViewMatrix()));

    osg::Vec3 eyeLocal = osg::Vec3(0.0f,0.0f,0.0f) * inverse_modelview;
    //osg::notify(osg::NOTICE)<<"  eyeLocal "<<eyeLocal<<std::endl;

    float eye_k = (eyeLocal-_origin)*_inverse_dw;
    osg::Vec3 eye_kPlane = eyeLocal-_dw*eye_k-_origin;

    // osg::notify(osg::NOTICE)<<"  eye_kPlane "<<eye_kPlane<<std::endl;

    float eye_i = eye_kPlane*_inverse_du;
    float eye_j = eye_kPlane*_inverse_dv;

    osg::Polytope frustum;
    frustum.setToUnitFrustum(false,false);
    frustum.transformProvidingInverse(*(cv->getProjectionMatrix()));
    frustum.transformProvidingInverse(*(cv->getModelViewMatrix()));

    float i_delta = _farTransition * _inverse_du.x();
    float j_delta = _farTransition * _inverse_dv.y();
    float k_delta = 1;//_nearTransition * _inverse_dw.z();

    int i_min = (int)floor(eye_i - i_delta);
    int j_min = (int)floor(eye_j - j_delta);
    int k_min = (int)floor(eye_k - k_delta);

    int i_max = (int)ceil(eye_i + i_delta);
    int j_max = (int)ceil(eye_j + j_delta);
    int k_max = (int)ceil(eye_k + k_delta);

    //osg::notify(osg::NOTICE)<<"i_delta="<<i_delta<<" j_delta="<<j_delta<<" k_delta="<<k_delta<<std::endl;

    unsigned int numTested=0;
    unsigned int numInFrustum=0;

    float iCyle = 0.43;
    float jCyle = 0.64;

    for(int i = i_min; i<=i_max; ++i)
    {
        for(int j = j_min; j<=j_max; ++j)
        {
            for(int k = k_min; k<=k_max; ++k)
            {
                float startTime = (float)(i)*iCyle + (float)(j)*jCyle;
                startTime = (startTime-floor(startTime))*_period;

                if (build(eyeLocal, i,j,k, startTime, pds, frustum, cv)) 
                    ++numInFrustum;
                
                ++numTested;
            }
        }
    }


#ifdef DO_TIMING
    osg::Timer_t endTick = osg::Timer::instance()->tick();

    osg::notify(osg::NOTICE)<<"time for cull "<<osg::Timer::instance()->delta_m(startTick,endTick)<<"ms  numTested= "<<numTested<<" numInFrustum= "<<numInFrustum<<std::endl;
    osg::notify(osg::NOTICE)<<"     quads "<<pds._quadSiltDrawable->getCurrentCellMatrixMap().size()<<"   points "<<pds._pointSiltDrawable->getCurrentCellMatrixMap().size()<<std::endl;
#endif
}

bool SiltEffect::build(const osg::Vec3 eyeLocal, int i, int j, int k, float startTime, SiltDrawableSet& pds, osg::Polytope& frustum, osgUtil::CullVisitor* cv) const
{
    osg::Vec3 position = _origin + osg::Vec3(float(i)*_du.x(), float(j)*_dv.y(), float(k+1)*_dw.z());
    osg::Vec3 scale(_du.x(), _dv.y(), -_dw.z());

    osg::BoundingBox bb(position.x(), position.y(), position.z()+scale.z(),
        position.x()+scale.x(), position.y()+scale.y(), position.z());

    if ( !frustum.contains(bb) ) 
        return false;

    osg::Vec3 center = position + scale*0.5f;
    float distance = (center-eyeLocal).length();

    osg::Matrix* mymodelview = 0;

    if (distance < _nearTransition)
    {
        SiltDrawable::DepthMatrixStartTime& mstp 
            = pds._quadSiltDrawable->getCurrentCellMatrixMap()[SiltDrawable::Cell(i,k,j)];
        
        mstp.depth = distance;
        mstp.startTime = startTime;
        mymodelview = &mstp.modelview;
    }
    else if (distance <= _farTransition)
    {
        SiltDrawable::DepthMatrixStartTime& mstp 
            = pds._pointSiltDrawable->getCurrentCellMatrixMap()[SiltDrawable::Cell(i,k,j)];
        
        mstp.depth = distance;
        mstp.startTime = startTime;
        mymodelview = &mstp.modelview;
    }
    else
    {
        return false;
    }

    *mymodelview = *(cv->getModelViewMatrix());

#if OPENSCENEGRAPH_MAJOR_VERSION > 2 || \
    (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION > 7) || \
    (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION == 7 && OPENSCENEGRAPH_PATCH_VERSION >= 3)

    // preMultTranslate and preMultScale introduced in rev 8868, which was 
    // before OSG 2.7.3.
    mymodelview->preMultTranslate(position);
    mymodelview->preMultScale(scale);

#else

    // Otherwise use unoptimized versions
    mymodelview->preMult(osg::Matrix::translate(position));
    mymodelview->preMult(osg::Matrix::scale(scale));

#endif

    cv->updateCalculatedNearFar(*(cv->getModelViewMatrix()),bb);

    return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//   Precipitation Drawable
//
////////////////////////////////////////////////////////////////////////////////////////////////////

SiltEffect::SiltDrawable::SiltDrawable():
_drawType(GL_QUADS),
_numberOfVertices(0)
{
    setSupportsDisplayList(false);
}

SiltEffect::SiltDrawable::SiltDrawable(const SiltDrawable& copy, const osg::CopyOp& copyop):
osg::Drawable(copy,copyop),
_geometry(copy._geometry),
_drawType(copy._drawType),
_numberOfVertices(copy._numberOfVertices)
{
}



void SiltEffect::SiltDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    if (!_geometry) return;

    const osg::Geometry::Extensions* extensions = osg::Geometry::getExtensions(renderInfo.getContextID(),true);

    glPushMatrix();

    typedef std::vector<const CellMatrixMap::value_type*> DepthMatrixStartTimeVector;
    DepthMatrixStartTimeVector orderedEntries;
    orderedEntries.reserve(_currentCellMatrixMap.size());

    for(CellMatrixMap::const_iterator citr = _currentCellMatrixMap.begin();
        citr != _currentCellMatrixMap.end();
        ++citr)
    {
        orderedEntries.push_back(&(*citr));
    }

    std::sort(orderedEntries.begin(),orderedEntries.end(),LessFunctor());

    for(DepthMatrixStartTimeVector::reverse_iterator itr = orderedEntries.rbegin();
        itr != orderedEntries.rend();
        ++itr)
    {
        extensions->glMultiTexCoord1f(GL_TEXTURE0+1, (*itr)->second.startTime);

        glMatrixMode( GL_MODELVIEW );
        glLoadMatrix((*itr)->second.modelview.ptr());

        _geometry->draw(renderInfo);

        unsigned int numVertices = osg::minimum(_geometry->getVertexArray()->getNumElements(), _numberOfVertices);
        glDrawArrays(_drawType, 0, numVertices);
    }

    glPopMatrix();
}
