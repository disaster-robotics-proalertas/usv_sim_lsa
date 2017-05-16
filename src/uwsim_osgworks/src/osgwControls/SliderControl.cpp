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

#include "osgwControls/SliderControl.h"
#include <osg/NodeCallback>
#include <osgGA/EventVisitor>

#include <stdlib.h>


using namespace osgwControls;

/* \cond */
class SliderCallback :public osg::NodeCallback
{
public:
    SliderCallback(SliderControl* sc){_sc = sc;}

    virtual void operator ()(osg::Node* node, osg::NodeVisitor* nv)
    {
        _sc->update(nv->getFrameStamp()->getReferenceTime());
        //osg::notify(osg::ALWAYS)<<"Visited"<<std::endl;
        traverse( node, nv );
    }
protected:
    SliderControl* _sc;

};

class TextCallback :public osg::NodeCallback
{
public:
    TextCallback(SliderControl* sc){_sc = sc;}

    virtual void operator ()(osg::Node* node, osg::NodeVisitor* nv)
    {
        //osg::notify(osg::ALWAYS)<<"visited"<<std::endl;
        osg::Geode* g = dynamic_cast<osg::Geode*> (node);
        osgText::Text* t = dynamic_cast<osgText::Text*> (g->getDrawable(0));
        char text [16];
        sprintf(text, "%.2f",_sc->getCurrentValue());
        t->setText(text);
        
        traverse( node, nv );
    }
protected:
    SliderControl* _sc;

};

class ButtonPickHandler : public osgGA::GUIEventHandler
{   
public:
    ButtonPickHandler(SliderControl* sc){_sc = sc;}
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>( &aa);
        if(!viewer) return false;
        switch(ea.getEventType())
        {
        case osgGA::GUIEventAdapter::PUSH:
        case osgGA::GUIEventAdapter::MOVE:
            {
                _mx = ea.getX();
                _my = ea.getY();
                return false;
            }
        case osgGA::GUIEventAdapter::RELEASE:
            {
                if(ea.getX()==_mx && ea.getY()==_my)
                {
                    if(pick(ea.getXnormalized(),ea.getYnormalized(),viewer))return true;
                }
                return false;
            }
        default:
            return false;
        }
    }
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>(nv);
        if (ev && ev->getActionAdapter() && !ev->getEvents().empty())
        {
            for(osgGA::EventQueue::Events::iterator itr = ev->getEvents().begin();
                itr != ev->getEvents().end();
                ++itr)
            {
                    handleWithCheckAgainstIgnoreHandledEventsMask(*(*itr), *(ev->getActionAdapter()), node, nv);
            }
        }
        traverse(node, nv);
    }
private:
    double _mx, _my;
    SliderControl* _sc;
    bool pick(const double x, const double y, osgViewer::Viewer* v)
    {
        //osg::notify(osg::ALWAYS)<<"Clicked: x: "<<x<<" y: "<<y<<"\n";
        if(!v->getSceneData())return false;
        double w(.005), h(.005);

        osgUtil::PolytopeIntersector* picker = 
            new osgUtil::PolytopeIntersector(osgUtil::Intersector::PROJECTION,x-w,y-h,x+w,y+h);
        osgUtil::IntersectionVisitor iv(picker);
        v->getCamera()->accept(iv);
        if(picker->containsIntersections())
        {
            const osg::NodePath& nodePath = picker->getFirstIntersection().nodePath;
            unsigned int idx = nodePath.size();

            while(idx--)
            {    
                osg::Geode* g = dynamic_cast<osg::Geode*>(nodePath[idx]);
                if(g==NULL)continue;
                if(g->getName()=="Forward")
                {
                    if(_sc->getPlayMode()==SliderControl::FORWARD) 
                        _sc->setPlayMode(SliderControl::STOP);
                    else _sc->setPlayMode(SliderControl::FORWARD);
                }
                else if(g->getName()=="Reverse")
                {
                    if(_sc->getPlayMode()==SliderControl::REVERSE) 
                        _sc->setPlayMode(SliderControl::STOP);
                    else _sc->setPlayMode(SliderControl::REVERSE);
                }
                else if(g->getName()=="Repeat")
                {
                    //osg::notify(osg::ALWAYS)<<"Repeat reached\n";
                    _sc->setLoopAnimation(!_sc->getLoopAnimation());
                }
                
                return true;
            }
        }

        return false;
    }
};

class SliderPickHandler : public osgGA::GUIEventHandler
{   
public:
    SliderPickHandler(SliderControl* sc){_sc = sc;_active = false;}
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>( &aa);
        if(!viewer) return false;
        switch(ea.getEventType())
        {
        case osgGA::GUIEventAdapter::PUSH:
            {
                if(pick(ea.getXnormalized(),ea.getYnormalized(),viewer)){
                    _pm=_sc->getPlayMode();
                    _sc->setPlayMode(SliderControl::STOP);
                    _active = true;
                    return true;
                }
            else return false;
            }
        case osgGA::GUIEventAdapter::DRAG:
            {
                if(_active)
                {
                    pick(ea.getXnormalized(),ea.getYnormalized(),viewer);
                    return true;
                }
                else return false;
            }
        case osgGA::GUIEventAdapter::RELEASE:
            {
                if(_active)
                {
                    _sc->setPlayMode(_pm);
                    _active = false;
                    if(pick(ea.getXnormalized(),ea.getYnormalized(),viewer))
                    {
                        return true;
                    }
                    else return false;
                }
                else return false;
            }
        default:
            return false;
        }
    }
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>(nv);
        if (ev && ev->getActionAdapter() && !ev->getEvents().empty())
        {
            for(osgGA::EventQueue::Events::iterator itr = ev->getEvents().begin();
                itr != ev->getEvents().end();
                ++itr)
            {
                    handleWithCheckAgainstIgnoreHandledEventsMask(*(*itr), *(ev->getActionAdapter()), node, nv);
            }
        }
        traverse(node, nv);
    }
private:
    bool _active;
    double _mx, _my;
    SliderControl::PlayMode _pm;
    SliderControl* _sc;
    bool pick(const double x, const double y, osgViewer::Viewer* v)
    {
        //osg::notify(osg::ALWAYS)<<"Clicked: x: "<<x<<" y: "<<y<<"\n";
        if(!v->getSceneData())return false;

        osgUtil::LineSegmentIntersector* picker = 
            new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION,x,y);
        osgUtil::IntersectionVisitor iv(picker);
        v->getCamera()->accept(iv);
        if(picker->containsIntersections())
        {
            for(osgUtil::LineSegmentIntersector::Intersections::const_iterator itr = picker->getIntersections().begin();
                itr !=picker->getIntersections().end();  itr++)
            {
                const osg::NodePath& nodePath = itr->nodePath;
                unsigned int idx = nodePath.size();
                //osg::notify(osg::ALWAYS)<<"Localintesection: x: "<<itr->getLocalIntersectPoint().x()<<
                 //   " y: "<<itr->getLocalIntersectPoint().y()<<std::endl;
                
                while(idx--)
                {
                    if(nodePath[idx]->getName()=="bar") 
                    {
                        _sc->setCurrentPos(itr->getLocalIntersectPoint().x());
                        return true;
                    }
                }
            }
            
           
        }

        return false;
    }
};
/* \endcond */

SliderControl::SliderControl()
  : _h(25),
    _time( 1.f ),
    _simTime(0),
    _root(new osg::Group ),
    _currentValue( 0. ),
    _playMode( STOP )
{
}

SliderControl::~SliderControl()
{
}

void
SliderControl::setCurrentValue( double v )
{
    _currentValue = v;
}
double
SliderControl::getCurrentValue() const
{
    return( _currentValue );
}

osg::Node*
SliderControl::getSliderControlSubgraph()
{
   /* osg::Camera* camera = new osg::Camera;
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1920,0,1080));
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);//set to 0 
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setAllowEventFocus(false);*/
    {
        osg::Group* bar = new osg::Group();
        osg::Geode* slider = new osg::Geode();
        _mt = new osg::MatrixTransform();
        osg::StateSet* stateset = bar->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        slider->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        osgText::Font* font = osgText::readFontFile("fonts/cour.ttf");


        {//bar
            osg::Geode* g = new osg::Geode;
            g->setName("bar");
            osg::Geometry* geom = new osg::Geometry;

            osg::Vec3Array* vertices = new osg::Vec3Array;
            vertices->push_back(osg::Vec3(0,0-_h/2,0));
            vertices->push_back(osg::Vec3(0,_h/2,0));
            vertices->push_back(osg::Vec3(_w,_h/2,0));
            vertices->push_back(osg::Vec3(_w,0-_h/2,0));
            geom->setVertexArray(vertices);

            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
            geom->setNormalArray(normals);
            geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

            osg::Vec4Array* colors = new osg::Vec4Array;
            colors->push_back(osg::Vec4(1.0f,.8,0.8f,0.2f));
            geom->setColorArray(colors);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);

            geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

            osg::StateSet* stateset = geom->getOrCreateStateSet();
            stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
            //stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
            stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            
            geom->getOrCreateStateSet()->setRenderBinDetails(2,"RenderBin");
            g->addDrawable(geom);
            bar->addChild(g);
        }
        {//minimum text
            osg::Geode* g = new osg::Geode;
            osgText::Text* minival = new osgText::Text();
            minival->setPosition(osg::Vec3(0,_h/2,0.));
            minival->setAxisAlignment(osgText::Text::XY_PLANE);
            minival->setAlignment(osgText::Text::LEFT_BOTTOM);
            char mintext [16];
            sprintf(mintext, "%.2f",_minVal);
            //osg::notify(osg::ALWAYS)<<mintext;
            minival->setText(mintext);
            minival->setFont(font);
            minival->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
            g->addDrawable(minival);
            bar->addChild(g);
        }
        {//maximum text
            osg::Geode* g = new osg::Geode;
            osgText::Text* maxval = new osgText::Text();
            maxval->setPosition(osg::Vec3(_w,_h/2,0.));
            maxval->setAxisAlignment(osgText::Text::XY_PLANE);
            maxval->setAlignment(osgText::Text::RIGHT_BOTTOM);
            char maxtext [16];
            sprintf(maxtext, "%.2f",_maxVal);
            //osg::notify(osg::ALWAYS)<<maxtext;
            maxval->setText(maxtext);
            maxval->setFont(font);
            maxval->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
            g->addDrawable(maxval);
            bar->addChild(g);
        }
        {//current value floating text
            osg::Geode* geode = new osg::Geode();
            osgText::Text* curval = new osgText::Text();
            curval->setDataVariance(osg::Object::DYNAMIC);
            curval->setPosition(osg::Vec3(0.,0.,.1));
            curval->setAxisAlignment(osgText::Text::SCREEN);
            curval->setAlignment(osgText::Text::CENTER_BOTTOM);
            char curtext [16];
            sprintf(curtext, "%.2f",_currentValue);
            //osg::notify(osg::ALWAYS)<<maxtext;
            curval->setText(curtext);
            curval->setFont(font);
            curval->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
            geode->addDrawable(curval);
            geode->setUpdateCallback(new TextCallback(this));
            geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
            //geode->getOrCreateStateSet()->setRenderBinDetails(0,"RenderBin");
            _mt->addChild(geode);
        }
        {//forward button
            osg::Geode* f = new osg::Geode();
            
            osg::Geometry* forward = new osg::Geometry();
            forward->setDataVariance(osg::Object::DYNAMIC);
            osg::Vec3Array* vertices = new osg::Vec3Array;
            vertices->push_back(osg::Vec3(2*_h,-_h/2,0.));
            vertices->push_back(osg::Vec3(3*_h,-_h/2,0.));
            vertices->push_back(osg::Vec3(3*_h,-_h/.66,0.));
            vertices->push_back(osg::Vec3(2*_h,-_h/.66,0.));
            forward->setVertexArray(vertices);

            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
            forward->setNormalArray(normals);
            forward->setNormalBinding(osg::Geometry::BIND_OVERALL);

            forward->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

            osg::Vec2Array* tc = new osg::Vec2Array;
            forward->setTexCoordArray(0,tc);
            tc->push_back(osg::Vec2(0.,0.5));
            tc->push_back(osg::Vec2(.5,0.5));
            tc->push_back(osg::Vec2(.5,1.));
            tc->push_back(osg::Vec2(0.,1.));
            osg::Vec2Array* tc2 = new osg::Vec2Array;
            forward->setTexCoordArray(1,tc2);
            tc2->push_back(osg::Vec2(0.5,0.5));
            tc2->push_back(osg::Vec2(1.,0.5));
            tc2->push_back(osg::Vec2(1.,1.));
            tc2->push_back(osg::Vec2(0.5,1.));
            osg::Image* image= osgDB::readImageFile("buttons.bmp");
            osg::Texture2D* tex = new osg::Texture2D;
            tex->setImage(image);
            forward->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
            forward->getOrCreateStateSet()->setTextureAttribute(1, tex);
            forward->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::OFF);
            f->addDrawable(forward);
            f->setName("Forward");
            bar->addChild(f);
            _forward = forward;
        }
        {//loop button
            osg::Geode* l = new osg::Geode();
            
            osg::Geometry* loop = new osg::Geometry();
            loop->setDataVariance(osg::Object::DYNAMIC);
            osg::Vec3Array* vertices = new osg::Vec3Array;
            vertices->push_back(osg::Vec3(_h,-_h/2,0.));
            vertices->push_back(osg::Vec3(2*_h,-_h/2,0.));
            vertices->push_back(osg::Vec3(2*_h,-_h/.66,0.));
            vertices->push_back(osg::Vec3(_h,-_h/.66,0.));
            loop->setVertexArray(vertices);

            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
            loop->setNormalArray(normals);
            loop->setNormalBinding(osg::Geometry::BIND_OVERALL);

            loop->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

            osg::Vec2Array* tc = new osg::Vec2Array;
            loop->setTexCoordArray(0,tc);
            tc->push_back(osg::Vec2(0.,0.));
            tc->push_back(osg::Vec2(.5,0.));
            tc->push_back(osg::Vec2(.5,0.5));
            tc->push_back(osg::Vec2(0.,0.5));
            osg::Vec2Array* tc2 = new osg::Vec2Array;
            loop->setTexCoordArray(1,tc2);
            tc2->push_back(osg::Vec2(0.5,0.));
            tc2->push_back(osg::Vec2(1.,0.));
            tc2->push_back(osg::Vec2(1.,.5));
            tc2->push_back(osg::Vec2(0.5,.5));
            osg::Image* image= osgDB::readImageFile("buttons.bmp");
            osg::Texture2D* tex = new osg::Texture2D;
            tex->setImage(image);
            loop->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
            loop->getOrCreateStateSet()->setTextureAttribute(1, tex);
            loop->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::ON);
            loop->getOrCreateStateSet()->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::OFF);
            l->addDrawable(loop);
            l->setName("Repeat");
            bar->addChild(l);
            _repeat = loop;
        }
        {//reverse button
            osg::Geode* r = new osg::Geode();
            
            osg::Geometry* reverse = new osg::Geometry();
            reverse->setDataVariance(osg::Object::DYNAMIC);
            osg::Vec3Array* vertices = new osg::Vec3Array;
            vertices->push_back(osg::Vec3(0,-_h/2,0.));
            vertices->push_back(osg::Vec3(_h,-_h/2,0.));
            vertices->push_back(osg::Vec3(_h,-_h/.66,0.));
            vertices->push_back(osg::Vec3(0,-_h/.66,0.));
            reverse->setVertexArray(vertices);

            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
            reverse->setNormalArray(normals);
            reverse->setNormalBinding(osg::Geometry::BIND_OVERALL);

            reverse->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

            osg::Vec2Array* tc = new osg::Vec2Array;
            reverse->setTexCoordArray(0,tc);
            tc->push_back(osg::Vec2(0.5,0.5));
            tc->push_back(osg::Vec2(0.0,0.5));
            tc->push_back(osg::Vec2(0.0,1.));
            tc->push_back(osg::Vec2(0.5,1.));
            osg::Vec2Array* tc2 = new osg::Vec2Array;
            reverse->setTexCoordArray(1,tc2);
            tc2->push_back(osg::Vec2(1.,0.5));
            tc2->push_back(osg::Vec2(0.5,0.5));
            tc2->push_back(osg::Vec2(0.5,1.));
            tc2->push_back(osg::Vec2(1.,1.));
            osg::Image* image= osgDB::readImageFile("buttons.bmp");
            osg::Texture2D* tex = new osg::Texture2D;
            tex->setImage(image);
            reverse->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
            reverse->getOrCreateStateSet()->setTextureAttribute(1, tex);
            reverse->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::OFF);
            r->addDrawable(reverse);
            r->setName("Reverse");
            bar->addChild(r);
            _reverse = reverse;
        }


        
        bar->setEventCallback(new ButtonPickHandler(this));
        _root->addChild(bar);
        
        _mt->setDataVariance(osg::Object::DYNAMIC);
        osg::Node * sl = osgDB::readNodeFile("slider.osg");
        if( sl == NULL )
            osg::notify( osg::FATAL ) << "Can't find slider.osg." << std::endl;
        else
        {
            sl->setName("slider");
            sl->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
            sl->setEventCallback(new SliderPickHandler(this));
            _mt->addChild(sl);
        }
        _matrix = osg::Matrix::scale(osg::Vec3(10.,10.,10.))
            //* osg::Matrix::rotate(osg::Vec3(1.,0.,0.),osg::Vec3(0.,1.,0.)) 
            //* osg::Matrix::rotate(osg::Vec3(0.,0.,1.),osg::Vec3(0.,-1.,0.))
            //* osg::Matrix::translate(osg::Vec3(6.45,0.,0.))
            ;
        _mt->setMatrix(_matrix);
        _mt->setUpdateCallback( new SliderCallback(this) );
        
        _root->addChild(_mt.get());
    }
    
    return( _root.get() );
}
void
SliderControl::setCurrentPos(double x)
{
    _currentValue = (x/_w)*(_maxVal-_minVal) + _minVal;
}
void
SliderControl::setDisplayArea( float len )
{
    _w = len;
    _h = len/11.;
}
void
SliderControl::setValueRange( double valueMin, double valueMax )
{
    _minVal = valueMin;
    _maxVal = valueMax;
}
void
SliderControl::setTimeRange( double time )
{
    _time = time;
}

void
SliderControl::setPlayMode( PlayMode pm )
{
    _playMode = pm;
}
SliderControl::PlayMode
SliderControl::getPlayMode() const
{
    return( _playMode );
}

void
SliderControl::setLoopAnimation( bool loop )
{
    _loop = loop;
}
bool
SliderControl::getLoopAnimation() const
{
    return( _loop );
}
void
SliderControl::update(double reftime)
{
    if(_simTime == 0)
    {   _simTime = reftime;
        return;}
    double change = (reftime - _simTime)*((_maxVal-_minVal)/_time);
    if(_playMode == FORWARD){
        _currentValue += change;
        if(_currentValue >= _maxVal && _loop) _currentValue = _minVal;
        else if(_currentValue>= _maxVal){_currentValue = _maxVal;_playMode = STOP;}
    }
    if(_playMode == REVERSE){
        _currentValue -= change;
        if(_currentValue <= _minVal && _loop) _currentValue = _maxVal;
        else if(_currentValue<= _minVal){_currentValue = _minVal; _playMode = STOP;}
    }

    float normalizedValue( ( _currentValue - _minVal ) / ( _maxVal - _minVal ) );
    _mt->setMatrix( _matrix * osg::Matrix::translate( osg::Vec3( normalizedValue*_w, 0., 0. )));
    _simTime = reftime;
    refreshButtons();
}
void
SliderControl::refreshButtons()
{
    if(_loop)
    {
        _repeat->getOrCreateStateSet()->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::OFF);
        _repeat->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::ON);
    }
    else
    {
        _repeat->getOrCreateStateSet()->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
        _repeat->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::OFF);
    }
    if(_playMode == FORWARD)
    {
        _forward->getOrCreateStateSet()->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::OFF);
        _forward->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::ON);
    }
    else
    {
        _forward->getOrCreateStateSet()->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
        _forward->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::OFF);
    }
    if(_playMode == REVERSE)
    {
        _reverse->getOrCreateStateSet()->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::OFF);
        _reverse->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::ON);
    }
    else
    {
        _reverse->getOrCreateStateSet()->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
        _reverse->getOrCreateStateSet()->setTextureMode(1,GL_TEXTURE_2D,osg::StateAttribute::OFF);
    }
    
}