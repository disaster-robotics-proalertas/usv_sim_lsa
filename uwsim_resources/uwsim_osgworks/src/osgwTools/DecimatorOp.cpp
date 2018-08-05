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

#include <osgwTools/DecimatorOp.h>
#include <osgwTools/Version.h>
#include <osg/TriangleIndexFunctor>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/TriStripVisitor>
#include <osgwTools/Export.h>

#include <set>
#include <list>
#include <algorithm>
#include <iterator>

namespace osgwTools {


/* \cond */

struct dereference_less
{
    template<class T, class U>
    inline bool operator() (const T& lhs,const U& rhs) const
    {
        return *lhs < *rhs;
    }
};

template<class T>
bool dereference_check_less(const T& lhs,const T& rhs)
{
    if (lhs==rhs) return false;
    if (!lhs) return true;
    if (!rhs) return false;
    return *lhs < *rhs;
};

struct dereference_clear
{
    template<class T>
    inline void operator() (const T& t)
    {
        T& non_const_t = const_cast<T&>(t);
        non_const_t->clear();
    }
};

class HalfEdgeCollapse
{
public:
    //public:
    typedef float error_type;

    struct Triangle;
    struct Edge;
    struct Point;
    //protected:
    typedef std::vector<float>                                                  FloatList;
    typedef std::set<osg::ref_ptr<Edge>,dereference_less >                      EdgeSet;
    typedef std::set< osg::ref_ptr<Point>,dereference_less >                    PointSet;
    typedef std::vector< osg::ref_ptr<Point> >                                  PointList;
    typedef std::list< osg::ref_ptr<Triangle> >                                 TriangleList;
    typedef std::set< osg::ref_ptr<Triangle> >                                  TriangleSet;
    typedef std::map< osg::ref_ptr<Triangle>, unsigned int, dereference_less >  TriangleMap;

    typedef std::vector< osg::ref_ptr<osg::Array> > ArrayList;

    osg::Geometry*                  _geometry;
    
    EdgeSet                         _edgeSet;
    TriangleSet                     _triangleSet;
    PointSet                        _pointSet;
    PointList                       _originalPointList;
    bool                            _ignoreBoundaries;



    HalfEdgeCollapse(){_ignoreBoundaries = false;};
    ~HalfEdgeCollapse();

    void setIgnoreBoundaries(bool setting){_ignoreBoundaries = setting;}
    bool getIgnoreBoundaries(){return _ignoreBoundaries;}

    void setGeometry(osg::Geometry* geometry);
    osg::Geometry* getGeometry() { return _geometry; }

    unsigned int getNumOfTriangles() { return _triangleSet.size(); }

    Point* computeInterpolatedPoint(Edge* edge,float r) const
    {
        Point* p1 = edge->_p1.get();
        Point* p2 = edge->_p2.get();
        
        if (p1==0 || p2==0)
        {
            osg::notify(osg::NOTICE)<<"Error computeInterpolatedPoint("<<edge<<",r) p1 and/or p2==0"<<std::endl;
            return 0;
        } 
        
        Point* point = new Point;
        float r1 = 1.0f-r;
        float r2 = r;

        point->_vertex = p1->_vertex * r1 + p2->_vertex * r2;
        unsigned int s = osg::minimum(p1->_attributes.size(),p2->_attributes.size());
        for(unsigned int i=0;i<s;++i)
        {
            point->_attributes.push_back(p1->_attributes[i]*r1 + p2->_attributes[i]*r2); 
        }
        return point;
    }

    float getAngle(Point* p1, Point* p2, Edge* edge) const
    {
        Point* p3 = (p2== edge->_p1)? edge->_p2.get() : edge->_p1.get();
        float a = (p1->_vertex - p2->_vertex).length();
        float b = (p2->_vertex - p3->_vertex).length();
        float c = (p1->_vertex - p3->_vertex).length();

        return 1 - ((a*a + b*b - c*c)/(2*a*b));
    }

    Point* computeOptimalPoint(Edge* edge) const
    {
        if (!edge->_p1.valid() || !edge->_p2.valid())
        {
            osg::notify(osg::ALWAYS)<<"Error in ComputeOptimalPoint()\n";
        }
        typedef std::set< osg::ref_ptr<Triangle> > LocalTriangleSet ;
        LocalTriangleSet triangles;
        std::copy( edge->_p1->_triangles.begin(), edge->_p1->_triangles.end(), std::inserter(triangles,triangles.begin()));
   
        Point* bestpoint = 0;
        Edge* sister( NULL );
        float bestangle = 0;
        for(LocalTriangleSet::iterator itr=triangles.begin();
                itr!=triangles.end();
                ++itr)
        {
            if(((*itr)->_e1->_p1 == edge->_p1)||((*itr)->_e1->_p2 == edge->_p1)) 
            {
                float angle = getAngle(edge->_p2.get(),edge->_p1.get(), (*itr)->_e1.get() );
                if (angle > bestangle)
                {
                    bestangle = angle;
                    sister = (*itr)->_e1.get();
                    bestpoint = edge->_p2.get();
                }
            }
            if(((*itr)->_e2->_p1 == edge->_p1)||((*itr)->_e2->_p2 == edge->_p1)) 
            {
                float angle = getAngle(edge->_p2.get(),edge->_p1.get(), (*itr)->_e2.get() );
                if (angle > bestangle)
                {
                    bestangle = angle;
                    sister = (*itr)->_e2.get();
                    bestpoint = edge->_p2.get();
                }
            }
            if(((*itr)->_e3->_p1 == edge->_p1)||((*itr)->_e3->_p2 == edge->_p1)) 
            {
                float angle = getAngle(edge->_p2.get(),edge->_p1.get(), (*itr)->_e3.get() );
                if (angle > bestangle)
                {
                    bestangle = angle;
                    sister = (*itr)->_e3.get();
                    bestpoint = edge->_p2.get();
                }
            }
            
        }

        triangles.clear();
        std::copy( edge->_p2->_triangles.begin(), edge->_p2->_triangles.end(), std::inserter(triangles,triangles.begin()));

        for(LocalTriangleSet::iterator itr=triangles.begin();
                itr!=triangles.end();
                ++itr)
        {
            if(((*itr)->_e1->_p1 == edge->_p2)||((*itr)->_e1->_p2 == edge->_p2)) 
            {
                float angle = getAngle(edge->_p1.get(),edge->_p2.get(), (*itr)->_e1.get() );
                if (angle > bestangle)
                {
                    bestangle = angle;
                    sister = (*itr)->_e1.get();
                    bestpoint = edge->_p1.get();
                }
            }
            if(((*itr)->_e2->_p1 == edge->_p2)||((*itr)->_e2->_p2 == edge->_p2)) 
            {
                float angle = getAngle(edge->_p1.get(),edge->_p2.get(), (*itr)->_e2.get());
                if (angle > bestangle)
                {
                    bestangle = angle;
                    sister = (*itr)->_e2.get();
                    bestpoint = edge->_p1.get();
                }
            }
            if(((*itr)->_e3->_p1 == edge->_p2)||((*itr)->_e3->_p2 == edge->_p2)) 
            {
                float angle = getAngle(edge->_p1.get(),edge->_p2.get(), (*itr)->_e3.get());
                if (angle > bestangle)
                {
                    bestangle = angle;
                    sister = (*itr)->_e3.get();
                    bestpoint = edge->_p1.get();
                }
            }
        }
        edge->setOptimalAngle(bestangle);
        if( sister==NULL )
        {
            //osg::notify( osg::FATAL ) << "NULL sister in computeOptimalPoint." << std::endl;
            edge->setSister( edge );
            bestpoint = edge->_p1.get();
        }
        else
            edge->setSister(sister);
        
        return bestpoint;

    // *********************************** CHANGE THIS! ***********************************
        //return edge->_p1;
    }
    
    error_type computeErrorMetric(Edge* edge,Point* point) const
    {
        //osg::notify(osg::ALWAYS) << "Optimal error of edge = "<<2 - error_type(edge->getOptimalAngle())<<std::endl;
        
        if (point)
        {
            //float a = (edge->_p1->_vertex - edge->_p2->_vertex).length();
            //float b = (edge->getSister()->_p1->_vertex - edge->getSister()->_p2->_vertex).length();

            //
            if((2 - error_type(edge->getOptimalAngle()))==0)return 0;
            typedef std::set< osg::ref_ptr<Triangle> > LocalTriangleSet ;
            LocalTriangleSet triangles;
            std::copy( edge->_p1->_triangles.begin(), edge->_p1->_triangles.end(), std::inserter(triangles,triangles.begin()));
            std::copy( edge->_p2->_triangles.begin(), edge->_p2->_triangles.end(), std::inserter(triangles,triangles.begin()));

            const osg::Vec3& vertex = point->_vertex;
            error_type error = 0.0;

            if (triangles.empty()) return 0.0;

            for(LocalTriangleSet::iterator itr=triangles.begin();
                itr!=triangles.end();
                ++itr)
            {
                error += fabs( (*itr)->distance(vertex) );
            }
            
            // use average of error
            error /= error_type(triangles.size());
            if (error == 0){
                return (2 - error_type(edge->getOptimalAngle()));// *( osg::maximum( a/b , b/a )*osg::maximum(a,b));
            }

            return error;
        }
        else
        {
            return 0;
        }
    }

    void updateErrorMetricForEdge(Edge* edge)
    {
        
        if (!edge->_p1 || !edge->_p2)
        {
            osg::notify(osg::NOTICE)<<"Error updateErrorMetricForEdge("<<edge<<") p1 and/or p2==0"<<std::endl;
            return;
        }

        osg::ref_ptr<Edge> keep_local_reference_to_edge(edge);
    
        if (_edgeSet.count(keep_local_reference_to_edge)!=0)
        {
            _edgeSet.erase(keep_local_reference_to_edge);
        }

        edge->_proposedPoint = computeOptimalPoint(edge);
        
        edge->updateMaxNormalDeviationOnHalfEdgeCollapse();
        
    
        if (edge->getMaxNormalDeviationOnHalfEdgeCollapse()<=1.0 && (!edge->isAdjacentToBoundary() || getIgnoreBoundaries() )
                                                                 && !edge->concavityFlip()     )
                      
            edge->setErrorMetric( computeErrorMetric( edge, edge->_proposedPoint.get()));
           
        else
            edge->setErrorMetric( FLT_MAX );    
        
        _edgeSet.insert(keep_local_reference_to_edge);

    }
    
    void updateErrorMetricForAllEdges()
    {
        typedef std::vector< osg::ref_ptr<Edge> > LocalEdgeList ;
        LocalEdgeList edges;
        std::copy( _edgeSet.begin(), _edgeSet.end(), std::back_inserter(edges));
        
        _edgeSet.clear();
        
        for(LocalEdgeList::iterator itr=edges.begin();
            itr!=edges.end();
            ++itr)
        {
            Edge* edge = itr->get();
            updateErrorMetricForEdge(edge);
        }
    }

    struct Point : public osg::Referenced
    {
        Point(): _protected(false), _index(0) {}
        
        bool _protected;

        unsigned int _index;

        osg::Vec3           _vertex;
        FloatList           _attributes;
        TriangleSet         _triangles;

        void clear()
        {
            _attributes.clear();
            _triangles.clear();
        }

        bool operator < ( const Point& rhs) const
        {
            if (_vertex < rhs._vertex) return true;
            if (rhs._vertex < _vertex) return false;
            
            return _attributes < rhs._attributes;
        }
        
        bool isBoundaryPoint() const
        {
            if (_protected) return true;
        
            for(TriangleSet::const_iterator itr=_triangles.begin();
                itr!=_triangles.end();
                ++itr)
            {
                const Triangle* triangle = itr->get();
                if ((triangle->_e1->_p1==this || triangle->_e1->_p2==this) && triangle->_e1->isBoundaryEdge()) return true;
                if ((triangle->_e2->_p1==this || triangle->_e2->_p2==this) && triangle->_e2->isBoundaryEdge()) return true;
                if ((triangle->_e3->_p1==this || triangle->_e3->_p2==this) && triangle->_e3->isBoundaryEdge()) return true;
            
                //if ((*itr)->isBoundaryTriangle()) return true;
            }
            return false;
        }

    };

    struct Edge : public osg::Referenced
    {
        Edge(): _errorMetric(0.0), _maximumDeviation(1.0) {}
        
        void clear()
        {
            _p1 = 0;
            _p2 = 0;
            _triangles.clear();
        }

        osg::ref_ptr<Point> _p1;
        osg::ref_ptr<Point> _p2;
        
        TriangleSet _triangles;

        error_type _errorMetric;
        error_type _maximumDeviation;
        float _optimalAngle;
        Edge* _sister;

        osg::ref_ptr<Point> _proposedPoint;

        void setErrorMetric(error_type errorMetric) { _errorMetric = errorMetric; }
        error_type getErrorMetric() const {
            //osg::notify(osg::ALWAYS) <<"Error Read to be: " << _errorMetric << std::endl;
            return _errorMetric; }

        void setOptimalAngle(float angle){_optimalAngle=angle;}
        float getOptimalAngle() const {return _optimalAngle;}
        
        void setSister(Edge* sister){_sister = sister;}
        osg::ref_ptr<Edge> getSister(){return _sister;}
        
        bool operator < ( const Edge& rhs) const
        {
            // both error metrics are computed
            if (getErrorMetric()<rhs.getErrorMetric()) return true;
            else if (rhs.getErrorMetric()<getErrorMetric()) return false;

            if (dereference_check_less(_p1,rhs._p1)) return true;
            if (dereference_check_less(rhs._p1,_p1)) return false;
            
            return dereference_check_less(_p2,rhs._p2);
        }
        
        bool operator == ( const Edge& rhs) const
        {
            if (&rhs==this) return true;
            if (*this<rhs) return false;
            if (rhs<*this) return false;
            return true;
        }

        bool operator != ( const Edge& rhs) const
        {
            if (&rhs==this) return false;
            if (*this<rhs) return true;
            if (rhs<*this) return true;
            return false;
        }
        
        void addTriangle(Triangle* triangle)
        {
            _triangles.insert(triangle);
            // if (_triangles.size()>2) osg::notify(osg::NOTICE)<<"Warning too many triangles ("<<_triangles.size()<<") sharing edge "<<std::endl;
        }
        
        bool isBoundaryEdge() const
        {
            return _triangles.size()<=1;
        }
        
        bool isAdjacentToBoundary() const
        {
            return isBoundaryEdge() || _p1->isBoundaryPoint() || _p2->isBoundaryPoint(); 
        }
        
        bool concavityFlip()
        {   
            Point* p1 = _proposedPoint.get();
            Point* p2 = (_p1 == _proposedPoint) ? _p2.get() : _p1.get();
            Point* p3 = (_sister->_p1 == p2) ? _sister->_p2.get() : _sister->_p1.get();
            std::set<osg::ref_ptr<Point> > points;
            points = comparePoints();
            std::set<osg::ref_ptr<Point> >::iterator itr = points.begin();
            if(itr == points.end())return false;
            Point* p4 = (*itr).get();
            if (p4 ==p1)return true;
            ++itr;
            if(itr == points.end())return false;

            Point* p5 = (*itr).get();
            if (p5 ==p1)return true;
            if(points.size()>2) {osg::notify(osg::ALWAYS)<<"Edge has > 2 triangles \n";return true;}
            if(intersect(p3,p4,p5,  p1,p2)==1 || (intersect(p3,p4,p5,  p1,p2)==2 && intersect(p1,p4,p5,  p3,p2)!=2)){return true; }

            return false;
        }
        #define SMALL_NUM  0.0000000000000001 // anything that avoids division overflow
        #define dot(u,v) ((u).x()*(v).x()+(u).y()*(v).y()+(u).z()*(v).z())
        int intersect(Point* p1,Point* p2,Point* p3,  //triangle
                            Point* p4,Point* p5)//segment
        {
            
            osg::Vec3 u, v, n,z,
                      dir, w0, w,
                      i;
            float r, a, b;
            z = osg::Vec3(0,0,0);
            u = p2->_vertex - p1->_vertex;
            v = p3->_vertex - p1->_vertex;
            n = u ^ v; //triangle normal
            if(n==z){return 0;}//degenerate triangle
            
            dir = p5->_vertex - p4->_vertex;//segment direction
            w0 = p4->_vertex - p1->_vertex;
            a = -dot(n,w0);
            b = dot(n,dir);
            if(fabs(b) <= SMALL_NUM){  //segment parallel to triangle plane
                if (a == 0)           //segment in plane
                    return 2;
                else return 0;//setment disjoint from plane
            }
            r = a/b;
            if(r <= 0.0 || r >= 1.0) {return 0;}//no intersect, segment ends before triangle plane
            
            return 1;

            i = p4->_vertex + dir*r;//intersect point on triangle plane
            //point enclosed by triangle?
            float uu,uv,vv,wu,wv,D;
            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);
            w = i - p1->_vertex;
            wu = dot(w,u);
            wv = dot(w,v);
            D = uv * uv - uu * vv;

            float s,t;
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        // I is outside T
                return 0;
            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  // I is outside T
                return 0;
            return 1;                      // I is in T
        }

        std::set<osg::ref_ptr<Point> > comparePoints()
        {

            std::set<osg::ref_ptr<Point> > points;
            if(_sister->_triangles.empty()) return points;
            for(TriangleSet::iterator itr = _sister->_triangles.begin();
                itr != _sister->_triangles.end(); ++itr)
            {
                if(((*itr)->_p1 == _sister->_p1 && (*itr)->_p2 == _sister->_p2)
                    ||((*itr)->_p1 == _sister->_p2 && (*itr)->_p2 == _sister->_p1))
                         points.insert((*itr)->_p3);
                if(((*itr)->_p2 == _sister->_p1 && (*itr)->_p3 == _sister->_p2)
                    ||((*itr)->_p2 == _sister->_p2 && (*itr)->_p3 == _sister->_p1))
                         points.insert((*itr)->_p1);
                if(((*itr)->_p1 == _sister->_p1 && (*itr)->_p3 == _sister->_p2)
                    ||((*itr)->_p1 == _sister->_p2 && (*itr)->_p3 == _sister->_p1))
                         points.insert((*itr)->_p2);
            }

            return points;
        }

        void updateMaxNormalDeviationOnHalfEdgeCollapse()
        {
            //osg::notify(osg::NOTICE)<<"updateMaxNormalDeviationOnHalfEdgeCollapse()"<<std::endl;
            _maximumDeviation = 0.0f;
            for(TriangleSet::iterator itr1=_p1->_triangles.begin();
                itr1!=_p1->_triangles.end();
                ++itr1)
            {
                if (_triangles.count(*itr1)==0)
                {
                    _maximumDeviation = osg::maximum(_maximumDeviation, (*itr1)->computeNormalDeviationOnHalfEdgeCollapse(this,_proposedPoint.get()));
                }
            }
            for(TriangleSet::iterator itr2=_p2->_triangles.begin();
                itr2!=_p2->_triangles.end();
                ++itr2)
            {
                if (_triangles.count(*itr2)==0)
                {
                    _maximumDeviation = osg::maximum(_maximumDeviation, (*itr2)->computeNormalDeviationOnHalfEdgeCollapse(this,_proposedPoint.get()));
                }
            }
        }
        
        error_type getMaxNormalDeviationOnHalfEdgeCollapse() const { return _maximumDeviation; } 

    };

    struct Triangle : public osg::Referenced
    {
        Triangle() {}
        
        void clear()
        {
            _p1 = 0;
            _p2 = 0;
            _p3 = 0;
        
            _e1 = 0;
            _e2 = 0;
            _e3 = 0;
        }

        inline bool operator < (const Triangle& rhs) const
        {
            if (dereference_check_less(_p1,rhs._p1)) return true;
            if (dereference_check_less(rhs._p1,_p1)) return false;


            const Point* lhs_lower = dereference_check_less(_p2,_p3) ? _p2.get() : _p3.get(); 
            const Point* rhs_lower = dereference_check_less(rhs._p2,rhs._p3) ? rhs._p2.get() : rhs._p3.get(); 

            if (dereference_check_less(lhs_lower,rhs_lower)) return true;
            if (dereference_check_less(rhs_lower,lhs_lower)) return false;

            const Point* lhs_upper = dereference_check_less(_p2,_p3) ? _p3.get() : _p2.get(); 
            const Point* rhs_upper = dereference_check_less(rhs._p2,rhs._p3) ? rhs._p3.get() : rhs._p2.get(); 

            return dereference_check_less(lhs_upper,rhs_upper);
        }


        void setOrderedPoints(Point* p1, Point* p2, Point* p3)
        {
            Point* points[3];
            points[0] = p1;
            points[1] = p2;
            points[2] = p3;

            // find the lowest value point in the list.
            unsigned int lowest = 0;
            if (dereference_check_less(points[1],points[lowest])) lowest = 1;
            if (dereference_check_less(points[2],points[lowest])) lowest = 2;

            _p1 = points[lowest];
            _p2 = points[(lowest+1)%3];
            _p3 = points[(lowest+2)%3];
        }
        
        void update()
        {
            _plane.set(_p1->_vertex,_p2->_vertex,_p3->_vertex);
            
        }
        
        osg::Plane computeNewPlaneOnHalfEdgeCollapse(Edge* edge,Point* pNew) const
        {
            const Point* p1 = (_p1==edge->_p1 || _p1==edge->_p2) ? pNew : _p1.get();  
            const Point* p2 = (_p2==edge->_p1 || _p2==edge->_p2) ? pNew : _p2.get();  
            const Point* p3 = (_p3==edge->_p1 || _p3==edge->_p2) ? pNew : _p3.get();
            
            return osg::Plane(p1->_vertex,p2->_vertex,p3->_vertex);
        }
        
        // note return 1 - dotproduct, so that deviation is in the range of 0.0 to 2.0, where 0 is coincident, 1.0 is 90 degrees, and 2.0 is 180 degrees.
        error_type computeNormalDeviationOnHalfEdgeCollapse(Edge* edge,Point* pNew) const
        {
            try{
                const Point* p1 = (_p1==edge->_p1 || _p1==edge->_p2) ? pNew : _p1.get();  
                const Point* p2 = (_p2==edge->_p1 || _p2==edge->_p2) ? pNew : _p2.get();  
                const Point* p3 = (_p3==edge->_p1 || _p3==edge->_p2) ? pNew : _p3.get();
                
                osg::Vec3 new_normal = (p2->_vertex - p1->_vertex) ^ (p3->_vertex - p2->_vertex);
                new_normal.normalize();

                error_type result = 1.0 - (new_normal.x() * _plane[0] + new_normal.y() * _plane[1] + new_normal.z() * _plane[2]);

                return result;
            }catch( char* /*str*/ )
            {
                return 3.0;
            }
        }

        error_type distance(const osg::Vec3& vertex) const
        {
            return error_type(_plane[0])*error_type(vertex.x())+
                   error_type(_plane[1])*error_type(vertex.y())+
                   error_type(_plane[2])*error_type(vertex.z())+
                   error_type(_plane[3]);
        }
        
        bool isBoundaryTriangle() const
        {
            return (_e1->isBoundaryEdge() || _e2->isBoundaryEdge() ||  _e3->isBoundaryEdge());
        }

       
        osg::ref_ptr<Point> _p1;
        osg::ref_ptr<Point> _p2;
        osg::ref_ptr<Point> _p3;
        
        osg::ref_ptr<Edge> _e1;
        osg::ref_ptr<Edge> _e2;
        osg::ref_ptr<Edge> _e3;
        
        osg::Plane _plane;

    };

    Triangle* addTriangle(unsigned int p1, unsigned int p2, unsigned int p3)
    {
        //osg::notify(osg::NOTICE)<<"addTriangle("<<p1<<","<<p2<<","<<p3<<")"<<std::endl;

        // detect if triangle is degenerate.
        if (p1==p2 || p2==p3 || p1==p3) return 0;
        
        Triangle* triangle = new Triangle;

        Point* points[3];
        points[0] = addPoint(triangle, p1);
        points[1] = addPoint(triangle, p2);
        points[2] = addPoint(triangle, p3);
        
        // find the lowest value point in the list.
        unsigned int lowest = 0;
        if (dereference_check_less(points[1],points[lowest])) lowest = 1;
        if (dereference_check_less(points[2],points[lowest])) lowest = 2;

        triangle->_p1 = points[lowest];
        triangle->_p2 = points[(lowest+1)%3];
        triangle->_p3 = points[(lowest+2)%3];

        triangle->_e1 = addEdge(triangle, triangle->_p1.get(), triangle->_p2.get());
        triangle->_e2 = addEdge(triangle, triangle->_p2.get(), triangle->_p3.get());
        triangle->_e3 = addEdge(triangle, triangle->_p3.get(), triangle->_p1.get());
        
        triangle->update();

        _triangleSet.insert(triangle);
        
        return triangle;
    }

    Triangle* addTriangle(Point* p1, Point* p2, Point* p3)
    {
        // osg::notify(osg::NOTICE)<<"      addTriangle("<<p1<<","<<p2<<","<<p3<<")"<<std::endl;

        // detect if triangle is degenerate.
        if (p1==p2 || p2==p3 || p1==p3 || !p1|| !p2 || !p3) 
        {
            // osg::notify(osg::NOTICE)<<"    **** addTriangle failed - p1==p2 || p2==p3 || p1==p3"<<std::endl;
            return 0;
        }
        
        Triangle* triangle = new Triangle;

        Point* points[3];
        points[0] = addPoint(triangle, p1);
       // osg::notify(osg::ALWAYS)<<"Test Point 1 Passed\n";
        points[1] = addPoint(triangle, p2);
        //osg::notify(osg::ALWAYS)<<"Test Point 3 Passed\n";
        points[2] = addPoint(triangle, p3);
        
        // find the lowest value point in the list.
        unsigned int lowest = 0;        
        if (dereference_check_less(points[1],points[lowest])) lowest = 1;
        if (dereference_check_less(points[2],points[lowest])) lowest = 2;

        triangle->_p1 = points[lowest];
        triangle->_p2 = points[(lowest+1)%3];
        triangle->_p3 = points[(lowest+2)%3];

        triangle->_e1 = addEdge(triangle, triangle->_p1.get(), triangle->_p2.get());
        triangle->_e2 = addEdge(triangle, triangle->_p2.get(), triangle->_p3.get());
        triangle->_e3 = addEdge(triangle, triangle->_p3.get(), triangle->_p1.get());
        
        triangle->update();

        _triangleSet.insert(triangle);

        return triangle;
    }

    void removeTriangle(Triangle* triangle)
    {
        if (triangle->_p1.valid()) removePoint(triangle,triangle->_p1.get());
        if (triangle->_p2.valid()) removePoint(triangle,triangle->_p2.get());
        if (triangle->_p3.valid()) removePoint(triangle,triangle->_p3.get());
        
        if (triangle->_e1.valid()) removeEdge(triangle,triangle->_e1.get());
        if (triangle->_e2.valid()) removeEdge(triangle,triangle->_e2.get());
        if (triangle->_e3.valid()) removeEdge(triangle,triangle->_e3.get());

        _triangleSet.erase(triangle);
    }

    void replaceTrianglePoint(Triangle* triangle, Point* pOriginal, Point* pNew)
    {
        if (triangle->_p1==pOriginal || triangle->_p2==pOriginal || triangle->_p3==pOriginal)
        {
            // fix the corner points to use the new point
            if (triangle->_p1==pOriginal) triangle->_p1=pNew;
            if (triangle->_p2==pOriginal) triangle->_p2=pNew;
            if (triangle->_p3==pOriginal) triangle->_p3=pNew;
            
            // fixes the edges so they point to use the new point
            triangle->_e1 = replaceEdgePoint(triangle->_e1.get(),pOriginal,pNew);
            triangle->_e2 = replaceEdgePoint(triangle->_e2.get(),pOriginal,pNew);
            triangle->_e3 = replaceEdgePoint(triangle->_e3.get(),pOriginal,pNew);
            
            // remove the triangle form the original point, and possibly the point if its the last triangle to use it
            removePoint(triangle, pOriginal);
            
            // add the triangle to that point
            addPoint(triangle,pNew);
        }
        
    }
    Edge* addEdge(Triangle* triangle, Point* p1, Point* p2)
    {
        // osg::notify(osg::NOTICE)<<"        addEdge("<<p1<<","<<p2<<")"<<std::endl;
        osg::ref_ptr<Edge> edge = new Edge;
        if (dereference_check_less(p1, p2))
        {
            edge->_p1 = p1;
            edge->_p2 = p2;
        }
        else
        {
            edge->_p1 = p2;
            edge->_p2 = p1;
        }
        
        edge->setErrorMetric( computeErrorMetric( edge.get(), edge->_proposedPoint.get()));
        
        EdgeSet::iterator itr = _edgeSet.find(edge);
        if (itr==_edgeSet.end())
        {
            // osg::notify(osg::NOTICE)<<"          addEdge("<<edge.get()<<") edge->_p1="<<edge->_p1.get()<<" _p2="<<edge->_p2.get()<<std::endl;
            _edgeSet.insert(edge);
        }
        else
        {

            // osg::notify(osg::NOTICE)<<"          reuseEdge("<<edge.get()<<") edge->_p1="<<edge->_p1.get()<<" _p2="<<edge->_p2.get()<<std::endl;
            edge = *itr;
        }
        
        edge->addTriangle(triangle);
        
        return edge.get();
    }

    void removeEdge(Triangle* triangle, Edge* edge)
    {
        EdgeSet::iterator itr = _edgeSet.find(edge);
        if (itr!=_edgeSet.end())
        {
            edge->_triangles.erase(triangle);
            if (edge->_triangles.empty())
            {
                edge->_p1 = 0;
                edge->_p2 = 0;

                // edge no longer in use, so need to delete.
                _edgeSet.erase(itr);
            }
        }
    }

    Edge* replaceEdgePoint(Edge* edge, Point* pOriginal, Point* pNew)
    {
        if (edge->_p1==pOriginal || edge->_p2==pOriginal)
        {
            EdgeSet::iterator itr = _edgeSet.find(edge);
            if (itr!=_edgeSet.end())
            {
                // remove the edge from the list, as its position in the list
                // may need to change once its values have been ammended 
                _edgeSet.erase(itr);
            }
            
            // modify its values
            if (edge->_p1==pOriginal) edge->_p1=pNew;
            if (edge->_p2==pOriginal) edge->_p2=pNew;

            if (dereference_check_less(edge->_p2,edge->_p1))
            {
                edge->_p1.swap(edge->_p2);
            }

            itr = _edgeSet.find(edge);
            if (itr!=_edgeSet.end())
            {
                // reuse existing edge.
                edge = const_cast<Edge*>(itr->get());
            }
            else
            {
                // put it back in.
                _edgeSet.insert(edge);
            }
            return edge;
        }
        else
        {
            return edge;
        }
        
    }

    Point* addPoint(Triangle* triangle, unsigned int p1)
    {
        return addPoint(triangle,_originalPointList[p1].get());
    }

    Point* addPoint(Triangle* triangle, Point* point)
    {   

        if(!point)
        {
            osg::notify(osg::ALWAYS)<<"Point Invalid\n";
        }
        PointSet::iterator itr = _pointSet.find(point);
        
        if (itr==_pointSet.end())
        {
            //osg::notify(osg::NOTICE)<<"  addPoint("<<point.get()<<")"<<std::endl;
            _pointSet.insert(point);
        }
        else
        {
            point = const_cast<Point*>(itr->get());
            //osg::notify(osg::NOTICE)<<"  reusePoint("<<point.get()<<")"<<std::endl;
        }
        //osg::notify(osg::ALWAYS)<<point->_vertex.length()<<"      "<<point->_triangles.size()<<"    "<<point->referenceCount()<<std::endl;
        point->_triangles.insert(triangle);
        //osg::notify(osg::ALWAYS)<<"Test Point 2 Passed\n";
        return point;
    }

    void removePoint(Triangle* triangle, Point* point)
    {
        PointSet::iterator itr = _pointSet.find(point);
        if (itr!=_pointSet.end())
        {
            point->_triangles.erase(triangle);
            
            if (point->_triangles.empty())
            {
                // point no longer in use, so need to delete.
                _pointSet.erase(itr);
            }
        }
        
    }

    unsigned int testPoint(Point* point)
    {
        unsigned int numErrors = 0;
        
        for(TriangleSet::iterator itr=point->_triangles.begin();
            itr!=point->_triangles.end();
            ++itr)
        {
            Triangle* triangle = const_cast<Triangle*>(itr->get());
            if (!(triangle->_p1 == point || triangle->_p2 == point || triangle->_p3 == point))
            {
                osg::notify(osg::NOTICE)<<"testPoint("<<point<<") error, triangle "<<triangle<<" does not point back to this point"<<std::endl;
                osg::notify(osg::NOTICE)<<"             triangle->_p1 "<<triangle->_p1.get()<<std::endl;
                osg::notify(osg::NOTICE)<<"             triangle->_p2 "<<triangle->_p2.get()<<std::endl;
                osg::notify(osg::NOTICE)<<"             triangle->_p3 "<<triangle->_p3.get()<<std::endl;
                ++numErrors;
            }
        }
        
        return numErrors;
    }
    
    unsigned int testAllPoints()
    {
        unsigned int numErrors = 0;
        for(PointSet::iterator itr = _pointSet.begin();
            itr!=_pointSet.end();
            ++itr)
        {
            numErrors += testPoint(const_cast<Point*>(itr->get()));
        }
        return numErrors;
    }

     unsigned int testEdge(Edge* edge)
    {
        unsigned int numErrors = 0;
        for(TriangleSet::iterator teitr=edge->_triangles.begin();
            teitr!=edge->_triangles.end();
            ++teitr)
        {
            Triangle* triangle = const_cast<Triangle*>(teitr->get());
            if (!(triangle->_e1 == edge || triangle->_e2 == edge || triangle->_e3 == edge))
            {
                osg::notify(osg::NOTICE)<<"testEdge("<<edge<<"). triangle != point back to this edge"<<std::endl;
                osg::notify(osg::NOTICE)<<"                     triangle->_e1=="<<triangle->_e1.get()<<std::endl;
                osg::notify(osg::NOTICE)<<"                     triangle->_e2=="<<triangle->_e2.get()<<std::endl;
                osg::notify(osg::NOTICE)<<"                     triangle->_e3=="<<triangle->_e3.get()<<std::endl;
                ++numErrors;
            }
        }
        
        if (edge->_triangles.empty())
        {
            osg::notify(osg::NOTICE)<<"testEdge("<<edge<<")._triangles is empty"<<std::endl;
            ++numErrors;
        }
        return numErrors;
    }

    unsigned int testAllEdges()
    {
        unsigned int numErrors = 0;
        for(EdgeSet::iterator itr = _edgeSet.begin();
            itr!=_edgeSet.end();
            ++itr)
        {
            numErrors += testEdge(const_cast<Edge*>(itr->get()));
        }
        return numErrors;
    }
    unsigned int testTriangle(Triangle* triangle)
    {
        unsigned int result = 0;
        if (!(triangle->_p1))
        {
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _p1==NULL"<<std::endl;
            ++result;
        }
        else if (triangle->_p1->_triangles.count(triangle)==0) 
        {
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _p1->_triangles does not contain triangle"<<std::endl;
            ++result;
        }

        if (!(triangle->_p2))
        {
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _p2==NULL"<<std::endl;
            ++result;
        }
        else if (triangle->_p2->_triangles.count(triangle)==0) 
        {
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _p2->_triangles does not contain triangle"<<std::endl;
            ++result;
        }

        if (!(triangle->_p3))
        {
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _p3==NULL"<<std::endl;
            ++result;
        }
        else if (triangle->_p3->_triangles.count(triangle)==0) 
        {
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _p3->_triangles does not contain triangle"<<std::endl;
            ++result;
        }
        
        if (testEdge(triangle->_e1.get()))
        {
            ++result;
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _e1 test failed"<<std::endl;
        }
        
        if (testEdge(triangle->_e2.get()))
        {
            ++result;
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _e2 test failed"<<std::endl;
        }

        if (testEdge(triangle->_e3.get()))
        {
            osg::notify(osg::NOTICE)<<"testTriangle("<<triangle<<") _e3 test failed"<<std::endl;
            ++result;
        }

        return result;
    }
    unsigned int testAllTriangles()
    {
        unsigned int numErrors = 0;
        for(TriangleSet::iterator itr=_triangleSet.begin();
            itr!=_triangleSet.end();
            ++itr)
        {
            numErrors += testTriangle(const_cast<Triangle*>(itr->get()));
        }
        return numErrors;
    }

    unsigned int computeNumBoundaryEdges()
    {
        unsigned int numBoundaryEdges = 0;
        for(EdgeSet::iterator itr = _edgeSet.begin();
            itr!=_edgeSet.end();
            ++itr)
        {
            if ((*itr)->isBoundaryEdge()) ++numBoundaryEdges;
        }
        return numBoundaryEdges;
    }

    bool collapseEdge(Edge* edge, Point* pNew)
    {        
        typedef std::set< osg::ref_ptr<Edge> > LocalEdgeList;
        osg::ref_ptr<Edge> keep_edge_locally_referenced_to_prevent_premature_deletion = edge;
        osg::ref_ptr<Point> keep_point_locally_referenced_to_prevent_premature_deletion = pNew;
        osg::ref_ptr<Point> edge_p1 = edge->_p1;
        osg::ref_ptr<Point> edge_p2 = edge->_p2;
        
        TriangleMap  triangleMap;
        TriangleList triangles_p1;
        TriangleList triangles_p2;
        LocalEdgeList oldEdges;
        
        
        if (edge_p1 != pNew)
        {
            for(TriangleSet::iterator itr=edge_p1->_triangles.begin();
                itr!=edge_p1->_triangles.end();
                ++itr)
            {
                if (edge->_triangles.count(*itr)==0)
                {
                    Triangle* triangle = const_cast<Triangle*>(itr->get());
                    triangles_p1.push_back(triangle);
                    oldEdges.insert(triangle->_e1);
                    oldEdges.insert(triangle->_e2);
                    oldEdges.insert(triangle->_e3);
                }
            }
            
            //triangles_p1 = edge_p1->_triangles;
        }
                
        if (edge_p2 != pNew)
        {
            //for all triangles attached to this point
            for(TriangleSet::iterator itr=edge_p2->_triangles.begin();
                itr!=edge_p2->_triangles.end();
                ++itr)
            {
                //if the triangle is not one of the triangles with this edge
                if (edge->_triangles.count(*itr)==0)
                {
                    //save all the old edges of the trangle and the triangle itself
                    Triangle* triangle = const_cast<Triangle*>(itr->get());
                    triangles_p2.push_back(triangle);
                    oldEdges.insert(triangle->_e1);
                    oldEdges.insert(triangle->_e2);
                    oldEdges.insert(triangle->_e3);
                }
            }
            //triangles_p2 = edge_p2->_triangles;
        }
        //for all the saved old edges
        for(LocalEdgeList::iterator oeitr=oldEdges.begin();
            oeitr!=oldEdges.end();
            ++oeitr)
        {
            //reset the error metric of all edges to 0.0
            _edgeSet.erase(*oeitr);
            
            const_cast<Edge*>(oeitr->get())->setErrorMetric(0.0f);
            
            _edgeSet.insert(*oeitr);
        }

        TriangleList::iterator titr_p1, titr_p2;
        //remove all the triangles around each point.
        for(titr_p1 = triangles_p1.begin();
            titr_p1 != triangles_p1.end();
            ++titr_p1)
        {
            removeTriangle(const_cast<Triangle*>(titr_p1->get()));
        }

        for(titr_p2 = triangles_p2.begin();
            titr_p2 != triangles_p2.end();
            ++titr_p2)
        {
            removeTriangle(const_cast<Triangle*>(titr_p2->get()));
        }

        //osg::notify(osg::NOTICE)<<"  pNew="<<pNew<<"\tedge_p1"<<edge_p1.get()<<"\tedge_p2"<<edge_p2.get()<<std::endl;
        
        // we copy the edge's _triangles and interate the copy of the triangle set to avoid invalidating iterators.
        TriangleSet trianglesToRemove = edge->_triangles;
        //remove triangles associated with that edge.
        for(TriangleSet::iterator teitr=trianglesToRemove.begin();
            teitr!=trianglesToRemove.end();
            ++teitr)
        {
            Triangle* triangle = const_cast<Triangle*>(teitr->get());
            removeTriangle(triangle);
        }

        LocalEdgeList newEdges;

        //for all the triangles around the end point of the edge
        for(titr_p1 = triangles_p1.begin();
            titr_p1 != triangles_p1.end();
            ++titr_p1)
        {
            //creat new triangles using the new point replacing the old end points
            Triangle* triangle = const_cast<Triangle*>(titr_p1->get());
 
            Point* p1 = (triangle->_p1==edge_p1 || triangle->_p1==edge_p2)? pNew : triangle->_p1.get();
            Point* p2 = (triangle->_p2==edge_p1 || triangle->_p2==edge_p2)? pNew : triangle->_p2.get();
            Point* p3 = (triangle->_p3==edge_p1 || triangle->_p3==edge_p2)? pNew : triangle->_p3.get();

            Triangle* newTri = addTriangle(p1,p2,p3);

            if (newTri)
            {
                newEdges.insert(newTri->_e1);
                newEdges.insert(newTri->_e2);
                newEdges.insert(newTri->_e3);
            }
        }

        for(titr_p2 = triangles_p2.begin();
            titr_p2 != triangles_p2.end();
            ++titr_p2)
        {
            Triangle* triangle = const_cast<Triangle*>(titr_p2->get());

            Point* p1 = (triangle->_p1==edge_p1 || triangle->_p1==edge_p2)? pNew : triangle->_p1.get();
            Point* p2 = (triangle->_p2==edge_p1 || triangle->_p2==edge_p2)? pNew : triangle->_p2.get();
            Point* p3 = (triangle->_p3==edge_p1 || triangle->_p3==edge_p2)? pNew : triangle->_p3.get();
            //osg::notify(osg::ALWAYS)<<"pNew is "<<pNew->_vertex.x()<<" "<<pNew->_vertex.y()<<" "<<pNew->_vertex.z()<<std::endl;
            Triangle* newTri = addTriangle(p1,p2,p3);
           
            if (newTri)
            {
                newEdges.insert(newTri->_e1);
                newEdges.insert(newTri->_e2);
                newEdges.insert(newTri->_e3);
            }
        }

        //list of edges to update the error on
        LocalEdgeList edges2UpdateErrorMetric;

        LocalEdgeList::const_iterator newEdgeIt(newEdges.begin());
        //go through all the edges we just created
        while (newEdgeIt != newEdges.end())
        {
            //set p to the point on the edge that is not our new point
            const Point* p = 0;
            if (newEdgeIt->get()->_p1.get() != pNew)
                p = newEdgeIt->get()->_p1.get();
            else
                p = newEdgeIt->get()->_p2.get();
            //go through all the triangles connected to point p
            TriangleSet::const_iterator triangleIt(p->_triangles.begin());
            while (triangleIt != p->_triangles.end())
            {
                //insert edges of the triangle to the needsupdating list if they contain p
                const Triangle* triangle = triangleIt->get();
                if (triangle->_e1->_p1 == p || triangle->_e1->_p2 == p)
                    edges2UpdateErrorMetric.insert(triangle->_e1);
                if (triangle->_e2->_p1 == p || triangle->_e2->_p2 == p)
                    edges2UpdateErrorMetric.insert(triangle->_e2);
                if (triangle->_e3->_p1 == p || triangle->_e3->_p2 == p)
                    edges2UpdateErrorMetric.insert(triangle->_e3);

                ++triangleIt;
            }

            ++newEdgeIt;
        }

        //insert all the newly created edges
        edges2UpdateErrorMetric.insert(newEdges.begin(), newEdges.end());

        // osg::notify(osg::NOTICE)<<"Edges to recalibarate "<<edges2UpdateErrorMetric.size()<<std::endl;
        //finally update the error metric on all the edges
        
        for(LocalEdgeList::iterator itr=edges2UpdateErrorMetric.begin();
            itr!=edges2UpdateErrorMetric.end();
            ++itr)
        {
            //osg::notify(osg::NOTICE)<<"updateErrorMetricForEdge("<<itr->get()<<")"<<std::endl;
            updateErrorMetricForEdge(const_cast<Edge*>(itr->get()));
        }
        //osg::notify(osg::ALWAYS)<<"Finishing Edge Collapse\n";
        return true;
    }

    bool collapseMinimumErrorEdge()
    {
        if (!_edgeSet.empty())
        {
            Edge* edge = const_cast<Edge*>(_edgeSet.begin()->get());

            if (edge->getErrorMetric()==FLT_MAX)
            {
                osg::notify(osg::INFO)<<"collapseMinimumErrorEdge() return false due to edge->getErrorMetric()==FLT_MAX"<<std::endl;
                return false;
            }

            osg::ref_ptr<Point> pNew = edge->_proposedPoint.valid()? edge->_proposedPoint.get() : computeInterpolatedPoint(edge,0.5f);
            //osg::notify(osg::INFO)<< "Collapsing min error edge: " << edge->getOptimalAngle() << std::endl;
            return (collapseEdge(edge,pNew.get()));
        }
        osg::notify(osg::INFO)<<"collapseMinimumErrorEdge() return false due to _edgeSet.empty()"<<std::endl;
        return false;
    }

    void copyBackToGeometry();
};

struct CollectTriangleOperator
{

    CollectTriangleOperator():_ec(0) {}

    void setHalfEdgeCollapse(HalfEdgeCollapse* ec) { _ec = ec; }
    
    HalfEdgeCollapse* _ec;    

    // for use  in the triangle functor.
    inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3)
    {
        _ec->addTriangle(p1,p2,p3);
    }

};

/** \relates DecimatorOp */
typedef osg::TriangleIndexFunctor<CollectTriangleOperator> CollectTriangleIndexFunctor;

class CopyVertexArrayToPointsVisitor : public osg::ArrayVisitor
{
    public:
        CopyVertexArrayToPointsVisitor(HalfEdgeCollapse::PointList& pointList):
            _pointList(pointList) {}
        
        virtual void apply(osg::Vec2Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i] = new HalfEdgeCollapse::Point;
                _pointList[i]->_index = i;
                
                osg::Vec2& value = array[i];
                osg::Vec3& vertex = _pointList[i]->_vertex;
                vertex.set(value.x(),value.y(),0.0f);  
            }
        }

        virtual void apply(osg::Vec3Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i] = new HalfEdgeCollapse::Point;
                _pointList[i]->_index = i;
                
                _pointList[i]->_vertex = array[i];
            }
        }
        
        virtual void apply(osg::Vec4Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i] = new HalfEdgeCollapse::Point;
                _pointList[i]->_index = i;
                
                osg::Vec4& value = array[i];
                osg::Vec3& vertex = _pointList[i]->_vertex;
                vertex.set(value.x()/value.w(),value.y()/value.w(),value.z()/value.w());  
            }
        }
        
        HalfEdgeCollapse::PointList& _pointList;

    protected:
    
        CopyVertexArrayToPointsVisitor& operator = (const CopyVertexArrayToPointsVisitor&) { return *this; }

};

class CopyArrayToPointsVisitor : public osg::ArrayVisitor
{
    public:
        CopyArrayToPointsVisitor(HalfEdgeCollapse::PointList& pointList):
            _pointList(pointList) {}
        
        template<class T>
        void copy(T& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
                _pointList[i]->_attributes.push_back((float)array[i]);  
        }
        
        virtual void apply(osg::Array&) {}
        virtual void apply(osg::ByteArray& array) { copy(array); }
        virtual void apply(osg::ShortArray& array) { copy(array); }
        virtual void apply(osg::IntArray& array) { copy(array); }
        virtual void apply(osg::UByteArray& array) { copy(array); }
        virtual void apply(osg::UShortArray& array) { copy(array); }
        virtual void apply(osg::UIntArray& array) { copy(array); }
        virtual void apply(osg::FloatArray& array) { copy(array); }

        virtual void apply(osg::Vec4ubArray& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                osg::Vec4ub& value = array[i];
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                attributes.push_back((float)value.r());  
                attributes.push_back((float)value.g());  
                attributes.push_back((float)value.b());  
                attributes.push_back((float)value.a());  
            }
        }

        virtual void apply(osg::Vec2Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                osg::Vec2& value = array[i];
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                attributes.push_back(value.x());  
                attributes.push_back(value.y());  
            }
        }

        virtual void apply(osg::Vec3Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                osg::Vec3& value = array[i];
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                attributes.push_back(value.x());  
                attributes.push_back(value.y());  
                attributes.push_back(value.z());  
            }
        }
        
        virtual void apply(osg::Vec4Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                osg::Vec4& value = array[i];
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                attributes.push_back(value.x());  
                attributes.push_back(value.y());  
                attributes.push_back(value.z());  
                attributes.push_back(value.w());  
            }
        }
        
        HalfEdgeCollapse::PointList& _pointList;
        
        
    protected:
    
        CopyArrayToPointsVisitor& operator = (const CopyArrayToPointsVisitor&) { return *this; }
};

void HalfEdgeCollapse::setGeometry(osg::Geometry* geometry)
{
    _geometry = geometry;
    
#if( OSGWORKS_OSG_VERSION < 30108 )    
    // check to see if vertex attributes indices exists, if so expand them to remove them
    // This was removed from OSG 3.1.8 upwards
    if (_geometry->suitableForOptimization())
    {
        // removing coord indices
        osg::notify(osg::INFO)<<"HalfEdgeCollapse::setGeometry(..): Removing attribute indices"<<std::endl;
        _geometry->copyToAndOptimize(*_geometry);
    }
#endif    
    
    // check to see arrays are shared
    if (_geometry->containsSharedArrays())
    {
        // duplicating arrays
        osg::notify(osg::INFO)<<"HalfEdgeCollapse::setGeometry(..): Duplicate shared arrays"<<std::endl;
        _geometry->duplicateSharedArrays();
    }

    unsigned int numVertices = geometry->getVertexArray()->getNumElements();
        
    _originalPointList.resize(numVertices);
    
    // copy vertices across to local point list
    CopyVertexArrayToPointsVisitor copyVertexArrayToPoints(_originalPointList);
    _geometry->getVertexArray()->accept(copyVertexArrayToPoints);
    
    // copy other per vertex attributes across to local point list.
    CopyArrayToPointsVisitor        copyArrayToPoints(_originalPointList);

    for(unsigned int ti=0;ti<_geometry->getNumTexCoordArrays();++ti)
    {
        if (_geometry->getTexCoordArray(ti))
            geometry->getTexCoordArray(ti)->accept(copyArrayToPoints);
    }

    if (_geometry->getNormalArray() && _geometry->getNormalBinding()==osg::Geometry::BIND_PER_VERTEX)
        geometry->getNormalArray()->accept(copyArrayToPoints);
        
    if (_geometry->getColorArray() && _geometry->getColorBinding()==osg::Geometry::BIND_PER_VERTEX)
        geometry->getColorArray()->accept(copyArrayToPoints);
        
    if (_geometry->getSecondaryColorArray() && _geometry->getSecondaryColorBinding()==osg::Geometry::BIND_PER_VERTEX)
        geometry->getSecondaryColorArray()->accept(copyArrayToPoints);

    if (_geometry->getFogCoordArray() && _geometry->getFogCoordBinding()==osg::Geometry::BIND_PER_VERTEX)
        geometry->getFogCoordArray()->accept(copyArrayToPoints);

    for(unsigned int vi=0;vi<_geometry->getNumVertexAttribArrays();++vi)
    {
        if (_geometry->getVertexAttribArray(vi) &&  _geometry->getVertexAttribBinding(vi)==osg::Geometry::BIND_PER_VERTEX)
            geometry->getVertexAttribArray(vi)->accept(copyArrayToPoints);
    }

    // now set the protected points up.
    /*for(Simplifier::IndexList::const_iterator pitr=protectedPoints.begin();
        pitr!=protectedPoints.end();
        ++pitr)
    {
        _originalPointList[*pitr]->_protected = true;
    }*/


    CollectTriangleIndexFunctor collectTriangles;
    collectTriangles.setHalfEdgeCollapse(this);
    
    _geometry->accept(collectTriangles);
    
}



HalfEdgeCollapse::~HalfEdgeCollapse()
{
    std::for_each(_edgeSet.begin(),_edgeSet.end(),dereference_clear());

    std::for_each(_triangleSet.begin(),_triangleSet.end(),dereference_clear());
    std::for_each(_pointSet.begin(),_pointSet.end(),dereference_clear());
    std::for_each(_originalPointList.begin(),_originalPointList.end(),dereference_clear());
}

class CopyPointsToArrayVisitor : public osg::ArrayVisitor
{
    public:
        CopyPointsToArrayVisitor(HalfEdgeCollapse::PointList& pointList):
            _pointList(pointList),
            _index(0) {}


        template<typename T,typename R>
        void copy(T& array, R /*dummy*/)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                if (_index<_pointList[i]->_attributes.size()) 
                {
                    float val = (_pointList[i]->_attributes[_index]);
                    array[i] = R (val);
                }
            }
                
            ++_index;
        }
        
        // use local typedefs if usinged char,short and int to get round gcc 3.3.1 problem with defining unsigned short()
        typedef unsigned char dummy_uchar;
        typedef unsigned short dummy_ushort;
        typedef unsigned int dummy_uint;
        
        virtual void apply(osg::Array&) {}
        virtual void apply(osg::ByteArray& array) { copy(array, char());}
        virtual void apply(osg::ShortArray& array) { copy(array, short()); }
        virtual void apply(osg::IntArray& array) { copy(array, int()); }
        virtual void apply(osg::UByteArray& array) { copy(array, dummy_uchar()); }
        virtual void apply(osg::UShortArray& array) { copy(array,dummy_ushort()); }
        virtual void apply(osg::UIntArray& array) { copy(array, dummy_uint()); }
        virtual void apply(osg::FloatArray& array) { copy(array, float()); }

        virtual void apply(osg::Vec4ubArray& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                array[i].set((unsigned char)attributes[_index],
                             (unsigned char)attributes[_index+1],
                             (unsigned char)attributes[_index+2],
                             (unsigned char)attributes[_index+3]);
            }
            _index += 4;
        }

        virtual void apply(osg::Vec2Array& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                if (_index+1<attributes.size()) array[i].set(attributes[_index],attributes[_index+1]);
            }
            _index += 2;
        }

        virtual void apply(osg::Vec3Array& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                if (_index+2<attributes.size()) array[i].set(attributes[_index],attributes[_index+1],attributes[_index+2]);
            }
            _index += 3;
        }
        
        virtual void apply(osg::Vec4Array& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                HalfEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                if (_index+3<attributes.size()) array[i].set(attributes[_index],attributes[_index+1],attributes[_index+2],attributes[_index+3]);
            }
            _index += 4;
        }
        
        HalfEdgeCollapse::PointList& _pointList;
        unsigned int _index;
        
    protected:
    
        CopyPointsToArrayVisitor& operator = (CopyPointsToArrayVisitor&) { return *this; }
};

class NormalizeArrayVisitor : public osg::ArrayVisitor
{
    public:
        NormalizeArrayVisitor() {}
        
        template<typename Itr>
        void normalize(Itr begin, Itr end)
        {
            for(Itr itr = begin;
                itr != end;
                ++itr)
            {
                itr->normalize();
            }
        }
        
        virtual void apply(osg::Vec2Array& array) { normalize(array.begin(),array.end()); }
        virtual void apply(osg::Vec3Array& array) { normalize(array.begin(),array.end()); }
        virtual void apply(osg::Vec4Array& array) { normalize(array.begin(),array.end()); }
        
};

class CopyPointsToVertexArrayVisitor : public osg::ArrayVisitor
{
    public:
        CopyPointsToVertexArrayVisitor(HalfEdgeCollapse::PointList& pointList):
            _pointList(pointList) {}
        
        virtual void apply(osg::Vec2Array& array)
        {
            array.resize(_pointList.size());
            
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i]->_index = i;
                osg::Vec3& vertex = _pointList[i]->_vertex;
                array[i].set(vertex.x(),vertex.y());
            }
        }

        virtual void apply(osg::Vec3Array& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i]->_index = i;
                array[i] = _pointList[i]->_vertex;
            }
        }
        
        virtual void apply(osg::Vec4Array& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i]->_index = i;
                osg::Vec3& vertex = _pointList[i]->_vertex;
                array[i].set(vertex.x(),vertex.y(),vertex.z(),1.0f);
            }
        }
        
        HalfEdgeCollapse::PointList& _pointList;
        
    protected:
    
        CopyPointsToVertexArrayVisitor& operator = (const CopyPointsToVertexArrayVisitor&) { return *this; }
};



void HalfEdgeCollapse::copyBackToGeometry()
{

    // rebuild the _pointList from the _pointSet
    _originalPointList.clear();
    std::copy(_pointSet.begin(),_pointSet.end(),std::back_inserter(_originalPointList));

    // copy vertices across to local point list
    CopyPointsToVertexArrayVisitor copyVertexArrayToPoints(_originalPointList);
    _geometry->getVertexArray()->accept(copyVertexArrayToPoints);
    
    // copy other per vertex attributes across to local point list.
    CopyPointsToArrayVisitor  copyArrayToPoints(_originalPointList);

    for(unsigned int ti=0;ti<_geometry->getNumTexCoordArrays();++ti)
    {
        if (_geometry->getTexCoordArray(ti))
            _geometry->getTexCoordArray(ti)->accept(copyArrayToPoints);
    }

    if (_geometry->getNormalArray() && _geometry->getNormalBinding()==osg::Geometry::BIND_PER_VERTEX)
    {
        _geometry->getNormalArray()->accept(copyArrayToPoints);

        // now normalize the normals.
        NormalizeArrayVisitor nav;
        _geometry->getNormalArray()->accept(nav);
    }
        
    if (_geometry->getColorArray() && _geometry->getColorBinding()==osg::Geometry::BIND_PER_VERTEX)
        _geometry->getColorArray()->accept(copyArrayToPoints);
        
    if (_geometry->getSecondaryColorArray() && _geometry->getSecondaryColorBinding()==osg::Geometry::BIND_PER_VERTEX)
        _geometry->getSecondaryColorArray()->accept(copyArrayToPoints);

    if (_geometry->getFogCoordArray() && _geometry->getFogCoordBinding()==osg::Geometry::BIND_PER_VERTEX)
        _geometry->getFogCoordArray()->accept(copyArrayToPoints);

    for(unsigned int vi=0;vi<_geometry->getNumVertexAttribArrays();++vi)
    {
        if (_geometry->getVertexAttribArray(vi) &&  _geometry->getVertexAttribBinding(vi)==osg::Geometry::BIND_PER_VERTEX)
            _geometry->getVertexAttribArray(vi)->accept(copyArrayToPoints);
    }

    typedef std::set< osg::ref_ptr<Triangle>, dereference_less >    TrianglesSorted;
    TrianglesSorted trianglesSorted;
    for(TriangleSet::iterator itr = _triangleSet.begin();
        itr != _triangleSet.end();
        ++itr)
    {
        trianglesSorted.insert(*itr);
    }

    osg::DrawElementsUInt* primitives = new osg::DrawElementsUInt(GL_TRIANGLES,trianglesSorted.size()*3);
    unsigned int pos = 0;
    for(TrianglesSorted::iterator titr=trianglesSorted.begin();
        titr!=trianglesSorted.end();
        ++titr)
    {
        const Triangle* triangle = (*titr).get();
        (*primitives)[pos++] = triangle->_p1->_index;
        (*primitives)[pos++] = triangle->_p2->_index;
        (*primitives)[pos++] = triangle->_p3->_index;
    }
    
    _geometry->getPrimitiveSetList().clear();
    _geometry->addPrimitiveSet(primitives);

}

/* \endcond */

DecimatorOp::DecimatorOp(double sampleRatio, double maximumError):
            _sampleRatio(sampleRatio),
            _maximumError(maximumError),
            _triStrip(true),
            _smoothing(false),
            _ignoreBoundaries(false),
            _minPrim(4)
            {};

DecimatorOp::DecimatorOp( const DecimatorOp& rhs, const osg::CopyOp& copyOp )
{
    _sampleRatio = rhs._sampleRatio;
    _maximumError = rhs._maximumError;
    _triStrip = rhs._triStrip;
    _smoothing = rhs._smoothing;
    _ignoreBoundaries = rhs._ignoreBoundaries;
}

void DecimatorOp::decimate(osg::Geometry& geometry)
{
    osg::notify( osg::INFO)<<"++++++++++++++decimator************"<<std::endl;

    HalfEdgeCollapse hec;
    hec.setIgnoreBoundaries( _ignoreBoundaries );
    hec.setGeometry(&geometry);
    hec.updateErrorMetricForAllEdges();
    unsigned int numOriginalPrimitives = hec._triangleSet.size();
    osg::notify( osg::INFO ) << "  Primitives Available to Decimate: "<<numOriginalPrimitives<<std::endl;
    
    if ((numOriginalPrimitives < _minPrim)){
       osg::notify( osg::INFO ) << "  Geomety too small to decimate: skipping...\n";
        return;
    }
    if (getSampleRatio()<1.0)
    {
        while (!hec._edgeSet.empty() &&
               continueDecimation((*hec._edgeSet.begin())->getErrorMetric() , numOriginalPrimitives, hec._triangleSet.size()) && 
               hec.collapseMinimumErrorEdge() && hec._triangleSet.size() > _minPrim)
        {
            //osg::notify(osg::NOTICE)<<"   Collapsed edge ec._triangleSet.size()="<<hec._triangleSet.size()<<" error="<<(*hec._edgeSet.begin())->getErrorMetric()<<" vs "<<getMaximumError()<<std::endl;
        }

        osg::notify(osg::INFO)<<"******* AFTER HALF EDGE COLLAPSE *********"<<hec._triangleSet.size()<<std::endl;
    }

    osg::notify(osg::INFO)<<"Number of triangle errors after half edge collapse= "<<hec.testAllTriangles()<<std::endl;
    osg::notify(osg::INFO)<<"Number of edge errors before half edge collapse= "<<hec.testAllEdges()<<std::endl;
    osg::notify(osg::INFO)<<"Number of point errors after edge collapse= "<<hec.testAllPoints()<<std::endl;
    osg::notify(osg::INFO)<<"Number of triangles= "<<hec._triangleSet.size()<<std::endl;
    osg::notify(osg::INFO)<<"Number of points= "<<hec._pointSet.size()<<std::endl;
    osg::notify(osg::INFO)<<"Number of edges= "<<hec._edgeSet.size()<<std::endl;
    osg::notify(osg::INFO)<<"Number of boundary edges= "<<hec.computeNumBoundaryEdges()<<std::endl;

    if (!hec._edgeSet.empty())
    {
        osg::notify(osg::INFO)<<std::endl<<"Decimator, Polygons in = "<<numOriginalPrimitives<<"\t;out = "<<hec._triangleSet.size()<<"\terror="<<(*hec._edgeSet.begin())->getErrorMetric()<<"\tvs "<<getMaximumError()<<std::endl<<std::endl;
        osg::notify(osg::INFO)<<           "        !hec._edgeSet.empty()  = "<<!hec._edgeSet.empty()<<std::endl;
        osg::notify(osg::INFO)<<           "        continueDecimation(,,)  = "<<continueDecimation((*hec._edgeSet.begin())->getErrorMetric() , numOriginalPrimitives, hec._triangleSet.size())<<std::endl;
    }
    
    hec.copyBackToGeometry();

    if (_smoothing)
    {
        osgUtil::SmoothingVisitor::smooth(geometry);
    }
    
    if (_triStrip)
    {
        osgUtil::TriStripVisitor stripper;
        stripper.stripify(geometry);
    }

}

}
