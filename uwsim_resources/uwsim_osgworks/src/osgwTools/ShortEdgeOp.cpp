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

#include <osgwTools/ShortEdgeOp.h>
#include <osgwTools/Version.h>
#include <osg/TriangleIndexFunctor>
#include <osg/BoundingBox>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/TriStripVisitor>
#include <osgwTools/Export.h>

#include <set>
#include <list>
#include <algorithm>
#include <iterator>

namespace osgwTools {

/* \cond */

struct SEdereference_less
{
    template<class T, class U>
    inline bool operator() (const T& lhs,const U& rhs) const
    {
        return *lhs < *rhs;
    }
};

template<class T>
bool SEdereference_check_less(const T& lhs,const T& rhs)
{
    if (lhs==rhs) return false;
    if (!lhs) return true;
    if (!rhs) return false;
    return *lhs < *rhs;
};

struct SEdereference_clear
{
    template<class T>
    inline void operator() (const T& t)
    {
        T& non_const_t = const_cast<T&>(t);
        non_const_t->clear();
    }
};

class ShortEdgeCollapse
{
public:
    //public:
    typedef float error_type;

    struct Triangle;
    struct Edge;
    struct Point;
    struct PointLink;
    struct LinkedPointData;

    //protected:
    typedef std::vector<float>                                                  FloatList;
    typedef std::set<osg::ref_ptr<Edge>,SEdereference_less >                    EdgeSet;
    typedef std::vector<osg::ref_ptr<Edge> >                                    EdgeList;
    typedef std::set< osg::ref_ptr<Point>,SEdereference_less >                  PointSet;
    typedef std::vector< osg::ref_ptr<Point> >                                  PointList;
    typedef std::list< osg::ref_ptr<Triangle> >                                 TriangleList;
    typedef std::set< osg::ref_ptr<Triangle> >                                  TriangleSet;
    typedef std::map< osg::ref_ptr<Triangle>, unsigned int, SEdereference_less >  TriangleMap;
    typedef std::vector< osg::ref_ptr<PointLink> >                              PointLinkList;
    typedef std::map< osg::ref_ptr<Point>, osg::ref_ptr<Triangle> >             PointTriangleMap;
    typedef std::list< osg::ref_ptr<LinkedPointData> >                          LinkedPointDataList;
    typedef std::vector< osg::ref_ptr<osg::Array> >                             ArrayList;

    osg::Geometry*                  _geometry;
    
    EdgeSet                         _edgeSet;
    TriangleSet                     _triangleSet;
    PointSet                        _pointSet;
    PointList                       _originalPointList;
    bool                            _ignoreBoundaries;

    unsigned int _collapsePasses, _linksFound;

    class BoundingBoxGeom : osg::BoundingBoxf
    {
    public:

        // resize the bounds to fit the geometry
        void setBounds(PointList const& pointList)
        {
            // walk the point list passing coordinate array to bounding box
            for ( PointList::const_iterator ptIt = pointList.begin(); ptIt != pointList.end(); ++ptIt)
            {
                expandBy((*ptIt)->_vertex);
            }
        }

        // used to determine the shortest distance from the edge of the bounding box container of the geometry to a point
        float edgeDistance(const Point* const pt) const
        {
            float x1 = _min.x();
            float x2 = _max.x();
            float y1 = _min.y();
            float y2 = _max.y();
            float z1 = _min.z();
            float z2 = _max.z();
            float px = pt->_vertex.x();
            float py = pt->_vertex.y();
            float pz = pt->_vertex.z();

            x1 = std::min( px - x1, x2 - px );
            y1 = std::min( py - y1, y2 - py );
            z1 = std::min( pz - z1, z2 - pz );
            return ( x1 + y1 + z1 );
        }

        float minDimension(void) const
        {

            float mindimension = std::min( xMax() - xMin(), yMax() - yMin());
            mindimension = std::min( mindimension, zMax() - zMin());

            return ( mindimension );
        }

        float minDimensionNonZero(void) const
        {
            #define SMALL_NUM_BBOX .0001     // just something a little bigger than 0
            float mindimension = std::min( xMax() - xMin(), yMax() - yMin());
            if (mindimension < SMALL_NUM_BBOX) 
                mindimension = std::max( xMax() - xMin(), yMax() - yMin());
            mindimension = std::min( mindimension, zMax() - zMin());
            if (mindimension < SMALL_NUM_BBOX) 
                mindimension = std::max( mindimension, zMax() - zMin());

            return ( mindimension );
        }

        void getDimensions(float& xDim, float& yDim, float& zDim) const
        {
            xDim = xMax() - xMin();
            yDim = yMax() - yMin();
            zDim = zMax() - zMin();
        }

    };

    BoundingBoxGeom _bBox;

    ShortEdgeCollapse(){_ignoreBoundaries = false; _collapsePasses=0; _linksFound = 0;};
    ~ShortEdgeCollapse();

    void setIgnoreBoundaries(bool setting){_ignoreBoundaries = setting;}
    bool getIgnoreBoundaries()const {return _ignoreBoundaries;}

    void setGeometry(osg::Geometry* geometry);
    void getDimensions(float& xDim, float& yDim, float& zDim) const {_bBox.getDimensions(xDim, yDim, zDim);}

    unsigned int getNumOfTriangles() const { return _triangleSet.size(); }

    Point* computeOptimalPoint(const Edge* const edge) const
    {
        if (!edge->_p1.valid() || !edge->_p2.valid() || edge->_p1 == edge->_p2)
        {
            osg::notify(osg::ALWAYS)<<"Error in ComputeOptimalPoint()\n";
            return 0;
        }
   
        Point* bestpoint = 0, *p1, *p2;
        float p1Distance, p2Distance;

        p1 = edge->_p1.get();
        p2 = edge->_p2.get();
        p1Distance = _bBox.edgeDistance(p1);
        p2Distance = _bBox.edgeDistance(p2);
        if (p2Distance < p1Distance)
            bestpoint = p2;
        else if (p2Distance > p1Distance)
            bestpoint = p1;
        else
        {
            if (p1->x() < p2->x())
                bestpoint = p1;
            else if (p1->x() > p2->x())
                bestpoint = p2;
            else if (p1->y() < p2->y())
                bestpoint = p1;
            else if (p1->y() > p2->y())
                bestpoint = p2;
            else if (p1->z() < p2->z())
                bestpoint = p1;
            else //if (p1->() > p2->z())
                bestpoint = p2;
        }
        return bestpoint;

    }
    
    error_type computeErrorMetric(const Edge* const edge) const
    {
        
       error_type error = (edge->_p1->_vertex - edge->_p2->_vertex).length();

       return error;
    }

    void updateErrorMetricForEdge(Edge * const edge)
    {
        
        if (!edge->_p1 || !edge->_p2 || ! edge->_triangles.size())
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
        
        if (!edge->isAdjacentToBoundary() || getIgnoreBoundaries() )
            edge->setErrorMetric( computeErrorMetric( edge));
        else
            edge->setErrorMetric( FLT_MAX );    
        
        _edgeSet.insert(keep_local_reference_to_edge);

    }
    
    void setErrorMetricForEdgeNoInsert(Edge* const edge)
    {
        
        edge->_proposedPoint = computeOptimalPoint(edge);
        if (edge->_proposedPoint)
            edge->setErrorMetric( computeErrorMetric( edge));
        else
            edge->setErrorMetric( FLT_MAX );
        
    }
    
    void updateErrorMetricForAllEdges(void)
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

    struct LinkedPointData : osg::Referenced
    {
        LinkedPointData(): _pRemove(0), _pSave(0)   {}
        LinkedPointData(Point* a): _pRemove(a), _pSave(0) {}
        LinkedPointData(Point* a, Point* b): _pRemove(a), _pSave(b) {}

        void setSavePoint(Point* b)  {_pSave = b;}
        void addRemoveTriangle(Triangle* const triangle) {_triToRemove.push_back(triangle);}
        void addModifyTriangle(Triangle* const triangle) {_triToModify.push_back(triangle);}
        Point* getRemovePoint(void) const  {return (_pRemove.get());}
        Point* getSavePoint(void) const  {return (_pSave.get());}

        osg::ref_ptr<Point> _pRemove;
        osg::ref_ptr<Point> _pSave;
        TriangleList _triToRemove;
        TriangleList _triToModify;
    };

    struct PointLink : osg::Referenced
    {
        PointLink(): _pA(0), _pB(0) {}
        PointLink(Point* a, Point* b): _pA(a), _pB(b) {}

        osg::ref_ptr<Point> _pA;
        osg::ref_ptr<Point> _pB;

        void completeLink( Point* const point)
        {
            if (_pA == 0)
                _pA = point;
            else if (_pB == 0)
                _pB = point;
        }

        Point *other(Point const* const pThis) const
        {
            return (pThis == _pA ? _pB.get(): _pA.get());
        }

        bool valid(void) const
        {
            return (_pA != 0 && _pB != 0);
        }
    };

    struct Point : public osg::Referenced
    {
        Point(): _protected(false), _index(0) {}
        
        bool _protected;

        unsigned int _index;

        osg::Vec3           _vertex;
        FloatList           _attributes;
        TriangleSet         _triangles;
        PointLinkList       _pointLinks;

        void clear(void)
        {
            _attributes.clear();
            _triangles.clear();
            _pointLinks.clear();
        }

        float x(void) const  { return _vertex[0];}
        float y(void) const  { return _vertex[1];}
        float z(void) const  { return _vertex[2];}

        void addLink(PointLink* pLink)
        {
            bool found (false);

            // search for a PointLink that already has this point
            if (pLink->_pA.get() == this)
            {
                for (PointLinkList::const_iterator itr = _pointLinks.begin(); itr != _pointLinks.end(); ++itr)
                {
                    PointLink const* qLink = (*itr).get();
                    if (qLink->_pA == pLink->_pB || qLink->_pB == pLink->_pB)
                        found = true;
                } // for
            }
            else if ((pLink->_pB.get() == this))
            {
                for (PointLinkList::const_iterator itr = _pointLinks.begin(); itr != _pointLinks.end(); ++itr)
                {
                    PointLink const* qLink = (*itr).get();
                    if (qLink->_pA == pLink->_pA || qLink->_pB == pLink->_pA)
                        found = true;
                } // for
            }
            if (! found)
                _pointLinks.push_back(pLink);
        }

        void clearLinks(void)
        {
            for (PointLinkList::iterator itr = _pointLinks.begin(); itr != _pointLinks.end(); ++itr)
            {
                PointLink *qLink = (*itr).get();
                if (qLink->valid())
                {
                    qLink->other(this)->removeLink(*itr);
                }
            } // for
            _pointLinks.clear();
        }

        void removeLink(osg::ref_ptr<PointLink>& pLink)
        {
            if (pLink->_pA == this)
                pLink->_pA = 0;
            if (pLink->_pB == this)
                pLink->_pB = 0;

            //_pointLinks.erase(pLink);
            for (PointLinkList::iterator pitr = _pointLinks.begin(); 
                pitr != _pointLinks.end(); 
                ++pitr)
            {
                if ((*pitr) == pLink)
                {
                    _pointLinks.erase(pitr);
                    break;
                }
            }
        }

        bool equals( const Point* const rhs ) const
        {
            return (_vertex[0] == rhs->_vertex[0] && _vertex[1] == rhs->_vertex[1] && _vertex[2] == rhs->_vertex[2]);
        }

        bool operator < ( Point const& rhs) const
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

        void removeTriangle(Triangle* const triangle)
        {
            TriangleSet::iterator itr=_triangles.find(triangle);
            if (itr != _triangles.end())
                _triangles.erase(itr);
        }

        void addTriangle(Triangle* const triangle)
        {
            _triangles.insert(triangle);
        }
    };

    EdgeSet::iterator findEdgeByAddress(const Edge* const edge)
    {
        for(EdgeSet::iterator esitr = _edgeSet.begin();
            esitr!=_edgeSet.end();
            ++esitr)
        {
            if ((*esitr) == edge)
            {
                return esitr;
            }
        }
        return _edgeSet.end();
    }

    struct Edge : public osg::Referenced
    {
        Edge(): _errorMetric(0.0) {}
        
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

        osg::ref_ptr<Point> _proposedPoint;

        void setErrorMetric(error_type const errorMetric) { _errorMetric = errorMetric; }
        error_type getErrorMetric() const {
            //osg::notify(osg::ALWAYS) <<"Error Read to be: " << _errorMetric << std::endl;
            return _errorMetric; }

        bool operator < ( const Edge& rhs) const
        {
            // both error metrics are computed
            if (getErrorMetric()<rhs.getErrorMetric()) return true;
            else if (rhs.getErrorMetric()<getErrorMetric()) return false;

            if (SEdereference_check_less(_p1,rhs._p1)) return true;
            if (SEdereference_check_less(rhs._p1,_p1)) return false;
            
            return SEdereference_check_less(_p2,rhs._p2);
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
        
        void addTriangle(Triangle* const triangle)
        {
            _triangles.insert(triangle);
            // if (_triangles.size()>2) osg::notify(osg::NOTICE)<<"Warning more than two triangles ("<<_triangles.size()<<") sharing edge "<<std::endl;
        }
        
        bool isBoundaryEdge() const
        {
            return (_triangles.size() <= 1);
        }
        
        bool isAdjacentToBoundary() const
        {
            return isBoundaryEdge() || _p1->isBoundaryPoint() || _p2->isBoundaryPoint(); 
        }

        bool coincidentEdge(const Edge& rhs) const
        {
            return ((_p1->equals(rhs._p1.get()) || _p1->equals(rhs._p2.get())) && (_p2->equals(rhs._p1.get()) || _p2->equals(rhs._p2.get())));
        }

        void setOrderedPoints(Point* const p1, Point* const p2)
        {
            Point* points[2];
            points[0] = p1;
            points[1] = p2;

            // find the lowest value point in the list.
            unsigned int lowest = 0;
            if (SEdereference_check_less(points[1],points[lowest])) lowest = 1;

            _p1 = points[lowest];
            _p2 = points[1-lowest];
        }
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
            if (SEdereference_check_less(_p1,rhs._p1)) return true;
            if (SEdereference_check_less(rhs._p1,_p1)) return false;


            const Point* lhs_lower = SEdereference_check_less(_p2,_p3) ? _p2.get() : _p3.get(); 
            const Point* rhs_lower = SEdereference_check_less(rhs._p2,rhs._p3) ? rhs._p2.get() : rhs._p3.get(); 

            if (SEdereference_check_less(lhs_lower,rhs_lower)) return true;
            if (SEdereference_check_less(rhs_lower,lhs_lower)) return false;

            const Point* lhs_upper = SEdereference_check_less(_p2,_p3) ? _p3.get() : _p2.get(); 
            const Point* rhs_upper = SEdereference_check_less(rhs._p2,rhs._p3) ? rhs._p3.get() : rhs._p2.get(); 

            return SEdereference_check_less(lhs_upper,rhs_upper);
        }


        void setOrderedPoints(Point* const p1, Point* const p2, Point* const p3)
        {
            Point* points[3];
            points[0] = p1;
            points[1] = p2;
            points[2] = p3;

            // find the lowest value point in the list.
            unsigned int lowest = 0;
            if (SEdereference_check_less(points[1],points[lowest])) lowest = 1;
            if (SEdereference_check_less(points[2],points[lowest])) lowest = 2;

            _p1 = points[lowest];
            _p2 = points[(lowest+1)%3];
            _p3 = points[(lowest+2)%3];
        }

        bool isBoundaryTriangle(void) const
        {
            return (_e1->isBoundaryEdge() || _e2->isBoundaryEdge() ||  _e3->isBoundaryEdge());
        }

        osg::ref_ptr<Point> _p1;
        osg::ref_ptr<Point> _p2;
        osg::ref_ptr<Point> _p3;
        
        osg::ref_ptr<Edge> _e1;
        osg::ref_ptr<Edge> _e2;
        osg::ref_ptr<Edge> _e3;
        
    };

    Triangle* addTriangle(const unsigned int p1, const unsigned int p2, const unsigned int p3)
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
        if (SEdereference_check_less(points[1],points[lowest])) lowest = 1;
        if (SEdereference_check_less(points[2],points[lowest])) lowest = 2;

        triangle->_p1 = points[lowest];
        triangle->_p2 = points[(lowest+1)%3];
        triangle->_p3 = points[(lowest+2)%3];

        triangle->_e1 = addEdge(triangle, triangle->_p1.get(), triangle->_p2.get());
        triangle->_e2 = addEdge(triangle, triangle->_p2.get(), triangle->_p3.get());
        triangle->_e3 = addEdge(triangle, triangle->_p3.get(), triangle->_p1.get());
        
        _triangleSet.insert(triangle);
        
        return triangle;
    }

    Triangle* addTriangle(Point* const p1, Point* const p2, Point* const p3)
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
        if (SEdereference_check_less(points[1],points[lowest])) lowest = 1;
        if (SEdereference_check_less(points[2],points[lowest])) lowest = 2;

        triangle->_p1 = points[lowest];
        triangle->_p2 = points[(lowest+1)%3];
        triangle->_p3 = points[(lowest+2)%3];

        triangle->_e1 = addEdge(triangle, triangle->_p1.get(), triangle->_p2.get());
        triangle->_e2 = addEdge(triangle, triangle->_p2.get(), triangle->_p3.get());
        triangle->_e3 = addEdge(triangle, triangle->_p3.get(), triangle->_p1.get());
        
        _triangleSet.insert(triangle);

        return triangle;
    }

    void removeTriangle(Triangle* const triangle)
    {
        if (triangle->_p1.valid()) removePoint(triangle,triangle->_p1.get());
        if (triangle->_p2.valid()) removePoint(triangle,triangle->_p2.get());
        if (triangle->_p3.valid()) removePoint(triangle,triangle->_p3.get());
        
        if (triangle->_e1.valid()) removeEdge(triangle,triangle->_e1.get());
        if (triangle->_e2.valid()) removeEdge(triangle,triangle->_e2.get());
        if (triangle->_e3.valid()) removeEdge(triangle,triangle->_e3.get());

        _triangleSet.erase(triangle);
    }

    void replaceTrianglePoint(Triangle* const triangle, Point* const pOriginal, Point* const pNew)
    {
        if (triangle->_p1==pOriginal || triangle->_p2==pOriginal || triangle->_p3==pOriginal)
        {
            // fix the corner points to use the new point
            if (triangle->_p1==pOriginal)
            {
                triangle->_p1=pNew;
            }
            if (triangle->_p2==pOriginal)
            {
                triangle->_p2=pNew;
            }
            if (triangle->_p3==pOriginal)
            {
                triangle->_p3=pNew;
            }
            
            // fixes the edges so they point to use the new point.
            // edge can change if another edge already exists with same points and error metric.
            // be sure to update the edge's triangles so that if they get to 0 removeEdge will delete it.
            // be sure to keep edge around until done testing it since find() requires comparison of struct Edge members
            {
                osg::ref_ptr<Edge> eStore = triangle->_e1;
                triangle->_e1 = replaceEdgePoint(triangle->_e1.get(),pOriginal,pNew);
                if (eStore != triangle->_e1)
                {
                    EdgeSet::iterator eitr = _edgeSet.find(eStore);
                    if (eitr != _edgeSet.end())
                    {
                        eStore->_triangles.erase(triangle);
                        if ((*eitr) == eStore && eStore->_triangles.empty())
                            _edgeSet.erase(eitr);
                    }
                    triangle->_e1->addTriangle(triangle);
                }
            }
            {
                osg::ref_ptr<Edge> eStore = triangle->_e2;
                triangle->_e2 = replaceEdgePoint(triangle->_e2.get(),pOriginal,pNew);
                if (eStore != triangle->_e2)
                {
                    EdgeSet::iterator eitr = _edgeSet.find(eStore);
                    if (eitr != _edgeSet.end())
                    {
                        eStore->_triangles.erase(triangle);
                        if ((*eitr) == eStore && eStore->_triangles.empty())
                            _edgeSet.erase(eitr);
                    }
                    triangle->_e2->addTriangle(triangle);
                }
            }
            {
                osg::ref_ptr<Edge> eStore = triangle->_e3;
                triangle->_e3 = replaceEdgePoint(triangle->_e3.get(),pOriginal,pNew);
                if (eStore != triangle->_e3)
                {
                    EdgeSet::iterator eitr = _edgeSet.find(eStore);
                    if (eitr != _edgeSet.end())
                    {
                        eStore->_triangles.erase(triangle);
                        if ((*eitr) == eStore && eStore->_triangles.empty())
                            _edgeSet.erase(eitr);
                    }
                    triangle->_e3->addTriangle(triangle);
                }
            }
            
            // remove the triangle from the original point, and possibly the point if its the last triangle to use it
            removePoint(triangle, pOriginal);
            
            // add the triangle to that point
            addPoint(triangle,pNew);
        }
        
    }

    Edge* addEdge(Triangle* const triangle, Point* const p1, Point* const p2)
    {
        // osg::notify(osg::NOTICE)<<"        addEdge("<<p1<<","<<p2<<")"<<std::endl;
        // create and initialize the points and error metric before testing to see if an identical edge already exists
        osg::ref_ptr<Edge> edge = new Edge;
        edge->setOrderedPoints(p1, p2);
        
        //edge->setErrorMetric( computeErrorMetric( edge.get()));
        // this sets optimal point which setErrorMetric does not do. It also checks for boundary conditions
        setErrorMetricForEdgeNoInsert(edge.get());

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

    void removeEdge(Triangle* const triangle, osg::ref_ptr<Edge> edge)
    {

        // find the edge before doing anything to it which would affect the ability to find it
        EdgeSet::iterator itr = _edgeSet.find(edge);

        #ifdef DEBUG_EDGE_REMOVE_ITERATOR
        EdgeSet::iterator itr2 = findEdgeByAddress(edge);
        if (itr != itr2)
            osg::notify(osg::ALWAYS)<<"Error in Edge Collapse: edge="<<edge<<", edge by find="<<(*itr)<<", by address="<<(*itr2)<<"\n";
        #endif 

        if (itr!=_edgeSet.end())
        {
            edge->_triangles.erase(triangle);
            if (triangle->_e1 == edge)
                triangle->_e1 = 0;
            else if (triangle->_e2 == edge)
                triangle->_e2 = 0;
            else if (triangle->_e3 == edge)
                triangle->_e3 = 0;
            if (edge->_triangles.empty())
            {
                edge->_p1 = 0;
                edge->_p2 = 0;

                // edge no longer in use, so need to delete.
                _edgeSet.erase(itr);
            }
        }
    }

    Edge* replaceEdgePoint(Edge* edge, Point* const pOriginal, Point* const pNew)
    {
        bool couldBePreviouslyModified( false );
        if (edge->_p1==pNew || edge->_p2==pNew)
            couldBePreviouslyModified = true;

        if ((edge->_p1==pOriginal || edge->_p2==pOriginal) || couldBePreviouslyModified)
        {
            EdgeSet::iterator itr = _edgeSet.find(edge);
            if (itr!=_edgeSet.end())
            {
                if ((*itr) != edge && couldBePreviouslyModified)
                {
                    // find edge by address and remove because there appears to have been a duplicate created by some previous edge collapse
                    EdgeSet::iterator itr2 = findEdgeByAddress(edge);
                    // remove the edge from the list, as it is apparently superceded
                    if (itr2 != _edgeSet.end())
                        _edgeSet.erase(itr2);
                    edge = (*itr).get();
                }
                // remove the edge from the list, as its position in the list
                // may need to change once its values have been ammended 
                _edgeSet.erase(itr);
            }
            
            // modify its values
            if (edge->_p1==pOriginal) edge->_p1=pNew;
            if (edge->_p2==pOriginal) edge->_p2=pNew;

            if (SEdereference_check_less(edge->_p2,edge->_p1))
            {
                edge->_p1.swap(edge->_p2);
            }
            setErrorMetricForEdgeNoInsert( edge );

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

    Point* addPoint(Triangle* const triangle, const unsigned int p1)
    {
        return addPoint(triangle,_originalPointList[p1].get());
    }

    Point* addPoint(Triangle* const triangle, Point* point)
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

    void removePoint(Triangle* const triangle, Point* const point)
    {
        PointSet::iterator itr = _pointSet.find(point);
        if (itr!=_pointSet.end())
        {
            point->_triangles.erase(triangle);
            
            if (point->_triangles.empty())
            {
                // break links to other points before erasure - seems to be a good idea
                // but doing this clearing seems to degrade the final decimated object. 
                // Needs more investigation since there seems to be no harm in not clearing the links.
                // It's possible that some points are removed and then added again. <<<>>>gh
//                point->clearLinks();
                // point no longer in use, so need to delete.
                _pointSet.erase(itr);
            }
        }
        
    }

    unsigned int testPoint(const Point* const point) const
    {
        unsigned int numErrors = 0;
        
        for(TriangleSet::const_iterator itr=point->_triangles.begin();
            itr!=point->_triangles.end();
            ++itr)
        {
            Triangle* triangle = itr->get();
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
    
    unsigned int testAllPoints(void) const
    {
        unsigned int numErrors = 0;
        for(PointSet::const_iterator itr = _pointSet.begin();
            itr!=_pointSet.end();
            ++itr)
        {
            numErrors += testPoint(itr->get());
        }
        return numErrors;
    }

     unsigned int testEdge(const Edge* const edge) const
    {
        unsigned int numErrors = 0;
        for(TriangleSet::const_iterator teitr=edge->_triangles.begin();
            teitr!=edge->_triangles.end();
            ++teitr)
        {
            Triangle* triangle = teitr->get();
            if (!(triangle->_e1 == edge || triangle->_e2 == edge || triangle->_e3 == edge))
            {
                osg::notify(osg::NOTICE)<<"testEdge("<<edge<<"). triangle != point back to this edge"<<std::endl;
                osg::notify(osg::NOTICE)<<"                     triangle->_e1=="<<triangle->_e1.get()<<std::endl;
                osg::notify(osg::NOTICE)<<"                     triangle->_e2=="<<triangle->_e2.get()<<std::endl;
                osg::notify(osg::NOTICE)<<"                     triangle->_e3=="<<triangle->_e3.get()<<std::endl;
                ++numErrors;
            }
        }
        
        if (edge->_p1 == 0 || edge->_p2 == 0)
        {
            osg::notify(osg::NOTICE)<<"testEdge("<<edge<<")._p1 or _p2 is 0"<<std::endl;
            ++numErrors;
        }

        if (edge->_triangles.empty())
        {
            osg::notify(osg::NOTICE)<<"testEdge("<<edge<<")._triangles is empty"<<std::endl;
            ++numErrors;
        }
        return numErrors;
    }

    unsigned int testAllEdges(void) const
    {
        unsigned int numErrors = 0;
        for(EdgeSet::const_iterator itr = _edgeSet.begin();
            itr!=_edgeSet.end();
            ++itr)
        {
            numErrors += testEdge(itr->get());
        }
        return numErrors;
    }

    unsigned int testTriangle(Triangle* const triangle) const
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
    unsigned int testAllTriangles() const
    {
        unsigned int numErrors = 0;
        for(TriangleSet::const_iterator itr=_triangleSet.begin();
            itr!=_triangleSet.end();
            ++itr)
        {
            numErrors += testTriangle(itr->get());
        }
        return numErrors;
    }

    void linkCommonPoints(void)
    {
        for (PointSet::iterator itr1 = _pointSet.begin(); itr1 != _pointSet.end(); ++itr1)
        {
            PointSet::iterator itr2 = itr1;
            ++itr2;
            for (; itr2 != _pointSet.end(); ++itr2)
            {
                // thanks to the ordering of the point set, any points with same coordinates are arranged sequentially in the set
                if ((*itr1)->equals((*itr2).get()))
                {
                    osg::ref_ptr<PointLink> pLink = new PointLink;
                    pLink->_pA = (*itr1);
                    pLink->_pB = (*itr2);
                    (*itr1)->addLink(pLink.get());
                    (*itr2)->addLink(pLink.get());
                    ++_linksFound;
                }
                else
                    break;
            }
        }
    }

    unsigned int computeNumBoundaryEdges(void) const
    {
        unsigned int numBoundaryEdges = 0;
        for(EdgeSet::const_iterator itr = _edgeSet.begin();
            itr!=_edgeSet.end();
            ++itr)
        {
            if ((*itr)->isBoundaryEdge()) ++numBoundaryEdges;
        }
        return numBoundaryEdges;
    }

#define SHORTEDGEOP_LINK_SHARED_POINTS
#ifndef SHORTEDGEOP_LINK_SHARED_POINTS
    bool collapseEdge(Edge* edge, Point* pNew)
    {        
        typedef std::set< osg::ref_ptr<Edge> > LocalEdgeList;
        // these aren't dereferneced, just used for the purpose stated
        osg::ref_ptr<Edge> keep_edge_locally_referenced_to_prevent_premature_deletion = edge;
        osg::ref_ptr<Point> keep_point_locally_referenced_to_prevent_premature_deletion = pNew;
        osg::ref_ptr<Point> edge_p1 = edge->_p1;
        osg::ref_ptr<Point> edge_p2 = edge->_p2;
        
        TriangleMap  triangleMap;
        TriangleList triangles_p1;
        TriangleList triangles_p2;
        LocalEdgeList oldEdges;
        
        // one of these cases will be true and one false
        if (edge_p1 != pNew)    // p2 == keep point
        {
            //for all triangles attached to deletion point
            for(TriangleSet::iterator itr=edge_p1->_triangles.begin();
                itr!=edge_p1->_triangles.end();
                ++itr)
            {
                // looking for triangles that include the deletion point but are not sharing the edge to be collapsed
                if (edge->_triangles.count(*itr)==0)
                {
                    // making a list of triangles attached to deletion point but without collapse edge
                    Triangle* triangle = const_cast<Triangle*>(itr->get());
                    triangles_p1.push_back(triangle);
                    // making list of edges that are part of those triangles
                    oldEdges.insert(triangle->_e1);
                    oldEdges.insert(triangle->_e2);
                    oldEdges.insert(triangle->_e3);
                }
            }
        }
                
        if (edge_p2 != pNew)    // p1 == keep point
        {
            //for all triangles attached to deletion point
            for(TriangleSet::iterator itr=edge_p2->_triangles.begin();
                itr!=edge_p2->_triangles.end();
                ++itr)
            {
                // looking for triangles that include the deletion point but are not sharing the edge to be collapsed
                if (edge->_triangles.count(*itr)==0)
                {
                    // making a list of triangles attached to deletion point but without collapse edge
                    Triangle* triangle = const_cast<Triangle*>(itr->get());
                    triangles_p2.push_back(triangle);
                    // making list of edges that are part of those triangles
                    oldEdges.insert(triangle->_e1);
                    oldEdges.insert(triangle->_e2);
                    oldEdges.insert(triangle->_e3);
                }
            }
        }
        //for all the saved old edges, remove from edge list, set error metric to 0 and re-insert which should put it at head of list
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

        // another option would be to delete the triangles that share the edge and use replaceTrianglePoint to substitute pNew for the delete point

        //remove all the triangles around each point.
        // One of these triangle sets will be empty
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
        
        // we copy the edge's _triangles and iterate the copy of the triangle set to avoid invalidating iterators.
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

        // for all the triangles around the removed end point of the edge.
        // one of these triangle sets will be empty
        for(titr_p1 = triangles_p1.begin();
            titr_p1 != triangles_p1.end();
            ++titr_p1)
        {
            //create new triangles using the new point replacing the old end points
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
                //insert edges of the triangle to the needs-updating list if they contain p
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

#else
//#define SHORTEDGEOP_TEST_ENTITES

    bool collapseEdge(Edge* const removeEdge, Point* const pNew)
    {        
        typedef std::set< osg::ref_ptr<Edge> > LocalEdgeSet;
        typedef std::set< osg::ref_ptr<Point> > LocalPointSet;
        // these aren't dereferneced, just used for the purpose stated
        osg::ref_ptr<Edge> keep_edge_locally_referenced_to_prevent_premature_deletion = removeEdge;
        osg::ref_ptr<Point> keep_point_locally_referenced_to_prevent_premature_deletion = pNew;
        osg::ref_ptr<Point> point_save = removeEdge->_p1 != pNew ? removeEdge->_p2: removeEdge->_p1;
        osg::ref_ptr<Point> point_remove = removeEdge->_p1 != pNew ? removeEdge->_p1: removeEdge->_p2;

        TriangleList trianglesAttachedToRemoveEdge;
        TriangleList otherTrianglesAttachedToRemovePoint;
        LocalEdgeSet edges2UpdateErrorMetric;
        LinkedPointDataList linkedPointDataList;

        ++_collapsePasses;

        //for all triangles attached to remove edge
        // Triangles #1
        for(TriangleSet::iterator titr = removeEdge->_triangles.begin();
            titr != removeEdge->_triangles.end();
            ++titr)
        {
            Triangle* triangle = const_cast<Triangle*>(titr->get());
            trianglesAttachedToRemoveEdge.push_back(triangle);
        }
            
        //for all triangles attached to remove point that don't share the remove edge
        // Triangles #2
        for(TriangleSet::iterator titr=point_remove->_triangles.begin();
            titr!=point_remove->_triangles.end();
            ++titr)
        {
            // looking for triangles that include the deletion point but are not sharing the edge to be collapsed
            if (removeEdge->_triangles.count(*titr)==0)
            {
                // making a list of triangles attached to deletion point but not sharing collapse edge
                Triangle* triangle = const_cast<Triangle*>(titr->get());
                otherTrianglesAttachedToRemovePoint.push_back(triangle);
            }
        }
        // look for linked points co-located with the point being deleted
        // Triangles #3 and #4
        for (PointLinkList::iterator pitr = point_remove->_pointLinks.begin(); 
            pitr != point_remove->_pointLinks.end(); 
            ++pitr)
        {
            // we have another deletion point if this point is connected directly to the non-delete point by an edge
            // test to see which point of the PointLink is the current remove point
            Point* pointLinkedToRemovePoint = point_remove == (*pitr)->_pA ? (*pitr)->_pB.get(): (*pitr)->_pA.get();

            osg::ref_ptr<LinkedPointData> pLinkData = new LinkedPointData(pointLinkedToRemovePoint);

            // iterate triangles attached to pointLinkedToRemovePoint
            for (TriangleSet::iterator titr = pointLinkedToRemovePoint->_triangles.begin();
                titr != pointLinkedToRemovePoint->_triangles.end();
                ++titr)
            {
                Triangle* triangle = const_cast<Triangle*>(titr->get());
                // walk the points of the triangle looking for points co-located at the non-deletion point point_save
                Point* pointLinkedToSavePoint = triangle->_p1->equals(point_save.get()) ? triangle->_p1.get(): triangle->_p2->equals(point_save.get()) ? triangle->_p2.get(): triangle->_p3->equals(point_save.get()) ? triangle->_p3.get(): 0;

                pLinkData->setSavePoint(pointLinkedToSavePoint);

                if (pointLinkedToSavePoint)
                {
                    // these are triangles #3
                    // this triangle will be eliminated
                    pLinkData->addRemoveTriangle(triangle);
                }
                else
                {
                    // these are triangles #4
                    // this triangle shares the remove point location but doesn't share the removed edge
                    pLinkData->addModifyTriangle(triangle);
                }

            }
            linkedPointDataList.push_back(pLinkData);
        }

        // triangles #1
        // remove the triangles that will be collapsed
        for(TriangleList::iterator titr=trianglesAttachedToRemoveEdge.begin();
            titr!=trianglesAttachedToRemoveEdge.end();
            ++titr)
        {
            Triangle* triangle = const_cast<Triangle*>(titr->get());
            osgwTools::ShortEdgeCollapse::removeTriangle(triangle);
        }

        // triangles #2
        // remodel triangles that will be kept but with changes

        //for all the triangles around the remove point except ones sharing the collapse edge.
        for(TriangleList::iterator titr = otherTrianglesAttachedToRemovePoint.begin();
            titr != otherTrianglesAttachedToRemovePoint.end();
            ++titr)
        {
            Triangle* triangle = const_cast<Triangle*>(titr->get());
            replaceTrianglePoint(triangle, point_remove.get(), point_save.get());
            edges2UpdateErrorMetric.insert(triangle->_e1);
            edges2UpdateErrorMetric.insert(triangle->_e2);
            edges2UpdateErrorMetric.insert(triangle->_e3);
        }

        // Reset edge errors
        // reset the error metric for the changed edges, removing and reinserting them to sort properly
        for(LocalEdgeSet::iterator itr = edges2UpdateErrorMetric.begin();
            itr != edges2UpdateErrorMetric.end();
            ++itr)
        {
            //osg::notify(osg::NOTICE)<<"updateErrorMetricForEdge("<<itr->get()<<")"<<std::endl;
            updateErrorMetricForEdge(const_cast<Edge*>(itr->get()));
        }
        // we'll clear this array and re-use it later
        edges2UpdateErrorMetric.clear();

        for (LinkedPointDataList::iterator lpdlitr = linkedPointDataList.begin();
            lpdlitr != linkedPointDataList.end();
            ++lpdlitr)
        {
            LinkedPointData* linkedPointData = const_cast<LinkedPointData*>(lpdlitr->get());

            // triangles #3
            // remove the linked triangles that will be collapsed
            for(TriangleList::iterator titr=linkedPointData->_triToRemove.begin();
                titr!=linkedPointData->_triToRemove.end();
                ++titr)
            {
                Triangle* triangle = const_cast<Triangle*>(titr->get());
                osgwTools::ShortEdgeCollapse::removeTriangle(triangle);
            }

            // triangles #4
            // remodeling linked triangles that will be kept but with changes
            Point* p_remove = linkedPointData->getRemovePoint();
            Point* p_save = linkedPointData->getSavePoint();
            if (! p_save)
                p_save = point_save.get();

            //for all the triangles around the remove point except ones sharing the collapse edge.
            for(TriangleList::iterator titr=linkedPointData->_triToModify.begin();
                titr != linkedPointData->_triToModify.end();
                ++titr)
            {
                Triangle* triangle = const_cast<Triangle*>(titr->get());
                replaceTrianglePoint(triangle, p_remove, p_save);
                edges2UpdateErrorMetric.insert(triangle->_e1);
                edges2UpdateErrorMetric.insert(triangle->_e2);
                edges2UpdateErrorMetric.insert(triangle->_e3);
            }
        }

        // Reset edge errors
        // reset the error metric for the changed edges, removing and reinserting them to sort properly
        for(LocalEdgeSet::iterator itr = edges2UpdateErrorMetric.begin();
            itr != edges2UpdateErrorMetric.end();
            ++itr)
        {
            //osg::notify(osg::NOTICE)<<"updateErrorMetricForEdge("<<itr->get()<<")"<<std::endl;
            updateErrorMetricForEdge(const_cast<Edge*>(itr->get()));
        }

        //osg::notify(osg::ALWAYS)<<"Finishing Edge Collapse\n";

#ifdef SHORTEDGEOP_TEST_ENTITES
        unsigned int triTest, edgeTest, pointTest;
        triTest = testAllTriangles();
        edgeTest = testAllEdges();
        pointTest = testAllPoints();
        if (triTest || edgeTest || pointTest)
        {
            osg::notify(osg::ALWAYS)<<"Error in Edge Collapse: point errors="<<pointTest<<", edge errors="<<edgeTest<<", triangle errors="<<triTest<<"\n";
            CollapsePasses;
            if (triTest) testAllTriangles();
            if (edgeTest) testAllEdges();
            if (pointTest) testAllPoints();
        }
#endif
        return true;
    }
#endif // SHORTEDGEOP_LINK_SHARED_POINTS

    bool collapseMinimumErrorEdge(void)
    {
        if (!_edgeSet.empty())
        {
            Edge* edge = _edgeSet.begin()->get();

            if (edge->getErrorMetric()==FLT_MAX)
            {
                osg::notify(osg::INFO)<<"collapseMinimumErrorEdge() return false due to edge->getErrorMetric()==FLT_MAX"<<std::endl;
                return false;
            }

            // pNew will be the point that is closest to outside of bounding box. The point that remains after collapse
            osg::ref_ptr<Point> pNew = edge->_proposedPoint.get();
            //osg::notify(osg::INFO)<< "Collapsing min error edge: " << edge->getErrorMetric() << std::endl;
            return (collapseEdge(edge, pNew.get()));
        }
        osg::notify(osg::INFO)<<"collapseMinimumErrorEdge() return false due to _edgeSet.empty()"<<std::endl;
        return false;
    }

    void copyBackToGeometry();
};

struct CollectTriangleOperatorSE
{

    CollectTriangleOperatorSE():_ec(0) {}

    void setShortEdgeCollapse(ShortEdgeCollapse* ec) { _ec = ec; }
    
    ShortEdgeCollapse* _ec;    

    // for use  in the triangle functor.
    inline void operator()(unsigned int p1, unsigned int p2, unsigned int p3)
    {
        _ec->addTriangle(p1,p2,p3);
    }

};

/** \relates ShortEdgeOp */
typedef osg::TriangleIndexFunctor<CollectTriangleOperatorSE> CollectTriangleIndexFunctorSE;

class CopyVertexArrayToPointsVisitorSE : public osg::ArrayVisitor
{
    public:
        CopyVertexArrayToPointsVisitorSE(ShortEdgeCollapse::PointList& pointList):
            _pointList(pointList) {}
        
        virtual void apply(osg::Vec2Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i] = new ShortEdgeCollapse::Point;
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
                _pointList[i] = new ShortEdgeCollapse::Point;
                _pointList[i]->_index = i;
                
                _pointList[i]->_vertex = array[i];
            }
        }
        
        virtual void apply(osg::Vec4Array& array)
        {
            if (_pointList.size()!=array.size()) return;
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                _pointList[i] = new ShortEdgeCollapse::Point;
                _pointList[i]->_index = i;
                
                osg::Vec4& value = array[i];
                osg::Vec3& vertex = _pointList[i]->_vertex;
                vertex.set(value.x()/value.w(),value.y()/value.w(),value.z()/value.w());  
            }
        }
        
        ShortEdgeCollapse::PointList& _pointList;

    protected:
    
        CopyVertexArrayToPointsVisitorSE& operator = (const CopyVertexArrayToPointsVisitorSE&) { return *this; }

};

class CopyArrayToPointsVisitor : public osg::ArrayVisitor
{
    public:
        CopyArrayToPointsVisitor(ShortEdgeCollapse::PointList& pointList):
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
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
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
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
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
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
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
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                attributes.push_back(value.x());  
                attributes.push_back(value.y());  
                attributes.push_back(value.z());  
                attributes.push_back(value.w());  
            }
        }
        
        ShortEdgeCollapse::PointList& _pointList;
        
        
    protected:
    
        CopyArrayToPointsVisitor& operator = (const CopyArrayToPointsVisitor&) { return *this; }
};

void ShortEdgeCollapse::setGeometry(osg::Geometry* const geometry)
{
    _geometry = geometry;
    
#if( OSGWORKS_OSG_VERSION < 30108 )    
    // check to see if vertex attributes indices exists, if so expand them to remove them
    // This was removed from OSG 3.1.8 upwards
    if (_geometry->suitableForOptimization())
    {
        // removing coord indices
        osg::notify(osg::INFO)<<"ShortEdgeCollapse::setGeometry(..): Removing attribute indices"<<std::endl;
        _geometry->copyToAndOptimize(*_geometry);
    }
#endif    
    
    // check to see arrays are shared
    if (_geometry->containsSharedArrays())
    {
        // duplicating arrays
        osg::notify(osg::INFO)<<"ShortEdgeCollapse::setGeometry(..): Duplicate shared arrays"<<std::endl;
        _geometry->duplicateSharedArrays();
    }

    unsigned int numVertices = geometry->getVertexArray()->getNumElements();
        
    _originalPointList.resize(numVertices);
    
    // copy vertices across to local point list
    CopyVertexArrayToPointsVisitorSE copyVertexArrayToPoints(_originalPointList);
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

    // set bounding box to the point list which will initialize the bounding box extents
    _bBox.setBounds(_originalPointList);
    //report dimensions
    float xDim, yDim, zDim;
    getDimensions(xDim, yDim, zDim);
    osg::notify( osg::INFO )<<"  Bounding box dimensions x="<<xDim<<" y="<<yDim<<" z="<<zDim<<std::endl;

    CollectTriangleIndexFunctorSE collectTrianglesSE;
    collectTrianglesSE.setShortEdgeCollapse(this);
    
    _geometry->accept(collectTrianglesSE);
    
}



ShortEdgeCollapse::~ShortEdgeCollapse()
{
    std::for_each(_edgeSet.begin(),_edgeSet.end(),SEdereference_clear());

    std::for_each(_triangleSet.begin(),_triangleSet.end(),SEdereference_clear());
    std::for_each(_pointSet.begin(),_pointSet.end(),SEdereference_clear());
    std::for_each(_originalPointList.begin(),_originalPointList.end(),SEdereference_clear());
}

class CopyPointsToArrayVisitor : public osg::ArrayVisitor
{
    public:
        CopyPointsToArrayVisitor(ShortEdgeCollapse::PointList& pointList):
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
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
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
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                if (_index+1<attributes.size()) array[i].set(attributes[_index],attributes[_index+1]);
            }
            _index += 2;
        }

        virtual void apply(osg::Vec3Array& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                if (_index+2<attributes.size()) array[i].set(attributes[_index],attributes[_index+1],attributes[_index+2]);
            }
            _index += 3;
        }
        
        virtual void apply(osg::Vec4Array& array)
        {
            array.resize(_pointList.size());
        
            for(unsigned int i=0;i<_pointList.size();++i) 
            {
                ShortEdgeCollapse::FloatList& attributes = _pointList[i]->_attributes;
                if (_index+3<attributes.size()) array[i].set(attributes[_index],attributes[_index+1],attributes[_index+2],attributes[_index+3]);
            }
            _index += 4;
        }
        
        ShortEdgeCollapse::PointList& _pointList;
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
        CopyPointsToVertexArrayVisitor(ShortEdgeCollapse::PointList& pointList):
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
        
        ShortEdgeCollapse::PointList& _pointList;
        
    protected:
    
        CopyPointsToVertexArrayVisitor& operator = (const CopyPointsToVertexArrayVisitor&) { return *this; }
};



void ShortEdgeCollapse::copyBackToGeometry()
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

    typedef std::set< osg::ref_ptr<Triangle>, SEdereference_less >    TrianglesSorted;
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

ShortEdgeOp::ShortEdgeOp(double sampleRatio, double maxFeature, unsigned int maxSteps):
            _sampleRatio(sampleRatio),
            _maxFeature(maxFeature),
            _triStrip( false ),
            _smoothing( false ),
            _ignoreBoundaries( false ),
            _minPrim( 4 ),
            _maxSteps( maxSteps ) 
            {};

ShortEdgeOp::ShortEdgeOp( const ShortEdgeOp& rhs, const osg::CopyOp& copyOp )
{
    _sampleRatio = rhs._sampleRatio;
    _maxFeature = rhs._maxFeature;
    _triStrip = rhs._triStrip;
    _smoothing = rhs._smoothing;
    _ignoreBoundaries = rhs._ignoreBoundaries;
    _minPrim = rhs._minPrim;
    _maxSteps = rhs._maxSteps ;
}

void ShortEdgeOp::decimate(osg::Geometry& geometry)
{
    osg::notify( osg::INFO)<<"++++++++++++short edge operator************"<<std::endl;

    ShortEdgeCollapse sec;
    sec.setIgnoreBoundaries( _ignoreBoundaries );
    sec.setGeometry(&geometry);
    sec.updateErrorMetricForAllEdges();
    sec.linkCommonPoints();
    unsigned int numOriginalPrimitives = sec._triangleSet.size();
    osg::notify( osg::INFO ) << "  Primitives Available to Decimate: "<<numOriginalPrimitives<<std::endl;
    osg::notify( osg::INFO ) << "  Links found between points: "<<sec._linksFound<<std::endl;
    
    if (numOriginalPrimitives < getMinPrimitives()){
       osg::notify( osg::INFO ) << "  Geometry too small to decimate: skipping...\n";
        return;
    }
    if (getSampleRatio()<1.0 && getMaxFeature()>0.0)
    {
        unsigned int decCt( 0 );
        osg::notify(osg::INFO)<<"   Collapsed edge#="<<decCt<<" Triangles remaining="<<sec._triangleSet.size()<<" Next edge length="<<(*sec._edgeSet.begin())->getErrorMetric()<<" vs "<<getMaxFeature()<<std::endl;
        while (!sec._edgeSet.empty() && decCt < _maxSteps &&
               continueDecimation((*sec._edgeSet.begin())->getErrorMetric() , numOriginalPrimitives, sec._triangleSet.size()) && 
               sec.collapseMinimumErrorEdge() && sec._triangleSet.size() > _minPrim)
        {
            osg::notify(osg::INFO)<<"   Collapsed edge#="<<decCt<<" Triangles remaining="<<sec._triangleSet.size()<<" Next edge length="<<(*sec._edgeSet.begin())->getErrorMetric()<<" vs "<<getMaxFeature()<<std::endl;
            ++decCt;
        }

        osg::notify(osg::INFO)<<"******* AFTER SHORT EDGE COLLAPSE *********"<<sec._triangleSet.size()<<std::endl;
    }

    osg::notify(osg::INFO)<<"Number of triangle errors after short edge collapse= "<<sec.testAllTriangles()<<std::endl;
    osg::notify(osg::INFO)<<"Number of edge errors before short edge collapse= "<<sec.testAllEdges()<<std::endl;
    osg::notify(osg::INFO)<<"Number of point errors after edge collapse= "<<sec.testAllPoints()<<std::endl;
    osg::notify(osg::INFO)<<"Number of triangles= "<<sec._triangleSet.size()<<std::endl;
    osg::notify(osg::INFO)<<"Number of points= "<<sec._pointSet.size()<<std::endl;
    osg::notify(osg::INFO)<<"Number of edges= "<<sec._edgeSet.size()<<std::endl;
    osg::notify(osg::INFO)<<"Number of boundary edges= "<<sec.computeNumBoundaryEdges()<<std::endl;

    if (!sec._edgeSet.empty())
    {
        osg::notify(osg::INFO)<<std::endl<<"Short Edge Op, Polygons in = "<<numOriginalPrimitives<<"\t;out = "<<sec._triangleSet.size()<<"\terror="<<(*sec._edgeSet.begin())->getErrorMetric()<<"\tvs "<<getMaxFeature()<<std::endl<<std::endl;
        osg::notify(osg::INFO)<<           "        !sec._edgeSet.empty()  = "<<!sec._edgeSet.empty()<<std::endl;
        osg::notify(osg::INFO)<<           "        continueDecimation(,,)  = "<<continueDecimation((*sec._edgeSet.begin())->getErrorMetric() , numOriginalPrimitives, sec._triangleSet.size())<<std::endl;
    }
    
    sec.copyBackToGeometry();

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
