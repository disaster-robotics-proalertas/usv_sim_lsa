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

#ifndef __OSGWQUERY_QUERY_STATS_H__
#define __OSGWQUERY_QUERY_STATS_H__ 1


#include <osgwQuery/Export.h>

#include <osg/Node>
#include <osg/Camera>
#include <osgText/Text>
#include <osgGA/GUIEventHandler>


namespace osgwQuery
{


/** \addtogroup GutheQuery */
/*@{*/


/** \class QueryStats QueryStats.h <osgwQuery/QueryStats.h>
\brief On-screen display of Guthe statistics regarding the query status of a given node.
*/
class OSGWQUERY_EXPORT QueryStats : public osg::Referenced
{
public:
    QueryStats( osg::Node* node );

    osg::Node* getNode() { return( _node.get() ); }

    /** Returns a post-render HUD camera and osgtext children to display
    query statistics. You should add the return value as a child to your
    root node. */
    osg::Node* getSceneGraph();

    /** By default, accumulation is disabled and calls to incFrames, incQueries,
    etc, are ignored. This function toggles the current accumulation state and
    returns the state after toggling. To obtain the current accumulation state
    without toggling, pass false for the \c toggle parameter. */
    bool toggleAccumulate( bool toggle=true );

    /** Enables stats display to console. When enabled, current stats are written
    to the console by the incFrame() method. Console display is off by default. */
    void setConsoleDisplay( bool display=true ) { _consoleDisplay=display; }
    bool getConsoleDisplay() const { return( _consoleDisplay ); }

    void clear();
    unsigned int incFrames( unsigned int n=1 );
    unsigned int incQueries( unsigned int n=1 );
    unsigned int incOccluded( unsigned int n=1 );
    unsigned int incRtLessQt( unsigned int n=1 );
    unsigned int incCGreaterB( unsigned int n=1 );
    unsigned int incFrustum( unsigned int n=1 );

    void setPoccl( const float poccl );

protected:
    unsigned int internalInc( unsigned int& val, osgText::Text* text, unsigned int n );

    osg::ref_ptr< osg::Node > _node;

    bool _accum;
    bool _consoleDisplay;

    unsigned int _numFrames;
    unsigned int _numQueries;
    unsigned int _numOccluded;
    unsigned int _numRtLessQt;
    unsigned int _numCGreaterB;
    unsigned int _numFrustum;

    osg::ref_ptr< osg::Camera > _cam;
    osg::ref_ptr< osgText::Text > _labels;
    osg::ref_ptr< osgText::Text > _frames;
    osg::ref_ptr< osgText::Text > _queries;
    osg::ref_ptr< osgText::Text > _occluded;
    osg::ref_ptr< osgText::Text > _rtLessQt;
    osg::ref_ptr< osgText::Text > _cGreaterB;
    osg::ref_ptr< osgText::Text > _frustum;
    osg::ref_ptr< osgText::Text > _poccl;
};


/** \class QueryStatsHandler QueryStats.h <osgwQuery/QueryStats.h>
\brief An event handler that toggles QueryStats active accumulation with the 'a' key.
*/
class OSGWQUERY_EXPORT QueryStatsHandler : public osgGA::GUIEventHandler
{
public:
    QueryStatsHandler( osgwQuery::QueryStats* qs );

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );

protected:
    osg::ref_ptr< osgwQuery::QueryStats > _qs;
};

/*@}*/


// osgwQuery
}

// __OSGWQUERY_QUERY_STATS_H__
#endif
