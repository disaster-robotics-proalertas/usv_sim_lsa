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

#ifndef __OSGWTOOLS_PARALLEL_VISITOR_H__
#define __OSGWTOOLS_PARALLEL_VISITOR_H__ 1


#include <osgwTools/Export.h>
#include <osg/ref_ptr>
#include <osg/Node>



namespace osgwTools
{


/** \brief Simultaneously walks two scene graphs
and executes a callback if Nodes are not identical */
class OSGWTOOLS_EXPORT ParallelVisitor
{
public:
    ParallelVisitor( osg::Node* sgA, osg::Node* sgB );
    ~ParallelVisitor();

    /** Compare sgA and sgB.
    \return True if match and False otherwise. */
    bool compare();

    /** \brief Callback executed if isMatch(nodeA,nodeB) returns False.

    Typical usage: isMatch detects that Nodes differ; callback
    performs operations to sync them. */
    struct ParallelVisitorCallback
    {
        ParallelVisitorCallback() {}
        virtual ~ParallelVisitorCallback() {}

        virtual bool operator()( osg::Node& grpA, osg::Node& grpB ) = 0;
    };
    void setCallback( ParallelVisitorCallback* pvcb );
    ParallelVisitor::ParallelVisitorCallback* getCallback() const;

protected:
    /** Overrides to specify your own custom comparison criteria. Return
    true if the two Nodes match, and false otherwise. If this function
    returns false and _pvcb is not NULL, (*_pvcb)() is called. */
    virtual bool isMatch( const osg::Node& nodeA, const osg::Node& nodeB ) const;

    bool recurseCompare( osg::Node* nodeA, osg::Node* nodeB );

    bool _compareResult;

    osg::ref_ptr< osg::Node > _sgA;
    osg::ref_ptr< osg::Node > _sgB;

    ParallelVisitorCallback* _pvcb;
};


// osgwTools
}

// __OSGWTOOLS_PARALLEL_VISITOR_H__
#endif
