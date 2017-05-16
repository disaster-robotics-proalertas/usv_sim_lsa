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

#ifndef __OSGWTOOLS_STATE_TRACKING_NODE_VISITOR_H__
#define __OSGWTOOLS_STATE_TRACKING_NODE_VISITOR_H__ 1

#include <osgwTools/Export.h>
#include <osg/NodeVisitor>
#include <deque>


namespace osgwTools
{


/** \class StateTrackingNodeVisitor StateTrackingNodeVisitor.h <osgwTools/StateTrackingNodeVisitor.h>
\brief A NodeVisitor with facilities for tracking StateSets.

This NodeVisitor provides a stack of StateSets and two routines to push and
pop state. Classes that derive from StateTrackingNodeVisitor are responsible
for calling pushStateSet() and popStateSet() when required. Typically, 
call pushStateSet() on entry to an apply() method, and popStateSet() just
prior to exit from an apply() method. NodeVisitor::traverse() should be called
between pushStateSet() and popStateSet().

After the current StateSet has been pushed onto the stack with a call to
pushStateSet(), the derived class can access the accumulated state for the
currently visited Node by accessing the back of the _stateStack deque.

Here's a typical use case:

\code
void MyClass::apply( osg::Group& group )
{
    // Push the current StateSet onto the stack (It's OK if
    // group->getStateSet() == NULL.):
    pushStateSet( group->getStateSet() );

    // Access the current accumulated StateSet for this node.
    const osg::StateSet* currentState = _stateStack.back().get();

    // Add your own code here to do whatevery you need to
    // with the current accumulated state.

    // Traverse subgraph.
    traverse( group );

    // Pop the stack back to the previous StateSet.
    popStateSet();
}
\endcode

*/
class OSGWTOOLS_EXPORT StateTrackingNodeVisitor : public osg::NodeVisitor
{
public:
    StateTrackingNodeVisitor( osg::NodeVisitor::TraversalMode mode = osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN );
    virtual ~StateTrackingNodeVisitor();

    virtual const char* libraryName() const { return( "osgwTools" ); }
    virtual const char* className() const { return( "StateTrackingNodeVisitor" ); }

    unsigned int getStateStackSize() { return( _stateStack.size() ); }

protected:
    typedef std::deque< osg::ref_ptr< osg::StateSet > > StateSetStack;
    StateSetStack _stateStack;

    /** Push the StateSet \c ss onto the stack. Derived classes
    typically call this on entry to an apply() method. After
    calling this function, derived classes can access the
    current accumulated state with \c _stateStack.back().
    */
    void pushStateSet( osg::StateSet* ss );
    /** Pop the top of the state stack. Derived classes typically
    call this prior to returning from an apply() method. */
    void popStateSet();

    /** For apply() methods that don't require access to the
    current accumulated state, pushTraversePop() can be used
    in place of NodeVisitor::traverse(node). pushTraversePop()
    is equivalent to the following code:
    \code
        pushStateSet( ss );
        traverse( node );
        popStateSet();
    \endcode
    */
    void pushTraversePop( osg::StateSet* ss, osg::Node& node );
};


// osgwTools
}


// __OSGWTOOLS_STATE_TRACKING_NODE_VISITOR_H__
#endif
