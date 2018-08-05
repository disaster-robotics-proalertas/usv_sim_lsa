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

#include "osgwTools/NodePathUtils.h"
#include <osg/Node>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Notify>

#ifdef BOOST_FOUND
#  include <boost/algorithm/string/replace.hpp>
#endif
#include <string>
#include <vector>
#include <iostream>
#include <sstream>


namespace osgwTools
{


/** \brief A class to enclose a string in double quotes, preserving any double quotes that exist
in the string. 

If a double quote exists in the string, this code converts it to a pair
of double quotes. For example:
Input:  test"string
Output: "test""string"
*/
class QuotedString
{
public:
    QuotedString()
    {
    }
    QuotedString( const std::string& unquotedString )
    {
        _unquoted = unquotedString;
        _quoted = addQuotes( _unquoted );
    }

    void setQuotedString( const std::string& quotedString )
    {
        _quoted = quotedString;
        _unquoted = removeQuotes( _quoted );
    }

    std::string getUnquoted() const
    {
        return( _unquoted );
    }
    std::string getQuoted() const
    {
        return( _quoted );
    }

protected:
    std::string _quoted, _unquoted;

    static std::string addQuotes( const std::string& in )
    {
#ifdef BOOST_FOUND
        std::string quoted( in );
        boost::algorithm::replace_all( quoted, "\"", "\"\"" );
        return( std::string( "\"" ) + quoted + std::string( "\"" ) );
#else
        std::string quoted( "\"" );
        std::string::size_type lastPos( 0 );
        std::string::size_type pos = in.find( '"', lastPos );
        while( pos != std::string::npos )
        {
            quoted += in.substr( lastPos, pos-lastPos+1 );
            quoted += '"';
            lastPos = pos+1;
            pos = in.find( '"', lastPos );
        }
        quoted += in.substr( lastPos, in.length()-lastPos );
        quoted += '"';
        return( quoted );
#endif
    }
    static std::string removeQuotes( const std::string& in )
    {
#ifdef BOOST_FOUND
        std::string unquoted( in.substr( 1, in.length()-2 ) );
        boost::algorithm::replace_all( unquoted, "\"\"", "\"" );
        return( unquoted );
#else
        std::string unquoted;
        std::string::size_type lastPos( 1 );
        std::string::size_type pos = in.find( "\"\"", lastPos );
        while( pos != std::string::npos )
        {
            unquoted += in.substr( lastPos, pos-lastPos+1 );
            lastPos = pos+2;
            pos = in.find( "\"\"", lastPos );
        }
        unquoted += in.substr( lastPos, in.length()-lastPos-1 );
        return( unquoted );
#endif
    }
};
std::ostream& operator<<( std::ostream& ostr, const QuotedString& qstr )
{
    ostr << qstr.getQuoted();
    return( ostr );
}
std::istream& operator>>( std::istream& istr, QuotedString& qstr )
{
    std::string str;
    char ch;
    bool done( false );
    unsigned int quoteCount( 0 );
    while( !done )
    {
        ch = istr.peek();
        if( ch == '"' )
            quoteCount++;
        if( ( ch != '"' ) && ( ( quoteCount & 0x1) == 0 ) )
            done = true;
        else
        {
            istr.read( &ch, 1 );
            str += ch;
        }
    }
    if( !str.empty() )
        qstr.setQuotedString( str );
    return( istr );
}

std::ostream& operator<<( std::ostream& ostr, const NodeData& nd )
{
    ostr << nd._index << "," <<
        QuotedString( nd._className ) << "," <<
        QuotedString( nd._objectName ) << ":";
    return( ostr );
}
std::istream& operator>>( std::istream& istr, NodeData& nd )
{
    QuotedString className, objectName;
    char sep;

    istr >> nd._index >> sep >>
        className >> sep >> objectName >> sep;
    nd._className = className.getUnquoted();
    nd._objectName = objectName.getUnquoted();

    return( istr );
}


IndexedNodePath nodePathToIndexed( const osg::NodePath& nodePath )
{
    IndexedNodePath inp;
    osg::Group* parent = NULL;
    osg::NodePath::const_iterator pathIter;
    for( pathIter = nodePath.begin(); pathIter != nodePath.end(); ++pathIter )
    {
        if( parent != NULL )
        {
            NodeData nd( parent->getChildIndex( *pathIter ), **pathIter );
            inp.push_back( nd );
        }
        parent = ( *pathIter )->asGroup();
    }
    return( inp );
}
osg::NodePath indexedToNodePath( const IndexedNodePath& indexedNodePath, osg::Group* root )
{
    osg::NodePath np;
    np.push_back( root );

    osg::Group* parent = root;
    IndexedNodePath::const_iterator pathIter;
    for( pathIter = indexedNodePath.begin(); pathIter != indexedNodePath.end(); ++pathIter )
    {
        const NodeData& nd = *pathIter;
        osg::Node* child = nd.findNode( parent );
        if( child != NULL )
        {
            np.push_back( child );
            parent = child->asGroup();
        }
    }
    return( np );
}


std::string indexedToString( const IndexedNodePath& indexedNodePath )
{
    std::ostringstream ostr;

    IndexedNodePath::const_iterator pathIter;
    for( pathIter = indexedNodePath.begin(); pathIter != indexedNodePath.end(); ++pathIter )
    {
        const NodeData& nd = *pathIter;
        ostr << nd;
    }
    return( ostr.str() );
}
IndexedNodePath stringToIndexed( const std::string& stringPath )
{
    IndexedNodePath inp;
    std::istringstream istr( stringPath );
    while( istr.good() )
    {
        NodeData nd;
        istr >> nd;
        if( istr.eof() )
            break;
        inp.push_back( nd );
    }
    return( inp );
}


std::string nodePathToString( const osg::NodePath& nodePath )
{
    return( indexedToString( nodePathToIndexed( nodePath ) ) );
}
osg::NodePath stringToNodePath( const std::string& stringPath, osg::Group* root )
{
    return( indexedToNodePath( stringToIndexed( stringPath ), root ) );
}


osg::Node* findNode( const IndexedNodePath& indexedNodePath, osg::Group* root )
{
    osg::NodePath np = indexedToNodePath( indexedNodePath, root );
    return( np.back() );
}
osg::Node* findNode( const std::string& stringPath, osg::Group* root )
{
    return( findNode( stringToIndexed( stringPath ), root ) );
}




NodeData::NodeData()
{
    _index = 0;
}
NodeData::NodeData( unsigned int index, const osg::Node& node )
  : _index( index ),
    _className( node.className() ),
    _objectName( node.getName() )
{
}

osg::Node* NodeData::findNode( osg::Group* parent ) const
{
    const bool badIndex( _index >= parent->getNumChildren() );
    bool badClassName( false );
    bool badObjectName( false );

    osg::Node* indexChild( NULL );
    if( !badIndex )
    {
        indexChild = parent->getChild( _index );
        badClassName = ( indexChild->className() != _className );
        badObjectName = ( indexChild->getName() != _objectName );
    }

    if( badIndex || badClassName || badObjectName )
    {
        if( badIndex )
            osg::notify( osg::WARN ) << "osgwTools::NodeData::findNode: Index out of range: " << _index <<
                ", parent has " << parent->getNumChildren() << std::endl;
        else if( badClassName )
            osg::notify( osg::WARN ) << "osgwTools::NodeData::findNode: _className: " << _className <<
                ", doesn't match indexChild " << _index << ": " << indexChild->className() << std::endl;
        else if( badObjectName )
            osg::notify( osg::WARN ) << "osgwTools::NodeData::findNode: _objectName: " << _objectName <<
                ", doesn't match indexChild " << _index << ": " << indexChild->getName() << std::endl;

        unsigned int bestMatchIdx( 0 );
        bool foundSomething( false );
        unsigned int idx;
        for( idx=0; idx<parent->getNumChildren(); idx++ )
        {
            osg::Node* node = parent->getChild( idx );
            if( ( node->className() == _className ) &&
                    ( node->getName() == _objectName ) )
            {
                osg::notify( osg::WARN ) << "  Selected alternate: index " << idx << std::endl;
                return( node );
            }
            else if( ( node->className() == _className ) ||
                    ( node->getName() == _objectName ) )
            {
                bestMatchIdx = idx;
                foundSomething = true;
            }
        }
        if( indexChild != NULL )
        {
            osg::notify( osg::WARN ) << "  Selected alternate with matching index." << std::endl;
            return( indexChild );
        }
        else if( foundSomething && ( bestMatchIdx < parent->getNumChildren() ) )
        {
            osg::notify( osg::WARN ) << "  Best match: index " << bestMatchIdx << std::endl;
            return( parent->getChild( bestMatchIdx ) );
        }
        else
        {
            osg::notify( osg::WARN ) << "  No match. Returning NULL" << std::endl;
            return( NULL );
        }
    }

    return( indexChild );
}

bool NodeData::operator==( const NodeData& rhs )
{
    return( ( _index == rhs._index ) &&
        ( _className == rhs._className ) &&
        ( _objectName == rhs._objectName ) );
}
bool NodeData::operator!=( const NodeData& rhs )
{
    return( !( this->operator==( rhs ) ) );
}




#ifdef OSGWORKS_BUILD_TESTS

int testQuoting( const std::string& input, const std::string& expectedResult )
{
    QuotedString qs( input );
    if( qs.getQuoted() != expectedResult )
    {
        osg::notify( osg::FATAL ) << "QuotedString adding quotes: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << qs.getQuoted() << std::endl;
        return( 1 );
    }
    return( 0 );
}
int testUnquoting( const std::string& input, const std::string& expectedResult )
{
    QuotedString qs;
    qs.setQuotedString( input );
    if( qs.getUnquoted() != expectedResult )
    {
        osg::notify( osg::FATAL ) << "QuotedString removing quotes: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << qs.getUnquoted() << std::endl;
        return( 1 );
    }
    return( 0 );
}
int testQuotedOStream( const std::string& input, const std::string& expectedResult )
{
    QuotedString qs( input );
    std::ostringstream ostr;
    ostr << qs;
    if( ostr.str() != expectedResult )
    {
        osg::notify( osg::FATAL ) << "QuotedString writing to ostream: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << ostr.str() << std::endl;
        return( 1 );
    }
    return( 0 );
}
int testQuotedIStream( const std::string& input, const std::string& expectedResult )
{
    std::istringstream istr( input );
    QuotedString qs;
    istr >> qs;
    if( qs.getUnquoted() != expectedResult )
    {
        osg::notify( osg::FATAL ) << "QuotedString reading from istream: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << qs.getUnquoted() << std::endl;
        return( 1 );
    }
    return( 0 );
}

int testNodeDataOStream( const NodeData& input, const std::string& expectedResult )
{
    std::ostringstream ostr;
    ostr << input;
    if( ostr.str() != expectedResult )
    {
        osg::notify( osg::FATAL ) << "NodeData writing to ostream: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << ostr.str() << std::endl;
        return( 1 );
    }
    return( 0 );
}
int testNodeDataIStream( const std::string& input, const NodeData& expectedResult )
{
    std::istringstream istr( input );
    NodeData nd;
    istr >> nd;
    if( nd != expectedResult )
    {
        osg::notify( osg::FATAL ) << "NodeData reading from istream: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << nd << std::endl;
        return( 1 );
    }
    return( 0 );
}

int testNodePathToString( const osg::NodePath& input, const std::string& expectedResult )
{
    std::string result = nodePathToString( input );
    if( result != expectedResult )
    {
        osg::notify( osg::FATAL ) << "NodePath conversion to string: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << result << std::endl;
        return( 1 );
    }
    return( 0 );
}
int testFindNode( const std::string& input, osg::Group* parent, osg::Node* expectedResult )
{
    osg::Node* result = findNode( input, parent );
    if( result != expectedResult )
    {
        osg::notify( osg::FATAL ) << "findNode from string: test failure." << std::endl;
        osg::notify( osg::FATAL ) << "  Expected: " << expectedResult << std::endl;
        osg::notify( osg::FATAL ) << "  Received: " << result << std::endl;
        return( 1 );
    }
    return( 0 );
}


int testNodePathUtils()
{
    // tests for the QuotedString class

    // Test adding quotes
    {
        std::string input( "abc" );
        std::string expectedResult( "\"abc\"" );
        if( testQuoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input;
        std::string expectedResult( "\"\"" );
        if( testQuoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "\"\"\"\"\"" );
        std::string expectedResult( "\"\"\"\"\"\"\"\"\"\"\"\"" );
        if( testQuoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "foo\"bar" );
        std::string expectedResult( "\"foo\"\"bar\"" );
        if( testQuoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "data type: \"unknown\"" );
        std::string expectedResult( "\"data type: \"\"unknown\"\"\"" );
        if( testQuoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "\"initial quote" );
        std::string expectedResult( "\"\"\"initial quote\"" );
        if( testQuoting( input, expectedResult ) != 0 )
            return( 1 );
    }

    // Test removing quotes
    {
        std::string input( "\"abc\"" );
        std::string expectedResult( "abc" );
        if( testUnquoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "\"\"" );
        std::string expectedResult;
        if( testUnquoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "\"\"\"\"\"\"\"\"\"\"\"\"" );
        std::string expectedResult( "\"\"\"\"\"" );
        if( testUnquoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "\"foo\"\"bar\"" );
        std::string expectedResult( "foo\"bar" );
        if( testUnquoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "\"data type: \"\"unknown\"\"\"" );
        std::string expectedResult( "data type: \"unknown\"" );
        if( testUnquoting( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "\"\"\"initial quote\"" );
        std::string expectedResult( "\"initial quote" );
        if( testUnquoting( input, expectedResult ) != 0 )
            return( 1 );
    }

    // Test writing to ostream
    {
        std::string input( "abc" );
        std::string expectedResult( "\"abc\"" );
        if( testQuotedOStream( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "data type: \"unknown\"" );
        std::string expectedResult( "\"data type: \"\"unknown\"\"\"" );
        if( testQuotedOStream( input, expectedResult ) != 0 )
            return( 1 );
    }

    // test reading from istream
    {
        // NOTE comma after final quote. Otherwise, istringstream reads garbage chars.
        std::string input( "\"abc\"," );
        std::string expectedResult( "abc" );
        if( testQuotedIStream( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        // NOTE comma after final quote. Otherwise, istringstream reads garbage chars.
        std::string input( "\"\"\"initial quote\"," );
        std::string expectedResult( "\"initial quote" );
        if( testQuotedIStream( input, expectedResult ) != 0 )
            return( 1 );
    }



    // NodeData tests

    // Test writing to ostream
    {
        osg::ref_ptr< osg::Group > grp = new osg::Group;
        grp->setName( "test group" );
        NodeData input( 121, *grp );
        std::string expectedResult( "121,\"Group\",\"test group\":" );
        if( testNodeDataOStream( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        osg::ref_ptr< osg::MatrixTransform > mt = new osg::MatrixTransform;
        NodeData input( 0, *mt );
        std::string expectedResult( "0,\"MatrixTransform\",\"\":" );
        if( testNodeDataOStream( input, expectedResult ) != 0 )
            return( 1 );
    }

    // Test reading from istream
    {
        std::string input( "121,\"Group\",\"test group\":" );
        osg::ref_ptr< osg::Group > grp = new osg::Group;
        grp->setName( "test group" );
        NodeData expectedResult( 121, *grp );
        if( testNodeDataIStream( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        std::string input( "0,\"MatrixTransform\",\"\":" );
        osg::ref_ptr< osg::MatrixTransform > mt = new osg::MatrixTransform;
        NodeData expectedResult( 0, *mt );
        if( testNodeDataIStream( input, expectedResult ) != 0 )
            return( 1 );
    }


    // Test NodePath conversion to string.
    {
        osg::ref_ptr< osg::Group > parent = new osg::Group;
        parent->setName( "parent" );
        osg::ref_ptr< osg::Group > child = new osg::Group;
        child->setName( "child" );
        parent->addChild( child.get() );

        osg::NodePath input;
        input.push_back( parent.get() );
        input.push_back( child.get() );

        std::string expectedResult( "0,\"Group\",\"child\":" );
        if( testNodePathToString( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        osg::ref_ptr< osg::Group > parent = new osg::Group;
        parent->setName( "parent" );
        osg::ref_ptr< osg::Group > child = new osg::Group;
        child->setName( "child" );
        parent->addChild( child.get() );
        osg::ref_ptr< osg::MatrixTransform > childMT = new osg::MatrixTransform;
        childMT->setName( "childMT" );
        parent->addChild( childMT.get() );

        osg::NodePath input;
        input.push_back( parent.get() );
        input.push_back( childMT.get() );

        std::string expectedResult( "1,\"MatrixTransform\",\"childMT\":" );
        if( testNodePathToString( input, expectedResult ) != 0 )
            return( 1 );
    }
    {
        osg::ref_ptr< osg::Group > parent = new osg::Group;
        parent->setName( "parent" );
        osg::ref_ptr< osg::Group > child = new osg::Group;
        child->setName( "child" );
        parent->addChild( child.get() );
        osg::ref_ptr< osg::MatrixTransform > childMT = new osg::MatrixTransform;
        childMT->setName( "childMT" );
        child->addChild( childMT.get() );

        osg::NodePath input;
        input.push_back( parent.get() );
        input.push_back( child.get() );
        input.push_back( childMT.get() );

        std::string expectedResult( "0,\"Group\",\"child\":0,\"MatrixTransform\",\"childMT\":" );
        if( testNodePathToString( input, expectedResult ) != 0 )
            return( 1 );
    }


    // Test findNode from a string.
    {
        osg::ref_ptr< osg::Group > parent = new osg::Group;
        parent->setName( "parent" );
        osg::ref_ptr< osg::Group > child = new osg::Group;
        child->setName( "child" );
        parent->addChild( child.get() );

        std::string input( "0,\"Group\",\"child\":" );
        osg::Node* expectedResult = child.get();
        if( testFindNode( input, parent.get(), expectedResult ) != 0 )
            return( 1 );

        // This is a bad index, but test should still pass.
        // NodeData::findNode should find child 0 with matching className and object name.
        // This test will display warning messages to the console:
        //     NodeData::findNode: Index out of range: 4, parent has 1
        //       Selected alternate: index 0
        osg::notify( osg::WARN ) << "----------------------------------------" << std::endl;
        osg::notify( osg::WARN ) << "--- Testing. Warning output is expected:" << std::endl;
        input = std::string( "4,\"Group\",\"child\":" );
        if( testFindNode( input, parent.get(), expectedResult ) != 0 )
            return( 1 );
    }
    {
        osg::ref_ptr< osg::Group > parent = new osg::Group;
        parent->setName( "parent" );
        osg::ref_ptr< osg::Group > child = new osg::Group;
        child->setName( "child" );
        parent->addChild( child.get() );
        osg::ref_ptr< osg::MatrixTransform > childMT = new osg::MatrixTransform;
        childMT->setName( "childMT" );
        parent->addChild( childMT.get() );

        std::string input( "1,\"MatrixTransform\",\"childMT\":" );
        osg::Node* expectedResult = childMT.get();
        if( testFindNode( input, parent.get(), expectedResult ) != 0 )
            return( 1 );
    }
    {
        osg::ref_ptr< osg::Group > parent = new osg::Group;
        parent->setName( "parent" );
        osg::ref_ptr< osg::Group > child = new osg::Group;
        child->setName( "child" );
        parent->addChild( child.get() );
        osg::ref_ptr< osg::MatrixTransform > childMT = new osg::MatrixTransform;
        childMT->setName( "childMT" );
        child->addChild( childMT.get() );

        std::string input( "0,\"Group\",\"child\":0,\"MatrixTransform\",\"childMT\":" );
        osg::Node* expectedResult = childMT.get();
        if( testFindNode( input, parent.get(), expectedResult ) != 0 )
            return( 1 );

        // This test should fail. Because this is an expected failure,
        // we return 1 only if the test passes.
        // This test will display warning messages to the console:
        //        NodeData::findNode: _className: MatrixTransform, doesn't match indexChild 0: Group
        //          No match. Returning NULL
        //        findNode from string: test failure.
        //          Expected: 00C29960
        //          Received: 00C27F38
        osg::notify( osg::WARN ) << "----------------------------------------" << std::endl;
        osg::notify( osg::WARN ) << "--- Testing. Warning output is expected:" << std::endl;
        input = std::string( "0,\"MatrixTransform\",\"childMT\":" );
                                                       /* Node: == (instead of !=) because failue is correct behavior */
        if( testFindNode( input, parent.get(), expectedResult ) == 0 )
            return( 1 );
    }

    return( 0 );
}

#endif


// osgwTools
}
