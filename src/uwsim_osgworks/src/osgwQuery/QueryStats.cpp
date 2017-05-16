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

#include <osgwQuery/QueryStats.h>

#include <osg/Camera>
#include <osg/Geode>
#include <osgText/Text>

#include <sstream>


namespace osgwQuery
{


QueryStats::QueryStats( osg::Node* node )
  : _node( node ),
    _accum( true ),
    _consoleDisplay( false )
{
    clear();
}

osg::Node* QueryStats::getSceneGraph()
{
    if( _cam.valid() )
        return( _cam.get() );

    _cam = new osg::Camera;
    _cam->setName( "__QueryStats" );
    _cam->setRenderOrder( osg::Camera::POST_RENDER );
    _cam->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
    _cam->setViewMatrix( osg::Matrix::identity() );
    _cam->setProjectionMatrix( osg::Matrix::identity() );
    _cam->setClearMask( 0 );

    osg::StateSet* ss = _cam->getOrCreateStateSet();
    ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );
    ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    osg::Geode* geode = new osg::Geode;
    _cam->addChild( geode );

    double textX( .85 );
    double textY( -.6 );
    const double textYDelta( -.04 );
    _labels = new osgText::Text;
    _labels->setFont( "arial.ttf" );
    _labels->setAlignment( osgText::TextBase::RIGHT_TOP );
    _labels->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _labels->setCharacterSize( .04 );
    _labels->setText( _node->getName() + "\nframes: \nqueries: \noccluded: \nNo Q (Rt < Qt): \nNo Q (Cost > Ben): \nQ (prev culled): \nPoccl: " );
    geode->addDrawable( _labels.get() );

    _frames = new osgText::Text;
    _frames->setDataVariance( osg::Object::DYNAMIC );
    _frames->setFont( "arial.ttf" );
    _frames->setAlignment( osgText::TextBase::LEFT_TOP );
    _frames->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _frames->setCharacterSize( .04 );
    _frames->setText( "0" );
    geode->addDrawable( _frames.get() );

    _queries = new osgText::Text;
    _queries->setDataVariance( osg::Object::DYNAMIC );
    _queries->setFont( "arial.ttf" );
    _queries->setAlignment( osgText::TextBase::LEFT_TOP );
    _queries->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _queries->setCharacterSize( .04 );
    _queries->setText( "0" );
    geode->addDrawable( _queries.get() );

    _occluded = new osgText::Text;
    _occluded->setDataVariance( osg::Object::DYNAMIC );
    _occluded->setFont( "arial.ttf" );
    _occluded->setAlignment( osgText::TextBase::LEFT_TOP );
    _occluded->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _occluded->setCharacterSize( .04 );
    _occluded->setText( "0" );
    geode->addDrawable( _occluded.get() );

    _rtLessQt = new osgText::Text;
    _rtLessQt->setDataVariance( osg::Object::DYNAMIC );
    _rtLessQt->setFont( "arial.ttf" );
    _rtLessQt->setAlignment( osgText::TextBase::LEFT_TOP );
    _rtLessQt->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _rtLessQt->setCharacterSize( .04 );
    _rtLessQt->setText( "0" );
    geode->addDrawable( _rtLessQt.get() );

    _cGreaterB = new osgText::Text;
    _cGreaterB->setDataVariance( osg::Object::DYNAMIC );
    _cGreaterB->setFont( "arial.ttf" );
    _cGreaterB->setAlignment( osgText::TextBase::LEFT_TOP );
    _cGreaterB->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _cGreaterB->setCharacterSize( .04 );
    _cGreaterB->setText( "0" );
    geode->addDrawable( _cGreaterB.get() );

    _frustum = new osgText::Text;
    _frustum->setDataVariance( osg::Object::DYNAMIC );
    _frustum->setFont( "arial.ttf" );
    _frustum->setAlignment( osgText::TextBase::LEFT_TOP );
    _frustum->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _frustum->setCharacterSize( .04 );
    _frustum->setText( "0" );
    geode->addDrawable( _frustum.get() );

    _poccl = new osgText::Text;
    _poccl->setDataVariance( osg::Object::DYNAMIC );
    _poccl->setFont( "arial.ttf" );
    _poccl->setAlignment( osgText::TextBase::LEFT_TOP );
    _poccl->setPosition( osg::Vec3( textX, textY, 0. ) );
    textY += textYDelta;
    _poccl->setCharacterSize( .04 );
    _poccl->setText( "0" );
    geode->addDrawable( _poccl.get() );

    return( _cam.get() );
}

bool QueryStats::toggleAccumulate( bool toggle )
{
    if( toggle )
    {
        _accum = !_accum;
        if( _accum )
            clear();
    }
    return( _accum );
}

void QueryStats::clear()
{
    _numFrames = _numQueries = _numOccluded = _numRtLessQt =
        _numCGreaterB = _numFrustum = 0;

    const std::string zeroStr( "0" );
    if( _frames.valid() )
        _frames->setText( zeroStr );
    if( _queries.valid() )
        _queries->setText( zeroStr );
    if( _occluded.valid() )
        _occluded->setText( zeroStr );
    if( _rtLessQt.valid() )
        _rtLessQt->setText( zeroStr );
    if( _cGreaterB.valid() )
        _cGreaterB->setText( zeroStr );
    if( _frustum.valid() )
        _frustum->setText( zeroStr );
    if( _poccl.valid() )
        _poccl->setText( zeroStr );
}
unsigned int QueryStats::incFrames( unsigned int n )
{
    if( _consoleDisplay )
    {
        osg::notify( osg::ALWAYS ) << "frames: " << _numFrames << std::endl <<
            "  queries: " << _numQueries << std::endl <<
            "  occluded: " << _numOccluded << std::endl <<
            "  No Q (Rt < Qt): " << _numRtLessQt << std::endl <<
            "  No Q (Cost > Ben): " << _numCGreaterB << std::endl <<
            "  Q (prev culled): " << _numFrustum << std::endl;
        if( _poccl.valid() )
            osg::notify( osg::ALWAYS ) << "  Poccl: " << 
                _poccl->getText().createUTF8EncodedString() << std::endl;
    }

    return( internalInc( _numFrames, _frames.get(), n ) );
}
unsigned int QueryStats::incQueries( unsigned int n )
{
    return( internalInc( _numQueries, _queries.get(), n ) );
}
unsigned int QueryStats::incOccluded( unsigned int n )
{
    return( internalInc( _numOccluded, _occluded.get(), n ) );
}
unsigned int QueryStats::incRtLessQt( unsigned int n )
{
    return( internalInc( _numRtLessQt, _rtLessQt.get(), n ) );
}
unsigned int QueryStats::incCGreaterB( unsigned int n )
{
    return( internalInc( _numCGreaterB, _cGreaterB.get(), n ) );
}
unsigned int QueryStats::incFrustum( unsigned int n )
{
    return( internalInc( _numFrustum, _frustum.get(), n ) );
}
unsigned int QueryStats::internalInc( unsigned int& val, osgText::Text* text, unsigned int n )
{
    if( !_accum )
        return( val );

    val += n;

    if( text != NULL )
    {
        std::ostringstream ostr;
        ostr << val;
        text->setText( ostr.str() );
    }

    return( val );
}

void QueryStats::setPoccl( const float poccl )
{
    if( !_accum )
        return;

    if( _poccl.valid() )
    {
        std::ostringstream ostr;
        ostr << poccl;
        _poccl->setText( ostr.str() );
    }
}



QueryStatsHandler::QueryStatsHandler( osgwQuery::QueryStats* qs )
  : _qs( qs )
{
    if( !_qs.valid() )
        osg::notify( osg::WARN ) << "QueryStatsHandler: ctor: QueryStats NULL." << std::endl;
}

bool QueryStatsHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    if( ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN )
    {
        if( ea.getKey() == 'a' )
        {
            _qs->toggleAccumulate();
            return( true );
        }
    }
    return( false );
}


// osgwQuery
}
