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

#include <osgwTools/CountsVisitor.h>
#include <osgwTools/StateSetUtils.h>
#include <osgwTools/Version.h>
#include <osg/LOD>
#include <osg/PagedLOD>
#include <osg/Switch>
#include <osg/Sequence>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Texture>
#include <osg/Program>
#include <osg/Uniform>
#include <osg/Geometry>
#include <osgSim/DOFTransform>
#include <osgText/Text>
#include <iostream>
#include <map>
#include <numeric>


namespace osgwTools
{


typedef std::map< int, int > ModeCounter;
static ModeCounter mc;


CountsVisitor::CountsVisitor( osg::NodeVisitor::TraversalMode mode )
  : osgwTools::StateTrackingNodeVisitor( mode ),
    _countUserMode( false ),
    _countUserAttr( false )
{
    reset();
}

CountsVisitor::~CountsVisitor()
{
}

unsigned int CountsVisitor::getVertices() const
{
    return( _vertices );
}
unsigned int CountsVisitor::getDrawArrays() const
{
    return( _drawArrays );
}
unsigned int CountsVisitor::getTotalDrawables() const
{
    return( _totalDrawables );
}
unsigned int CountsVisitor::getNumDrawablesUserModeOff() const
{
    return( _drawUserModeOff );
}

float CountsVisitor::getChildrenPerNode()
{
    const unsigned int allNodes( _nodes+_groups+_lods+_pagedLods+_switches+_sequences+_transforms+_matrixTransforms+_dofTransforms );
    if( allNodes > 0 )
        return( (float)_totalChildren / (float)allNodes );
    else
        return( 0.f );
}
float CountsVisitor::getChildrenPerGroup()
{
    const unsigned int allGroups( _groups + _lods + _pagedLods + _switches + _sequences + _transforms + _matrixTransforms + _dofTransforms );
    if( allGroups > 0 )
        return( (float)_totalChildren / (float)allGroups );
    else
        return( 0.f );
}
float CountsVisitor::getDrawablesPerGeode()
{
    if( _geodes > 0 )
        return( (float)_totalDrawables / (float)_geodes );
    else
        return( 0.f );
}
float CountsVisitor::getDrawablesPerNode()
{
    const unsigned int allNodes( _nodes+_groups+_lods+_pagedLods+_switches+_sequences+_transforms+_matrixTransforms+_dofTransforms );
    if( allNodes > 0 )
        return( (float)_totalDrawables / (float)allNodes );
    else
        return( 0.f );
}
float CountsVisitor::getPrimitiveSetsPerGeometry()
{
    if( _geometries > 0 )
        return( (float) _primitiveSets / (float) _geometries );
    else
        return( 0.f );
}
float CountsVisitor::getVerticesPerGeometry()
{
    if( _geometries > 0 )
        return( (float) _vertices / (float) _geometries );
    else
        return( 0.f );
}

void CountsVisitor::setUserMode( GLenum userMode )
{
    _userMode = userMode;
    _countUserMode = true;
}
void CountsVisitor::setUserAttribute( osg::StateAttribute::Type userAttr )
{
    _userAttr = userAttr;
    _countUserAttr = true;
}


void
CountsVisitor::reset()
{
    _depth = 0;
    _maxDepth = 0;

    _nodes = 0;
    _groups = 0;
    _lods = 0;
    _pagedLods = 0;
    _switches = 0;
    _sequences = 0;
    _transforms = 0;
    _matrixTransforms = 0;
    _dofTransforms = 0;
    _geodes = 0;
    _totalDrawables = 0;
    _drawables = 0;
    _geometries = 0;
    _nullGeometries = 0;
    _texts = 0;
    _vertices = 0;
    _stateSets = 0;
    _emptyStateSets = 0;
    _uniforms = 0;
    _programs = 0;
    _attributes = 0;
    _modes = 0;
    _texAttributes = 0;
    _texModes = 0;
    _textures = 0;
    _primitiveSets = 0;
    _drawArrays = 0;

    _totalUserModes = 0;
    _totalUserAttrs = 0;
    _drawUserModeOn = 0;
    _drawUserModeOff = 0;
    _drawUserModeNotSet = 0;

    _totalChildren = 0;
#if( OSGWORKS_OSG_VERSION < 30108 )
    _slowPathGeometries = 0;
#endif

    _uNodes.clear();
    _uGroups.clear();
    _uLods.clear();
    _uPagedLods.clear();
    _uSwitches.clear();
    _uSequences.clear();
    _uTransforms.clear();
    _uMatrixTransforms.clear();
    _uDofTransforms.clear();
    _uGeodes.clear();
    _uDrawables.clear();
    _uGeometries.clear();
    _uTexts.clear();
    _uVertices.clear();
    _uStateSets.clear();
    _uUniforms.clear();
    _uPrograms.clear();
    _uAttributes.clear();
    _uTexAttributes.clear();
    _uTextures.clear();
    _uPrimitiveSets.clear();
    _uDrawArrays.clear();

    _childrenPerGroup.clear();
    _drawablesPerGeode.clear();
    _primSetsPerGeom.clear();
    _vertsPerGeom.clear();

    _maxChildren.clear();
    _minChildrenCount = 0xffffffff;
    _maxChildrenCount = 0;

    _maxDrawable.clear();
    _minDrawableCount = 0xffffffff;
    _maxDrawableCount = 0;

    _maxPrimSet.clear();
    _maxPrimSetGeom = NULL;
    _minPrimSetCount = 0xffffffff;
    _maxPrimSetCount = 0;

    _minVertices.clear();
    _minVerticesGeom = NULL;
    _minVerticesCount = 0xffffffff;
    _maxVerticesCount = 0;
}

void
CountsVisitor::dump( std::ostream& ostr )
{
    ostr << std::endl;
    ostr << "           OSG Object \tCount\tUnique" << std::endl;
    ostr << "           ---------- \t-----\t------" << std::endl;
    ostr << "               Groups \t" << _groups << "\t" << _uGroups.size() << std::endl;
    ostr << "                 LODs \t" << _lods << "\t" << _uLods.size() << std::endl;
    ostr << "            PagedLODs \t" << _pagedLods << "\t" << _uPagedLods.size() << std::endl;
    ostr << "             Switches \t" << _switches << "\t" << _uSwitches.size() << std::endl;
    ostr << "            Sequences \t" << _sequences << "\t" << _uSequences.size() << std::endl;
    ostr << "     MatrixTransforms \t" << _matrixTransforms << "\t" << _uMatrixTransforms.size() << std::endl;
    ostr << "        DOFTransforms \t" << _dofTransforms << "\t" << _uDofTransforms.size() << std::endl;
    ostr << "     Other Transforms \t" << _transforms << "\t" << _uTransforms.size() << std::endl;
    ostr << "               Geodes \t" << _geodes << "\t" << _uGeodes.size() << std::endl;
    ostr << "          Other Nodes \t" << _nodes << "\t" << _uNodes.size() << std::endl;
    ostr << "      Empty StateSets \t" << _emptyStateSets << std::endl;
    ostr << "      Total StateSets \t" << _stateSets << "\t" << _uStateSets.size() << std::endl;
    ostr << "             Programs \t" << _programs << "\t" << _uPrograms.size() << std::endl;
    ostr << "             Uniforms \t" << _uniforms << "\t" << _uUniforms.size() << std::endl;
    if( _countUserMode )
        ostr << "           User Modes \t" << _totalUserModes << std::endl;
    if( _countUserAttr )
        ostr << "      User Attributes \t" << _totalUserAttrs << std::endl;
    ostr << "     Total Attributes \t" << _attributes << "\t" << _uAttributes.size() << std::endl;
    ostr << "          Total Modes \t" << _modes << std::endl;
    ostr << "             Textures \t" << _textures << "\t" << _uTextures.size() << std::endl;
    ostr << "  Total TexAttributes \t" << _texAttributes << "\t" << _uTexAttributes.size() << std::endl;
    ostr << "       Total TexModes \t" << _texModes << std::endl;
    ostr << "      NULL Geometries \t" << _nullGeometries << std::endl;
    ostr << "     Total Geometries \t" << _geometries << "\t" << _uGeometries.size() << std::endl;
    ostr << "                Texts \t" << _texts << "\t" << _uTexts.size() << std::endl;
    ostr << "      Other Drawables \t" << _drawables << "\t" << _uDrawables.size() << std::endl;
    ostr << "      Totol Drawables \t" << _totalDrawables << std::endl;
    ostr << "   Drawables per Node \t" << getDrawablesPerNode() << std::endl;
    ostr << "           DrawArrays \t" << _drawArrays << "\t" << _uDrawArrays.size() << std::endl;
    ostr << "  Total PrimitiveSets \t" << _primitiveSets << "\t" << _uPrimitiveSets.size() << std::endl;
    if( _countUserMode )
    {
        ostr << "Drawables with user Modes:" << std::endl;
        ostr << "              Enabled \t" << _drawUserModeOn << std::endl;
        ostr << "             Disabled \t" << _drawUserModeOff << std::endl;
        ostr << "              Not set \t" << _drawUserModeNotSet << std::endl;
    }

#if( OSGWORKS_OSG_VERSION < 30108 )
    if (_slowPathGeometries)
        ostr << "      Slow Path Geoms \t" << _slowPathGeometries << std::endl;
#endif

    ostr << "       Total Vertices \t" << _vertices << std::endl;
    ostr << "            Max Depth \t" << _maxDepth << std::endl;

    double mean, median, stdev;
    stats( mean, median, stdev, _childrenPerGroup );
    ostr << std::endl;
    ostr << "Children per Group" << std::endl;
    ostr << "  Mean: " << mean <<
        ",   Median: " << median <<
        ",   Std dev: " << stdev << std::endl;
    ostr << "  Min: " << _minChildrenCount <<
      ",   Max: " << _maxChildrenCount << std::endl;
    ostr << "  Group with max children: "; dumpNodePath( ostr, _maxChildren );

    stats( mean, median, stdev, _drawablesPerGeode );
    ostr << std::endl;
    ostr << "Drawables per Geode" << std::endl;
    ostr << "  Mean: " << mean <<
        ",   Median: " << median <<
        ",   Std dev: " << stdev << std::endl;
    ostr << "  Min: " << _minDrawableCount <<
        ",   Max: " << _maxDrawableCount << std::endl;
    ostr << "  Geode with max drawables: "; dumpNodePath( ostr, _maxDrawable );

    stats( mean, median, stdev, _primSetsPerGeom );
    ostr << std::endl;
    ostr << "PrimitivesSets per Geometry" << std::endl;
    ostr << "  Mean: " << mean <<
        ",   Median: " << median <<
        ",   Std dev: " << stdev << std::endl;
    ostr << "  Min: " << _minPrimSetCount <<
        ",   Max: " << _maxPrimSetCount << std::endl;
    ostr << "  Geometry with max PrimitiveSets: "; dumpNodePath( ostr, _maxPrimSet );
    ostr << "    Geometry name: \"" << _maxPrimSetGeom->getName() << "\"" << std::endl;

    stats( mean, median, stdev, _vertsPerGeom );
    ostr << std::endl;
    ostr << "Vertices per Geometry" << std::endl;
    ostr << "  Mean: " << mean <<
        ",   Median: " << median <<
        ",   Std dev: " << stdev << std::endl;
    ostr << "  Min: " << _minVerticesCount <<
        ",   Max: " << _maxVerticesCount << std::endl;
    ostr << "  Geometry with min Vertices: "; dumpNodePath( ostr, _minVertices );
    ostr << "    Geometry name: \"" << _minVerticesGeom->getName() << "\"" << std::endl;
}

void CountsVisitor::apply( const osg::Geode& node, osg::Drawable* draw )
{
    apply( draw->getStateSet() );

    pushStateSet( draw->getStateSet() );

    if( _countUserMode )
    {
        if( isSet( _userMode, _stateStack.back().get() ) )
        {
            if( isEnabled( _userMode, _stateStack.back().get() ) )
                _drawUserModeOn++;
            else
                _drawUserModeOff++;
        }
        else
            _drawUserModeNotSet++;
    }

    _totalDrawables++;
    osg::Geometry* geom;
    if (dynamic_cast<osgText::Text*>( draw ) != NULL)
    {
        _texts++;
        osg::ref_ptr<osg::Object> rp = (osg::Object*)draw;
        _uTexts.insert( rp );
    }
    else if ( (geom = dynamic_cast<osg::Geometry*>( draw )) != NULL)
    {
        _geometries++;
        osg::ref_ptr<osg::Object> rp = (osg::Object*)geom;
        _uGeometries.insert( rp );

#if( OSGWORKS_OSG_VERSION < 30108 )
        // OSG removed slow paths in 3.1.8 tag.
        if (!geom->areFastPathsUsed())
            _slowPathGeometries++;
#endif

        numPrimSetCheck( node, geom );
        unsigned int localVertices( 0 );
        if( geom->getNumPrimitiveSets() > 0 )
        {
            unsigned int idx;
            for( idx=0; idx < geom->getNumPrimitiveSets(); idx++ )
            {
                osg::PrimitiveSet* ps = geom->getPrimitiveSet( idx );
                localVertices += ps->getNumIndices();
            }
            _vertices += localVertices;
        }
        else
            _nullGeometries++;
        numVerticesCheck( node, geom, localVertices );
        osg::ref_ptr<osg::Object> rpv = (osg::Object*)( geom->getVertexArray() );
        _uVertices.insert( rpv );

        if( geom->getNumPrimitiveSets() > 0 )
        {
            _primitiveSets += geom->getNumPrimitiveSets();
            osg::Geometry::PrimitiveSetList& psl = geom->getPrimitiveSetList();
            osg::Geometry::PrimitiveSetList::const_iterator pslit;
            for( pslit = psl.begin(); pslit != psl.end(); pslit++ )
            {
                osg::ref_ptr<osg::Object> rpps = (osg::Object*)( pslit->get() );
                _uPrimitiveSets.insert( rpps );
                const osg::DrawArrays* da = dynamic_cast< const osg::DrawArrays* >( pslit->get() );
                if( da )
                {
                    _drawArrays++;
                    osg::ref_ptr<osg::Object> rpda = (osg::Object*)( da );
                    _uDrawArrays.insert( rpda );
                }
            }
        }
    }
    else
    {
        _drawables++;
        osg::ref_ptr<osg::Object> rp = (osg::Object*)draw;
        _uDrawables.insert( rp );
    }

    popStateSet();
}
void CountsVisitor::apply( osg::StateSet* stateSet )
{
    if( stateSet == NULL )
        return;
    _stateSets++;
    osg::ref_ptr<osg::Object> ssrp = (osg::Object*)stateSet;
    _uStateSets.insert( ssrp );

    if( osgwTools::isEmpty( *stateSet ) )
        _emptyStateSets++;

    const osg::StateSet::TextureAttributeList& tal = stateSet->getTextureAttributeList();
    const osg::StateSet::TextureModeList tml = stateSet->getTextureModeList();
    unsigned int idx;
    for( idx=0; idx<tal.size(); idx++ )
    {
        if( tal.size() > 0 )
        {
            const osg::StateSet::AttributeList& al = tal[ idx ];
            _texAttributes += al.size();
            osg::StateSet::AttributeList::const_iterator ait;
            for( ait=al.begin(); ait!=al.end(); ait++ )
            {
                osg::ref_ptr<osg::Object> arp = (osg::Object*)( ait->second.first.get() );
                _uTexAttributes.insert( arp );
            }
        }

        if( tml.size() > 0 )
            _texModes += tml[ idx ].size();

        osg::Texture* texture = static_cast< osg::Texture* >(
            stateSet->getTextureAttribute( idx, osg::StateAttribute::TEXTURE ) );
        if( texture != NULL )
        {
            _textures++;
            osg::ref_ptr<osg::Object> trp = (osg::Object*)texture;
            _uTextures.insert( trp );
        }
    }

    const osg::StateSet::AttributeList& al = stateSet->getAttributeList();
    _attributes += al.size();
    osg::StateSet::AttributeList::const_iterator ait;
    for( ait=al.begin(); ait!=al.end(); ait++ )
    {
        osg::StateAttribute* sa = ait->second.first.get();
        if( _countUserAttr && ( sa->getType() == _userAttr ) )
            _totalUserAttrs++;

        osg::ref_ptr<osg::Object> arp = (osg::Object*)( sa );
        _uAttributes.insert( arp );
    }

    _modes += stateSet->getModeList().size();
    {
        const osg::StateSet::ModeList& ml = stateSet->getModeList();
        osg::StateSet::ModeList::const_iterator it;
        for( it=ml.begin(); it != ml.end(); it++ )
        {
            if( _countUserMode && ( it->first == _userMode ) )
                _totalUserModes++;

            mc[ it->first ] += 1;
        }
    }

    osg::Program* program = static_cast< osg::Program* >(
        stateSet->getAttribute( osg::StateAttribute::PROGRAM ) );
    if( program != NULL )
    {
        _programs++;
        osg::ref_ptr<osg::Object> prp = (osg::Object*)program;
        _uPrograms.insert( prp );
    }

    const osg::StateSet::UniformList ul = stateSet->getUniformList();
    _uniforms += ul.size();
    osg::StateSet::UniformList::const_iterator it;
    for( it=ul.begin(); it!=ul.end(); it++ )
    {
        osg::ref_ptr<osg::Object> urp = (osg::Object*)( it->second.first.get() );
        _uUniforms.insert( urp );
    }
}

void
CountsVisitor::apply( osg::Node& node )
{
    pushStateSet( node.getStateSet() );

    _nodes++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uNodes.insert( rp );
    apply( node.getStateSet() );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( node );
    _depth--;

    popStateSet();
}

void CountsVisitor::numChildrenCheck( const osg::Group& node )
{
    const unsigned int nc( node.getNumChildren() );
    _childrenPerGroup.push_back( (double)nc );
    if( nc > _maxChildrenCount )
    {
        _maxChildrenCount = nc;
        _maxChildren = getNodePath();
    }
    if( nc < _minChildrenCount )
        _minChildrenCount = nc;
}
void CountsVisitor::numDrawableCheck( const osg::Geode& node )
{
    const unsigned int nd( node.getNumDrawables() );
    _drawablesPerGeode.push_back( (double)nd );
    if( nd > _maxDrawableCount )
    {
        _maxDrawableCount = nd;
        _maxDrawable = getNodePath();
    }
    if( nd < _minDrawableCount )
        _minDrawableCount = nd;
}
void CountsVisitor::numPrimSetCheck( const osg::Geode& node, osg::Geometry* geom )
{
    const unsigned int nps( geom->getNumPrimitiveSets() );
    _primSetsPerGeom.push_back( nps );
    if( nps > _maxPrimSetCount )
    {
        _maxPrimSetCount = nps;
        _maxPrimSet = getNodePath();
        _maxPrimSetGeom = geom;
    }
    if( nps < _minPrimSetCount )
        _minPrimSetCount = nps;
}
void CountsVisitor::numVerticesCheck( const osg::Geode& node, osg::Geometry* geom, const unsigned int numVerts )
{
    _vertsPerGeom.push_back( (double)numVerts );
    if( numVerts < _minVerticesCount )
    {
        _minVerticesCount = numVerts;
        _minVertices = getNodePath();
        _minVerticesGeom = geom;
    }
    if( numVerts > _maxVerticesCount )
        _maxVerticesCount = numVerts;
}

void
CountsVisitor::apply( osg::Group& node )
{
    pushStateSet( node.getStateSet() );

    _groups++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uGroups.insert( rp );
    _totalChildren += node.getNumChildren();
    numChildrenCheck( node );
    apply( node.getStateSet() );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}

void
CountsVisitor::apply( osg::LOD& node )
{
    pushStateSet( node.getStateSet() );

    _lods++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uLods.insert( rp );
    _totalChildren += node.getNumChildren();
    apply( node.getStateSet() );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}

void
CountsVisitor::apply( osg::PagedLOD& node )
{
    pushStateSet( node.getStateSet() );

    osg::Group* grp = node.getParent(0);
    osg::Group* gPar = NULL;
    if (grp)
        gPar = grp->getParent(0);
    apply( node.getStateSet() );

    _pagedLods++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uPagedLods.insert( rp );
    _totalChildren += node.getNumChildren();
    numChildrenCheck( node );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}

void
CountsVisitor::apply( osg::Switch& node )
{
    pushStateSet( node.getStateSet() );

    _switches++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uSwitches.insert( rp );
    _totalChildren += node.getNumChildren();
    numChildrenCheck( node );
    apply( node.getStateSet() );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}

void
CountsVisitor::apply( osg::Sequence& node )
{
    pushStateSet( node.getStateSet() );

    _sequences++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uSequences.insert( rp );
    _totalChildren += node.getNumChildren();
    numChildrenCheck( node );
    apply( node.getStateSet() );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}

void
CountsVisitor::apply( osg::Transform& node )
{
    pushStateSet( node.getStateSet() );

    if (dynamic_cast<osgSim::DOFTransform*>( &node ) != NULL)
    {
        _dofTransforms++;
        osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
        _uDofTransforms.insert( rp );
    }
    else
    {
        _transforms++;
        osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
        _uTransforms.insert( rp );
    }
    _totalChildren += node.getNumChildren();
    numChildrenCheck( node );
    apply( node.getStateSet() );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}

void
CountsVisitor::apply( osg::MatrixTransform& node )
{
    pushStateSet( node.getStateSet() );

    _matrixTransforms++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uMatrixTransforms.insert( rp );
    _totalChildren += node.getNumChildren();
    numChildrenCheck( node );
    apply( node.getStateSet() );

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}

void
CountsVisitor::apply( osg::Geode& node )
{
    pushStateSet( node.getStateSet() );

    _geodes++;
    osg::ref_ptr<osg::Object> rp = (osg::Object*)&node;
    _uGeodes.insert( rp );
    numDrawableCheck( node );
    apply( node.getStateSet() );

    unsigned int idx;
    for (idx=0; idx<node.getNumDrawables(); idx++)
    {
        osg::Drawable* draw = node.getDrawable( idx );
        apply( node, draw );
    }

    if (++_depth > _maxDepth)
        _maxDepth = _depth;
    traverse( (osg::Node&)node );
    _depth--;

    popStateSet();
}


void CountsVisitor::dumpNodePath( std::ostream& ostr, const osg::NodePath& np )
{
    for( unsigned int idx=0; idx < np.size(); ++idx )
    {
        ostr << "\"" << np[ idx ]->getName() << "\"";
        if( idx < np.size()-1 )
            ostr << ", ";
    }
    ostr << std::endl;
}

bool CountsVisitor::isSet( GLenum stateItem, osg::StateSet* ss )
{
    if( ss == NULL )
        return( false );

    // StateSet is not NULL. Query the mode.
    osg::StateAttribute::GLModeValue mode;
    mode = ss->getMode( stateItem );

    // The item is set if the mode is anything other than INHERIT.
    return( mode != osg::StateAttribute::INHERIT );
}
bool CountsVisitor::isEnabled( GLenum stateItem, osg::StateSet* ss )
{
    if( ss == NULL )
        // Calling code must use isSet() to verify a mode is set, then
        // call isEnabled() to see if the mode is enabled or not. We return
        // false here, but calling code should never call us with a NULL
        // StateSet, as isSet() would have returned false.
        return( false );

    // StateSet is not NULL. Query the mode.
    osg::StateAttribute::GLModeValue mode;
    mode = ss->getMode( stateItem );

    if( mode & osg::StateAttribute::ON )
        // Item is enabled if its value is ON.
        return( true );
    else
        // If it's not enabled, then it's off or not set.
        return( false );
}

void CountsVisitor::stats( double& mean, double& median, double& stdev, DoubleVec& v )
{
    if( v.size() == 0 )
    {
        mean = median = stdev = 0.;
        return;
    }

    double sum = std::accumulate( v.begin(), v.end(), 0.0 );
    mean = sum / v.size();

    const unsigned int idx( ((v.size() & 0x1) == 1) ? v.size() / 2 + 1 : v.size() / 2 );
    median = v[ idx ];

    double sqSum = std::inner_product( v.begin(), v.end(), v.begin(), 0.0 );
    stdev = sqrt( sqSum / v.size() - mean * mean );
}


// osgwTools
}
