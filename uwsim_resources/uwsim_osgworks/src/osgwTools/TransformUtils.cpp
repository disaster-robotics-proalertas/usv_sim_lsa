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

#include "osgwTools/TransformUtils.h"

#include <osg/Matrix>
#include <osg/Transform>
#include <osg/Camera>


namespace osgwTools {

class TransformVisitor : public osg::NodeVisitor
{
public:
    
    enum CoordMode
    {
        WORLD_TO_LOCAL,
        LOCAL_TO_WORLD
    };
    
    
    CoordMode       _coordMode;
    osg::Matrix&         _matrix;
    bool            _ignoreCameras;
    
    TransformVisitor(osg::Matrix& matrix,CoordMode coordMode, bool ignoreCameras):
    osg::NodeVisitor(),
    _coordMode(coordMode),
    _matrix(matrix),
    _ignoreCameras(ignoreCameras)
    {}
    
    virtual void apply(osg::Transform& transform)
    {
        if (_coordMode==LOCAL_TO_WORLD)
        {
            transform.computeLocalToWorldMatrix(_matrix,this);
        }
        else // worldToLocal
        {
            transform.computeWorldToLocalMatrix(_matrix,this);
        }
    }
    
    void accumulate(const osg::NodePath& nodePath)
    {
        if (nodePath.empty()) return;
        
        unsigned int i = 0;
        if (_ignoreCameras)
        {
            // we need to found out the last absolute Camera in NodePath and
            // set the i index to after it so the final accumulation set ignores it.
            i = nodePath.size();
            osg::NodePath::const_reverse_iterator ritr;
            for(ritr = nodePath.rbegin();
                ritr != nodePath.rend();
                ++ritr, --i)
            {
                const osg::Camera* camera = dynamic_cast<const osg::Camera*>(*ritr);
                if (camera && 
                    (camera->getReferenceFrame()!=osg::Transform::RELATIVE_RF || camera->getParents().empty()))
                {
                    break;
                }
            }
        }            
        
        // do the accumulation of the active part of nodepath.        
        for(;
            i<nodePath.size();
            ++i)
        {
            const_cast<osg::Node*>(nodePath[i])->accept(*this);
        }
    }
    
protected:
    
    TransformVisitor& operator = (const TransformVisitor&) { return *this; }
    
};
    
OSGWTOOLS_EXPORT osg::Matrix computeLocalToWorldWithNodeMask(const osg::NodePath& nodePath, const unsigned int mask, bool ignoreCameras)
{
    osg::Matrix matrix;
    osgwTools::TransformVisitor tv( matrix, osgwTools::TransformVisitor::LOCAL_TO_WORLD, ignoreCameras );
    tv.setNodeMaskOverride( mask ); // set override
    tv.accumulate( nodePath );
    return matrix;
}
    

// osgwTools
}
