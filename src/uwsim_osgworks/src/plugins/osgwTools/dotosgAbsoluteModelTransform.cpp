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

#include "osgwTools/AbsoluteModelTransform.h"

#include <iostream>

#include <osg/Matrix>
#include <osg/io_utils>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgDB/ParameterOutput>


/** \addtogroup Plugins
@{*/

/** \addtogroup DotOSGSupport
@{*/


bool AMT_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool AMT_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy AMT_Proxy
(
    new osgwTools::AbsoluteModelTransform,
    "AbsoluteModelTransform",
    "Object Node Transform AbsoluteModelTransform Group",
    AMT_readLocalData,
    AMT_writeLocalData
);


bool readMatrix( osg::Matrix& matrix, osgDB::Input& fr, const char* keyword="Matrix" )
{
    bool iteratorAdvanced = false;
    
    if (fr[0].matchWord(keyword) && fr[1].isOpenBracket())
    {
        int entry = fr[0].getNoNestedBrackets();

        fr += 2;

        int row=0;
        int col=0;
        double v;
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            if (fr[0].getFloat(v))
            {
                matrix(row,col)=v;
                ++col;
                if (col>=4)
                {
                    col = 0;
                    ++row;
                }
                ++fr;
            }
            else fr.advanceOverCurrentFieldOrBlock();
        }
        ++fr; // skip closing brace
        iteratorAdvanced = true;
    }        
        
    return iteratorAdvanced;
}

bool writeMatrix( const osg::Matrix& matrix, osgDB::Output& fw, const char* keyword="Matrix" )
{
    fw.indent() << keyword <<" {" << std::endl;
    fw.moveIn();
    fw.indent() << matrix(0,0) << " " << matrix(0,1) << " " << matrix(0,2) << " " << matrix(0,3) << std::endl;
    fw.indent() << matrix(1,0) << " " << matrix(1,1) << " " << matrix(1,2) << " " << matrix(1,3) << std::endl;
    fw.indent() << matrix(2,0) << " " << matrix(2,1) << " " << matrix(2,2) << " " << matrix(2,3) << std::endl;
    fw.indent() << matrix(3,0) << " " << matrix(3,1) << " " << matrix(3,2) << " " << matrix(3,3) << std::endl;
    fw.moveOut();
    fw.indent() << "}"<< std::endl;
    return true;
}



bool AMT_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgwTools::AbsoluteModelTransform& amt = static_cast< osgwTools::AbsoluteModelTransform& >( obj );
    bool advance( false );

    osg::Matrix m;
    if( readMatrix( m, fr ))
    {
        amt.setMatrix( m );
        advance = true;
    }
    
    return( advance );
}

bool AMT_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgwTools::AbsoluteModelTransform& amt = static_cast< const osgwTools::AbsoluteModelTransform& >( obj );

    writeMatrix( amt.getMatrix(), fw );

    return( true );
}

/*@}*/

/*@}*/
