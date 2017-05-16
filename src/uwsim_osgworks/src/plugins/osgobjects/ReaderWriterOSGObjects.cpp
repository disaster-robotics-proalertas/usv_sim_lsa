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

#include "ReaderWriterOSGObjects.h"
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/Output>
#include <osgDB/Input>
#include <osg/io_utils> // Must include this *before* ParameterOutput. Really.
#include <osgDB/ParameterOutput>
#include <osg/Notify>

#include <string>
#include <iostream>
#include <fstream>


ReaderWriterOSGObjects::ReaderWriterOSGObjects()
{
    supportsExtension( "osgarray", "Supports reading and writing unsupported osg::Array Objects." );
}
ReaderWriterOSGObjects::~ReaderWriterOSGObjects()
{
}

const char*
ReaderWriterOSGObjects::className() const
{
    return "External OSG Objects ReaderWriter";
}



osg::Array* Array_readLocalData( osgDB::Input& fr)
{
    int entry = fr[0].getNoNestedBrackets();

    const std::string arrayName = fr[0].getStr();

    unsigned int capacity = 0;
    fr[1].getUInt(capacity);
    ++fr;

    fr += 2;


    osg::Array* return_array = 0;

    if ( arrayName == "ByteArray" )
    {
        osg::ByteArray* array = new osg::ByteArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            int int_value;
            if (fr[0].getInt(int_value))
            {
                ++fr;
                array->push_back(int_value);
            }
            else ++fr;
        }
        ++fr;

        return_array = array;
    }
    else if ( arrayName == "ShortArray" )
    {
        osg::ShortArray* array = new osg::ShortArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            int int_value;
            if (fr[0].getInt(int_value))
            {
                ++fr;
                array->push_back(int_value);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "IntArray" )
    {
        osg::IntArray* array = new osg::IntArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            int int_value;
            if (fr[0].getInt(int_value))
            {
                ++fr;
                array->push_back(int_value);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "UByteArray" )
    {
        osg::UByteArray* array = new osg::UByteArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int uint_value;
            if (fr[0].getUInt(uint_value))
            {
                ++fr;
                array->push_back(uint_value);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "UShortArray" )
    {
        osg::UShortArray* array = new osg::UShortArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int uint_value;
            if (fr[0].getUInt(uint_value))
            {
                ++fr;
                array->push_back(uint_value);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "UIntArray" )
    {
        osg::UIntArray* array = new osg::UIntArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int uint_value;
            if (fr[0].getUInt(uint_value))
            {
                ++fr;
                array->push_back(uint_value);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( ( arrayName == "UVec4bArray" ) || ( arrayName == "Vec4ubArray" ) )
    {
        osg::Vec4ubArray* array = new osg::Vec4ubArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int r,g,b,a;
            if (fr[0].getUInt(r) &&
                fr[1].getUInt(g) &&
                fr[2].getUInt(b) &&
                fr[3].getUInt(a))
            {
                fr+=4;
                array->push_back(osg::Vec4ub(r,g,b,a));
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "FloatArray" )
    {
        osg::FloatArray* array = new osg::FloatArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            float float_value;
            if (fr[0].getFloat(float_value))
            {
                ++fr;
                array->push_back(float_value);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "DoubleArray" )
    {
        osg::DoubleArray* array = new osg::DoubleArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            double double_value;
            if (fr[0].getFloat(double_value))
            {
                ++fr;
                array->push_back(double_value);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec2Array" )
    {
        osg::Vec2Array* array = new osg::Vec2Array;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            osg::Vec2 v;
            if (fr[0].getFloat(v.x()) && fr[1].getFloat(v.y()))
            {
                fr += 2;
                array->push_back(v);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec2dArray" )
    {
        osg::Vec2dArray* array = new osg::Vec2dArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            osg::Vec2d v;
            if (fr[0].getFloat(v.x()) && fr[1].getFloat(v.y()))
            {
                fr += 2;
                array->push_back(v);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec3Array" )
    {
        osg::Vec3Array* array = new osg::Vec3Array;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            osg::Vec3 v;
            if (fr[0].getFloat(v.x()) && fr[1].getFloat(v.y()) && fr[2].getFloat(v.z()))
            {
                fr += 3;
                array->push_back(v);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec3dArray" )
    {
        osg::Vec3dArray* array = new osg::Vec3dArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            osg::Vec3d v;
            if (fr[0].getFloat(v.x()) && fr[1].getFloat(v.y()) && fr[2].getFloat(v.z()))
            {
                fr += 3;
                array->push_back(v);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec4Array" )
    {
        osg::Vec4Array* array = new osg::Vec4Array;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            osg::Vec4 v;
            if (fr[0].getFloat(v.x()) && fr[1].getFloat(v.y()) && fr[2].getFloat(v.z()) && fr[3].getFloat(v.w()))
            {
                fr += 4;
                array->push_back(v);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec4dArray" )
    {
        osg::Vec4dArray* array = new osg::Vec4dArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            osg::Vec4d v;
            if (fr[0].getFloat(v.x()) && fr[1].getFloat(v.y()) && fr[2].getFloat(v.z()) && fr[3].getFloat(v.w()))
            {
                fr += 4;
                array->push_back(v);
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec2bArray" )
    {
        osg::Vec2bArray* array = new osg::Vec2bArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int r,g;
            if (fr[0].getUInt(r) &&
                fr[1].getUInt(g))
            {
                fr+=2;
                array->push_back(osg::Vec2b(r,g));
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec3bArray" )
    {
        osg::Vec3bArray* array = new osg::Vec3bArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int r,g,b;
            if (fr[0].getUInt(r) &&
                fr[1].getUInt(g) &&
                fr[2].getUInt(b))
            {
                fr+=3;
                array->push_back(osg::Vec3b(r,g,b));
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec4bArray" )
    {
        osg::Vec4bArray* array = new osg::Vec4bArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int r,g,b,a;
            if (fr[0].getUInt(r) &&
                fr[1].getUInt(g) &&
                fr[2].getUInt(b) &&
                fr[3].getUInt(a))
            {
                fr+=4;
                array->push_back(osg::Vec4b(r,g,b,a));
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec2sArray" )
    {
        osg::Vec2sArray* array = new osg::Vec2sArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int r,g;
            if (fr[0].getUInt(r) &&
                fr[1].getUInt(g))
            {
                fr+=2;
                array->push_back(osg::Vec2s(r,g));
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec3sArray" )
    {
        osg::Vec3sArray* array = new osg::Vec3sArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int r,g,b;
            if (fr[0].getUInt(r) &&
                fr[1].getUInt(g) &&
                fr[2].getUInt(b))
            {
                fr+=3;
                array->push_back(osg::Vec3s(r,g,b));
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    else if ( arrayName == "Vec4sArray" )
    {
        osg::Vec4sArray* array = new osg::Vec4sArray;
        array->reserve(capacity);
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            unsigned int r,g,b,a;
            if (fr[0].getUInt(r) &&
                fr[1].getUInt(g) &&
                fr[2].getUInt(b) &&
                fr[3].getUInt(a))
            {
                fr+=4;
                array->push_back(osg::Vec4s(r,g,b,a));
            }
            else ++fr;
        }
        ++fr;
        return_array = array;
    }
    
    return return_array;
}


bool Array_writeLocalData( const osg::Array& array, osgDB::Output& fw )
{
    switch(array.getType())
    {
        case( osg::Array::ByteArrayType):
            {
                const osg::ByteArray& carray = static_cast<const osg::ByteArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArrayAsInts(fw,carray.begin(),carray.end());
                return true;
            }
            break;
        case( osg::Array::ShortArrayType):
            {
                const osg::ShortArray& carray = static_cast<const osg::ShortArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end());
                return true;
            }
            break;
        case( osg::Array::IntArrayType):
            {
                const osg::IntArray& carray = static_cast<const osg::IntArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end());
                return true;
            }
            break;
        case( osg::Array::UByteArrayType):
            {
                const osg::UByteArray& carray = static_cast<const osg::UByteArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArrayAsInts(fw,carray.begin(),carray.end());
                return true;
            }
            break;
        case( osg::Array::UShortArrayType):
            {
                const osg::UShortArray& carray = static_cast<const osg::UShortArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end());
                return true;
            }
            break;
        case( osg::Array::UIntArrayType):
            {
                const osg::UIntArray& carray = static_cast<const osg::UIntArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<" ";
                osgDB::writeArray(fw,carray.begin(),carray.end());
                return true;
            }
            break;
        case( osg::Array::Vec4ubArrayType):
            {
                const osg::Vec4ubArray& carray = static_cast<const osg::Vec4ubArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<" ";
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                return true;
            }
            break;
        case( osg::Array::FloatArrayType):
            {
                const osg::FloatArray& carray = static_cast<const osg::FloatArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end());
                return true;
            }
            break;
        case( osg::Array::Vec2ArrayType):
            {
                const osg::Vec2Array& carray = static_cast<const osg::Vec2Array&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                return true;
            }
            break;
        case( osg::Array::Vec3ArrayType):
            {
                const osg::Vec3Array& carray = static_cast<const osg::Vec3Array&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                return true;
            }
            break;
        case( osg::Array::Vec4ArrayType):
            {
                const osg::Vec4Array& carray = static_cast<const osg::Vec4Array&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                return true;
            }
            break;
        case( osg::Array::DoubleArrayType):
            {
                int prec = fw.precision(15);
                const osg::DoubleArray& carray = static_cast<const osg::DoubleArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end());
                fw.precision(prec);
                return true;
            }
            break;
        case( osg::Array::Vec2dArrayType):
            {
                int prec = fw.precision(15);
                const osg::Vec2dArray& carray = static_cast<const osg::Vec2dArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                fw.precision(prec);
                return true;
            }
            break;
        case( osg::Array::Vec3dArrayType):
            {
                int prec = fw.precision(15);
                const osg::Vec3dArray& carray = static_cast<const osg::Vec3dArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                fw.precision(prec);
                return true;
            }
            break;
        case( osg::Array::Vec4dArrayType):
            {
                int prec = fw.precision(15);
                const osg::Vec4dArray& carray = static_cast<const osg::Vec4dArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                fw.precision(prec);
                return true;
            }
            break;
        case( osg::Array::Vec2sArrayType):
            {
                const osg::Vec2sArray& carray = static_cast<const osg::Vec2sArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(), 3);
                return true;
            }
            break;
        case( osg::Array::Vec3sArrayType):
            {
                const osg::Vec3sArray& carray = static_cast<const osg::Vec3sArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(), 2);
                return true;
            }
            break;
        case( osg::Array::Vec4sArrayType):
            {
                const osg::Vec4sArray& carray = static_cast<const osg::Vec4sArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<std::endl;
                osgDB::writeArray(fw,carray.begin(),carray.end(), 1);
                return true;
            }
            break;
        case( osg::Array::Vec2bArrayType):
            {
                const osg::Vec2bArray& carray = static_cast<const osg::Vec2bArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<" ";
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                return true;
            }
            break;
        case( osg::Array::Vec3bArrayType):
            {
                const osg::Vec3bArray& carray = static_cast<const osg::Vec3bArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<" ";
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                return true;
            }
            break;
        case(osg::Array::Vec4bArrayType):
            {
                const osg::Vec4bArray& carray = static_cast<const osg::Vec4bArray&>(array);
                fw<<array.className()<<" "<<carray.size()<<" ";
                osgDB::writeArray(fw,carray.begin(),carray.end(),1);
                return true;
            }
            break;
        case(osg::Array::ArrayType):
        default:
            return false;
    }
}



osgDB::ReaderWriter::ReadResult
ReaderWriterOSGObjects::readObject( const std::string& fileName, const Options* options ) const
{
    OSG_INFO << "ReaderWriterOSGObjects: readObject " << fileName << std::endl;

    const std::string ext = osgDB::getFileExtension( fileName );
    if( !acceptsExtension( ext ) )
        return( ReadResult::FILE_NOT_HANDLED );

    std::ifstream ifstr;( fileName.c_str() );
    ifstr.open( fileName.c_str() );
    osgDB::Input istr;
    istr.attach( &ifstr );

    // Force a read to check for eof.
    istr[0].getNoNestedBrackets();
    if( istr.eof() )
        return( ReadResult::FILE_NOT_FOUND );

    osg::Array* array( Array_readLocalData( istr ) );
    if( array != NULL )
        return( array );
    else
        return( ReadResult::ERROR_IN_READING_FILE );
}

osgDB::ReaderWriter::WriteResult
ReaderWriterOSGObjects::writeObject( const osg::Object& obj, const std::string& fileName, const Options* options ) const
{
    OSG_INFO << "ReaderWriterOSGObjects: writeObject " << fileName << std::endl;

    const std::string ext = osgDB::getFileExtension( fileName );
    if( !acceptsExtension( ext ) )
        return( WriteResult::FILE_NOT_HANDLED );

    osg::Object* nonConstObj( const_cast< osg::Object* >( &obj ) );
    osg::Array* array( dynamic_cast< osg::Array* >( nonConstObj ) );
    osgDB::Output ostr( fileName.c_str() );

    bool result( Array_writeLocalData( *array, ostr ) );

    return( result ? WriteResult::FILE_SAVED : WriteResult::ERROR_IN_WRITING_FILE );
}


REGISTER_OSGPLUGIN( osgobjects, ReaderWriterOSGObjects )
