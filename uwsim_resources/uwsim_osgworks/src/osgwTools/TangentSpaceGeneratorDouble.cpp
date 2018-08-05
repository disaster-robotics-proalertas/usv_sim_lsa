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

// This code was taken from OSG v2.8.5 and modified to use double precision.

#include <osgwTools/TangentSpaceGeneratorDouble.h>

#include <osgwTools/Version.h>
#include <osg/Notify>
#include <osg/io_utils>


namespace osgwTools {


TangentSpaceGeneratorDouble::TangentSpaceGeneratorDouble()
:    osg::Referenced(),
    T_(new osg::Vec4Array),
    B_(new osg::Vec4Array),
    N_(new osg::Vec4Array)
{
}

TangentSpaceGeneratorDouble::TangentSpaceGeneratorDouble(const TangentSpaceGeneratorDouble &copy, const osg::CopyOp &copyop)
:    osg::Referenced(copy),
    T_(static_cast<osg::Vec4Array *>(copyop(copy.T_.get()))),
    B_(static_cast<osg::Vec4Array *>(copyop(copy.B_.get()))),
    N_(static_cast<osg::Vec4Array *>(copyop(copy.N_.get())))
{
}

void TangentSpaceGeneratorDouble::generate(osg::Geometry *geo, int normal_map_tex_unit)
{

#if( OSGWORKS_OSG_VERSION < 30108 )    
    // check to see if vertex attributes indices exists, if so expand them to remove them
    // This was removed from OSG 3.1.8 upwards    
    if (geo->suitableForOptimization())
    {
        // removing coord indices so we don't have to deal with them in the binormal code.
        osg::notify(osg::INFO)<<"TangentSpaceGeneratorDouble::generate(Geometry*,int): Removing attribute indices"<<std::endl;
        geo->copyToAndOptimize(*geo);
    }
#endif

    const osg::Array *vx = geo->getVertexArray();
    const osg::Array *nx = geo->getNormalArray();
    const osg::Array *tx = geo->getTexCoordArray(normal_map_tex_unit);

    if (!vx || !tx) return;


    unsigned int vertex_count = vx->getNumElements();
    if (geo->getVertexIndices() == NULL) {
        T_->assign(vertex_count, osg::Vec4());
        B_->assign(vertex_count, osg::Vec4());
        N_->assign(vertex_count, osg::Vec4());
    } else {
        unsigned int index_count = geo->getVertexIndices()->getNumElements();
        T_->assign(index_count, osg::Vec4());
        B_->assign(index_count, osg::Vec4());
        N_->assign(index_count, osg::Vec4());
        indices_ = new osg::UIntArray();
        unsigned int i;
        for (i=0;i<index_count;i++) {
            indices_->push_back(i);
        }
    }

    unsigned int i; // VC6 doesn't like for-scoped variables

    for (unsigned int pri=0; pri<geo->getNumPrimitiveSets(); ++pri) {
        osg::PrimitiveSet *pset = geo->getPrimitiveSet(pri);

        unsigned int N = pset->getNumIndices();

        switch (pset->getMode()) {

            case osg::PrimitiveSet::TRIANGLES:
                for (i=0; i<N; i+=3) {
                    compute(pset, vx, nx, tx, i, i+1, i+2);
                }
                break;

            case osg::PrimitiveSet::QUADS:
                for (i=0; i<N; i+=4) {
                    compute(pset, vx, nx, tx, i, i+1, i+2);
                    compute(pset, vx, nx, tx, i+2, i+3, i);
                }
                break;

            case osg::PrimitiveSet::TRIANGLE_STRIP:
                if (pset->getType() == osg::PrimitiveSet::DrawArrayLengthsPrimitiveType) {
                    osg::DrawArrayLengths *dal = static_cast<osg::DrawArrayLengths *>(pset);
                    unsigned int j = 0;
                    for (osg::DrawArrayLengths::const_iterator pi=dal->begin(); pi!=dal->end(); ++pi) {
                        unsigned int iN = static_cast<unsigned int>(*pi-2);
                        for (i=0; i<iN; ++i, ++j) {
                            if ((i%2) == 0) {
                                compute(pset, vx, nx, tx, j, j+1, j+2);
                            } else {
                                compute(pset, vx, nx, tx, j+1, j, j+2);
                            }
                        }
                        j += 2;
                    }
                } else {
                    for (i=0; i<N-2; ++i) {
                        if ((i%2) == 0) {
                            compute(pset, vx, nx, tx, i, i+1, i+2);                            
                        } else {
                            compute(pset, vx, nx, tx, i+1, i, i+2);
                        }
                    }
                }
                break;

            case osg::PrimitiveSet::TRIANGLE_FAN:
                if (pset->getType() == osg::PrimitiveSet::DrawArrayLengthsPrimitiveType) {
                    osg::DrawArrayLengths *dal = static_cast<osg::DrawArrayLengths *>(pset);
                    unsigned int j = 0;
                    for (osg::DrawArrayLengths::const_iterator pi=dal->begin(); pi!=dal->end(); ++pi) {
                        unsigned int iN = static_cast<unsigned int>(*pi-2);
                        for (i=0; i<iN; ++i) {
                            compute(pset, vx, nx, tx, 0, j+1, j+2);
                        }
                        j += 2;
                    }
                } else {
                    for (i=0; i<N-2; ++i) {
                        compute(pset, vx, nx, tx, 0, i+1, i+2);
                    }
                }
                break;

            case osg::PrimitiveSet::POINTS:
            case osg::PrimitiveSet::LINES:
            case osg::PrimitiveSet::LINE_STRIP:
            case osg::PrimitiveSet::LINE_LOOP:
                break;

            default: osg::notify(osg::WARN) << "Warning: TangentSpaceGeneratorDouble: unknown primitive mode " << pset->getMode() << "\n";
        }
    }

    // normalize basis vectors and force the normal vector to match
    // the triangle normal's direction
    unsigned int attrib_count = vx->getNumElements();
    if (geo->getVertexIndices() != NULL) {
        attrib_count = geo->getVertexIndices()->getNumElements();
    }
    for (i=0; i<attrib_count; ++i) {
        osg::Vec4 &vT = (*T_)[i];
        osg::Vec4 &vB = (*B_)[i];
        osg::Vec4 &vN = (*N_)[i];
    
        osg::Vec3d txN = osg::Vec3d(vT.x(), vT.y(), vT.z()) ^ osg::Vec3d(vB.x(), vB.y(), vB.z());
        
        if (txN * osg::Vec3d(vN.x(), vN.y(), vN.z()) >= 0) {
            vN = osg::Vec4(txN, 0);
        } else {
            vN = osg::Vec4(-txN, 0);
        }

        vT.normalize();
        vB.normalize();
        vN.normalize();
    }
    /* TO-DO: if indexed, compress the attributes to have only one
     * version of each (different indices for each one?) */
}

void TangentSpaceGeneratorDouble::compute(osg::PrimitiveSet *pset,
                                    const osg::Array* vx,
                                    const osg::Array* nx,
                                    const osg::Array* tx,
                                    int iA, int iB, int iC)
{
    iA = pset->index(iA);
    iB = pset->index(iB);
    iC = pset->index(iC);

    osg::Vec3d P1;
    osg::Vec3d P2;
    osg::Vec3d P3;

    int i; // VC6 doesn't like for-scoped variables

    switch (vx->getType())
    {
    case osg::Array::Vec2ArrayType:
        for (i=0; i<2; ++i) {
            P1.ptr()[i] = static_cast<const osg::Vec2Array&>(*vx)[iA].ptr()[i];
            P2.ptr()[i] = static_cast<const osg::Vec2Array&>(*vx)[iB].ptr()[i];
            P3.ptr()[i] = static_cast<const osg::Vec2Array&>(*vx)[iC].ptr()[i];
        }
        break;

    case osg::Array::Vec3ArrayType:
        P1 = static_cast<const osg::Vec3Array&>(*vx)[iA];
        P2 = static_cast<const osg::Vec3Array&>(*vx)[iB];
        P3 = static_cast<const osg::Vec3Array&>(*vx)[iC];
        break;

    case osg::Array::Vec4ArrayType:
        for (i=0; i<3; ++i) {
            P1.ptr()[i] = static_cast<const osg::Vec4Array&>(*vx)[iA].ptr()[i];
            P2.ptr()[i] = static_cast<const osg::Vec4Array&>(*vx)[iB].ptr()[i];
            P3.ptr()[i] = static_cast<const osg::Vec4Array&>(*vx)[iC].ptr()[i];
        }
        break;

    default:
        osg::notify(osg::WARN) << "Warning: TangentSpaceGeneratorDouble: vertex array must be Vec2Array, Vec3Array or Vec4Array" << std::endl;
    }

    osg::Vec3d N1;
    osg::Vec3d N2;
    osg::Vec3d N3;

    if(nx)
    {
        switch (nx->getType())
        {
        case osg::Array::Vec2ArrayType:
            for (i=0; i<2; ++i) {
                N1.ptr()[i] = static_cast<const osg::Vec2Array&>(*nx)[iA].ptr()[i];
                N2.ptr()[i] = static_cast<const osg::Vec2Array&>(*nx)[iB].ptr()[i];
                N3.ptr()[i] = static_cast<const osg::Vec2Array&>(*nx)[iC].ptr()[i];
            }
            break;

        case osg::Array::Vec3ArrayType:
            N1 = static_cast<const osg::Vec3Array&>(*nx)[iA];
            N2 = static_cast<const osg::Vec3Array&>(*nx)[iB];
            N3 = static_cast<const osg::Vec3Array&>(*nx)[iC];
            break;

        case osg::Array::Vec4ArrayType:
            for (i=0; i<3; ++i) {
                N1.ptr()[i] = static_cast<const osg::Vec4Array&>(*nx)[iA].ptr()[i];
                N2.ptr()[i] = static_cast<const osg::Vec4Array&>(*nx)[iB].ptr()[i];
                N3.ptr()[i] = static_cast<const osg::Vec4Array&>(*nx)[iC].ptr()[i];
            }
            break;

        default:
            osg::notify(osg::WARN) << "Warning: TangentSpaceGeneratorDouble: normal array must be Vec2Array, Vec3Array or Vec4Array" << std::endl;
        }
    }

    osg::Vec2d uv1;
    osg::Vec2d uv2;
    osg::Vec2d uv3;

    switch (tx->getType())
    {
    case osg::Array::Vec2ArrayType:
        uv1 = static_cast<const osg::Vec2Array&>(*tx)[iA];
        uv2 = static_cast<const osg::Vec2Array&>(*tx)[iB];
        uv3 = static_cast<const osg::Vec2Array&>(*tx)[iC];
        break;

    case osg::Array::Vec3ArrayType:
        for (i=0; i<2; ++i) {
            uv1.ptr()[i] = static_cast<const osg::Vec3Array&>(*tx)[iA].ptr()[i];
            uv2.ptr()[i] = static_cast<const osg::Vec3Array&>(*tx)[iB].ptr()[i];
            uv3.ptr()[i] = static_cast<const osg::Vec3Array&>(*tx)[iC].ptr()[i];
        }
        break;

    case osg::Array::Vec4ArrayType:
        for (i=0; i<2; ++i) {
            uv1.ptr()[i] = static_cast<const osg::Vec4Array&>(*tx)[iA].ptr()[i];
            uv2.ptr()[i] = static_cast<const osg::Vec4Array&>(*tx)[iB].ptr()[i];
            uv3.ptr()[i] = static_cast<const osg::Vec4Array&>(*tx)[iC].ptr()[i];
        }
        break;

    default:
        osg::notify(osg::WARN) << "Warning: TangentSpaceGeneratorDouble: texture coord array must be Vec2Array, Vec3Array or Vec4Array" << std::endl;
    }

    if(nx){
        osg::Vec3d V, T1, T2, T3, B1, B2, B3;

        V = osg::Vec3d(P2.x() - P1.x(), uv2.x() - uv1.x(), uv2.y() - uv1.y()) ^ 
            osg::Vec3d(P3.x() - P1.x(), uv3.x() - uv1.x(), uv3.y() - uv1.y());
        if (V.x() != 0) {
            V.normalize();
            T1.x() += -V.y() / V.x();
            B1.x() += -V.z() / V.x();
            T2.x() += -V.y() / V.x();
            B2.x() += -V.z() / V.x();
            T3.x() += -V.y() / V.x();
            B3.x() += -V.z() / V.x();            
        }

        V = osg::Vec3d(P2.y() - P1.y(), uv2.x() - uv1.x(), uv2.y() - uv1.y()) ^ 
            osg::Vec3d(P3.y() - P1.y(), uv3.x() - uv1.x(), uv3.y() - uv1.y());
        if (V.x() != 0) {
            V.normalize();
            T1.y() += -V.y() / V.x();
            B1.y() += -V.z() / V.x();
            T2.y() += -V.y() / V.x();
            B2.y() += -V.z() / V.x();
            T3.y() += -V.y() / V.x();
            B3.y() += -V.z() / V.x();            
        }

        V = osg::Vec3d(P2.z() - P1.z(), uv2.x() - uv1.x(), uv2.y() - uv1.y()) ^ 
            osg::Vec3d(P3.z() - P1.z(), uv3.x() - uv1.x(), uv3.y() - uv1.y());
        if (V.x() != 0) {
            V.normalize();
            T1.z() += -V.y() / V.x();
            B1.z() += -V.z() / V.x();
            T2.z() += -V.y() / V.x();
            B2.z() += -V.z() / V.x();
            T3.z() += -V.y() / V.x();
            B3.z() += -V.z() / V.x();            
        }

        if( (T1.length() == 0) && (B1.length() == 0) &&
            (T2.length() == 0) && (B2.length() == 0) &&
            (T3.length() == 0) && (B3.length() == 0) )
        {
            osg::notify( osg::WARN ) << "Bad tangent space." << std::endl;
            osg::notify( osg::WARN ) << " iA " << iA << " uv1 " << uv1 << std::endl;
            osg::notify( osg::WARN ) << " iB " << iB << " uv2 " << uv2 << std::endl;
            osg::notify( osg::WARN ) << " iC " << iC << " uv3 " << uv3 << std::endl;
        }

        osg::Vec3d tempvec;
        tempvec = N1 ^ T1;
        (*T_)[iA] = osg::Vec4(tempvec ^ N1, 0);
        tempvec = B1 ^ N1;
        (*B_)[iA] = osg::Vec4(N1 ^ tempvec, 0);
        tempvec = N2 ^ T2;
        (*T_)[iB] = osg::Vec4(tempvec ^ N2, 0);
        tempvec = B2 ^ N2;
        (*B_)[iB] = osg::Vec4(N2 ^ tempvec, 0);
        tempvec = N3 ^ T3;
        (*T_)[iC] = osg::Vec4(tempvec ^ N3, 0);
        tempvec = B3 ^ N3;
        (*B_)[iC] = osg::Vec4(N3 ^ tempvec, 0);

        (*N_)[iA] += osg::Vec4(N1, 0);
        (*N_)[iB] += osg::Vec4(N2, 0);
        (*N_)[iC] += osg::Vec4(N3, 0);
    }
    else{
        osg::Vec3d face_normal = (P2 - P1) ^ (P3 - P1);

        osg::Vec3d V;

        V = osg::Vec3d(P2.x() - P1.x(), uv2.x() - uv1.x(), uv2.y() - uv1.y()) ^ 
            osg::Vec3d(P3.x() - P1.x(), uv3.x() - uv1.x(), uv3.y() - uv1.y());
        if (V.x() != 0) {
            V.normalize();
            (*T_)[iA].x() += -V.y() / V.x();
            (*B_)[iA].x() += -V.z() / V.x();
            (*T_)[iB].x() += -V.y() / V.x();
            (*B_)[iB].x() += -V.z() / V.x();
            (*T_)[iC].x() += -V.y() / V.x();
            (*B_)[iC].x() += -V.z() / V.x();            
        }

        V = osg::Vec3d(P2.y() - P1.y(), uv2.x() - uv1.x(), uv2.y() - uv1.y()) ^ 
            osg::Vec3d(P3.y() - P1.y(), uv3.x() - uv1.x(), uv3.y() - uv1.y());
        if (V.x() != 0) {
            V.normalize();
            (*T_)[iA].y() += -V.y() / V.x();
            (*B_)[iA].y() += -V.z() / V.x();
            (*T_)[iB].y() += -V.y() / V.x();
            (*B_)[iB].y() += -V.z() / V.x();
            (*T_)[iC].y() += -V.y() / V.x();
            (*B_)[iC].y() += -V.z() / V.x();            
        }

        V = osg::Vec3d(P2.z() - P1.z(), uv2.x() - uv1.x(), uv2.y() - uv1.y()) ^ 
            osg::Vec3d(P3.z() - P1.z(), uv3.x() - uv1.x(), uv3.y() - uv1.y());
        if (V.x() != 0) {
            V.normalize();
            (*T_)[iA].z() += -V.y() / V.x();
            (*B_)[iA].z() += -V.z() / V.x();
            (*T_)[iB].z() += -V.y() / V.x();
            (*B_)[iB].z() += -V.z() / V.x();
            (*T_)[iC].z() += -V.y() / V.x();
            (*B_)[iC].z() += -V.z() / V.x();            
        }

        (*N_)[iA] += osg::Vec4(face_normal, 0);
        (*N_)[iB] += osg::Vec4(face_normal, 0);
        (*N_)[iC] += osg::Vec4(face_normal, 0);
    }
}


// osgwTools
}
