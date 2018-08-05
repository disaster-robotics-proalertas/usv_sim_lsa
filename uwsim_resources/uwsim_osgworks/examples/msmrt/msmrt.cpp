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

#include <osg/Node>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/GLExtensions>
#include <osg/buffered_value>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgwTools/FBOUtils.h>
#include <osgwTools/Version.h>

#include <string>


const int winW( 800 ), winH( 600 );


/* \cond */
class MSMRTCallback : public osg::Camera::DrawCallback
{
public:
    MSMRTCallback( osg::Camera* cam )
      : _cam( cam )
    {
    }

    virtual void operator()( osg::RenderInfo& renderInfo ) const
    {
        osg::State& state = *renderInfo.getState();
        const unsigned int ctx = state.getContextID();
        osg::FBOExtensions* fboExt = osg::FBOExtensions::instance( ctx, true );

        PerContextInfo& ctxInfo( _contextInfo[ ctx ] );
        if( ctxInfo.__glGetFramebufferAttachmentParameteriv == NULL )
        {
            // Initialize function pointer for FBO query.
            osg::setGLExtensionFuncPtr( ctxInfo.__glGetFramebufferAttachmentParameteriv, "glGetFramebufferAttachmentParameteriv" );
            if( ctxInfo.__glGetFramebufferAttachmentParameteriv == NULL )
                osg::setGLExtensionFuncPtr( ctxInfo.__glGetFramebufferAttachmentParameteriv, "glGetFramebufferAttachmentParameterivEXT" );
            if( ctxInfo.__glGetFramebufferAttachmentParameteriv == NULL )
            {
                osg::notify( osg::ALWAYS ) << "Can't get function pointer glGetFramebufferAttachmentParameteriv" << std::endl;
                return;
            }
        }

        const GLint width = _cam->getViewport()->width();
        const GLint height = _cam->getViewport()->height();

#if 0
        // Make sure something is actually bound.
        GLint drawFBO( -1 );
        glGetIntegerv( GL_DRAW_FRAMEBUFFER_BINDING, &drawFBO );
#endif

        // BlitFramebuffer blits to all attached color buffers in the
        // draw FBO. We only want to blit to attachment1, so aave
        // attachment0 and then unbind it.
        GLint destColorTex0( -1 );
        ctxInfo.__glGetFramebufferAttachmentParameteriv(
            GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
            GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &destColorTex0 );
        osgwTools::glFramebufferTexture2D( fboExt,
            GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
            GL_TEXTURE_2D, 0, 0 );

        // Verification
        //osg::notify( osg::ALWAYS ) << "Dest " << std::hex << destColorTex0 << std::endl;

        // Set draw and read buffers to attachment1 to read from correct
        // buffer and avoid INVALID_FRAMEBUFFER_OPERATION error.
        glDrawBuffer( GL_COLOR_ATTACHMENT1 );
        glReadBuffer( GL_COLOR_ATTACHMENT1 );

        // Blit, from (multisampled read FBO) attachment1 to
        // (non-multisampled draw FBO) attachment1.
        osgwTools::glBlitFramebuffer( fboExt, 0, 0, width, height, 0, 0, width, height,
            GL_COLOR_BUFFER_BIT, GL_NEAREST );

        // Restore draw and read buffers
        glDrawBuffer( GL_COLOR_ATTACHMENT0 );
        glReadBuffer( GL_COLOR_ATTACHMENT0 );

        // Restore the draw FBO's attachment0.
        osgwTools::glFramebufferTexture2D( fboExt,
            GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
            GL_TEXTURE_2D, destColorTex0, 0 );

        // We disabled FBO unbinding in the RenderStage,
        // so do it ourself here.
        /* TBD update to use FBOUtils.h */
        osgwTools::glBindFramebuffer( fboExt, GL_FRAMEBUFFER, 0 );
    }

protected:
    osg::ref_ptr< osg::Camera > _cam;

    typedef void APIENTRY TglGetFramebufferAttachmentParameteriv( GLenum, GLenum, GLenum, GLint* );

    // Each different context could potentially have a different address for
    // the FBO query function. For this reason, keep it in buffered_value
    // and init / index the function pointer on a per-context basis.
    // Of course, this wouldn't be necessary if OSG already has this function
    // pointer in FBOExtensions. Or if OSG used something like GLEW.
    struct PerContextInfo
    {
        PerContextInfo()
        {
            __glGetFramebufferAttachmentParameteriv = NULL;
        }

        TglGetFramebufferAttachmentParameteriv* __glGetFramebufferAttachmentParameteriv;
    };
    mutable osg::buffered_object< PerContextInfo > _contextInfo;
};


// RenderStage unbinds FBOs before executing post-draw callbacks.
// The only way I know of to access the RenderStage (to disable this
// unbinding) is with a cull callback.
class KeepFBOsBoundCallback : public osg::NodeCallback
{
public:
    KeepFBOsBoundCallback() {}

    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        if( nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR )
        {
            traverse( node, nv );
            return;
        }

        // Get the current RenderStage and prevent it from unbinding the FBOs
        // just before our post-draw MSMRTCallback is executed. We need them
        // bound in our callback so we can execute another glBlitFramebuffer.
        // After the blit, MSMRTCallback unbinds the FBOs.
        osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
        // Don't use getRenderStage(). It returns the _root_ RenderStage.
        //osgUtil::RenderStage* rs = cv->getRenderStage();
        osgUtil::RenderStage* rs = cv->getCurrentRenderBin()->getStage();
        rs->setDisableFboAfterRender( false );

        traverse( node, nv );
    }
};
/* \endcond */


// State set for writing two color values. Used when rendering
// main scene graph.
void
mrtStateSet( osg::StateSet* ss )
{
    ss->addUniform( new osg::Uniform( "tex", 0 ) );

    std::string fragsource = 
        "uniform sampler2D tex; \n"
        "void main() \n"
        "{ \n"
            "gl_FragData[0] = texture2D( tex, gl_TexCoord[0].st ); // gl_Color; \n"
            "gl_FragData[1] = vec4( 0.0, 0.0, 1.0, 0.0 ); \n"
        "} \n";
    osg::Shader* fragShader = new osg::Shader();
    fragShader->setType( osg::Shader::FRAGMENT );
    fragShader->setShaderSource( fragsource );

    osg::Program* program = new osg::Program();
    program->addShader( fragShader );
    ss->setAttribute( program, osg::StateAttribute::ON );
}

// State set for combining two textures. Used
// when rendering fullscreen tri pairs.
void
mrtStateSetTriPair( osg::StateSet* ss, osg::Texture2D* tex0, osg::Texture2D* tex1 )
{
    ss->setTextureAttributeAndModes( 0, tex0, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes( 1, tex1, osg::StateAttribute::ON );

    ss->addUniform( new osg::Uniform( "tex0", 0 ) );
    ss->addUniform( new osg::Uniform( "tex1", 1 ) );

    std::string fragsource = 
        "uniform sampler2D tex0; \n"
        "uniform sampler2D tex1; \n"
        "void main() \n"
        "{ \n"
            "gl_FragData[0] = texture2D( tex0, gl_TexCoord[0].st ) \n"
                " + texture2D( tex1, gl_TexCoord[0].st ); \n"
        "} \n";
    osg::Shader* fragShader = new osg::Shader();
    fragShader->setType( osg::Shader::FRAGMENT );
    fragShader->setShaderSource( fragsource );

    osg::Program* program = new osg::Program();
    program->addShader( fragShader );
    ss->setAttribute( program, osg::StateAttribute::ON );

    ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
}


osg::Node*
postRender( osgViewer::Viewer& viewer )
{
    osg::Camera* rootCamera( viewer.getCamera() );

    // MRT: Attach two color buffers to the root camera, one for
    // the standard color image, and another for the glow color.
    osg::Texture2D* tex0 = new osg::Texture2D;
    tex0->setTextureWidth( winW );
    tex0->setTextureHeight( winH );
    tex0->setInternalFormat( GL_RGBA );
    tex0->setBorderWidth( 0 );
    tex0->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
    tex0->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
    // Full color: attachment 0
    rootCamera->attach( osg::Camera::COLOR_BUFFER0, tex0, 0, 0, false, 8, 8 );

    osg::Texture2D* tex1 = new osg::Texture2D;
    tex1->setTextureWidth( winW );
    tex1->setTextureHeight( winH );
    tex1->setInternalFormat( GL_RGBA );
    tex1->setBorderWidth( 0 );
    tex1->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
    tex1->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
    // Glow color: attachment 1
    rootCamera->attach( osg::Camera::COLOR_BUFFER1, tex1, 0, 0, false, 8, 8 );

    rootCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT, osg::Camera::FRAME_BUFFER );
#if( OSGWORKS_OSG_VERSION >= 20906 )
    rootCamera->setImplicitBufferAttachmentMask(
        osg::Camera::IMPLICIT_COLOR_BUFFER_ATTACHMENT|osg::Camera::IMPLICIT_DEPTH_BUFFER_ATTACHMENT,
        osg::Camera::IMPLICIT_COLOR_BUFFER_ATTACHMENT );
#endif

    // Post-draw callback on root camera handles resolving
    // multisampling for the MRT case.
    MSMRTCallback* msmrt = new MSMRTCallback( rootCamera );
    rootCamera->setPostDrawCallback( msmrt );


    // Configure postRenderCamera to draw fullscreen textured tri pair.
    // This will combine the two (resolved) textures into a single image.
    osg::ref_ptr< osg::Camera > postRenderCamera( new osg::Camera );
    postRenderCamera->setClearColor( osg::Vec4( 0., 1., 0., 1. ) ); // should never see this.
    postRenderCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER, osg::Camera::FRAME_BUFFER );

    postRenderCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
    postRenderCamera->setRenderOrder( osg::Camera::POST_RENDER );
    postRenderCamera->setViewMatrix( osg::Matrixd::identity() );
    postRenderCamera->setProjectionMatrix( osg::Matrixd::identity() );

    osg::Geode* geode( new osg::Geode );
    geode->setCullingActive( false );
    geode->addDrawable( osg::createTexturedQuadGeometry(
        osg::Vec3( -1,-1,0 ), osg::Vec3( 2,0,0 ), osg::Vec3( 0,2,0 ) ) );
    mrtStateSetTriPair( geode->getOrCreateStateSet(), tex0, tex1 );

    postRenderCamera->addChild( geode );

    return( postRenderCamera.release() );
}

int
main( int argc, char** argv )
{
    osg::notify( osg::ALWAYS ) <<
// cols:  12345678901234567890123456789012345678901234567890123456789012345678901234567890
         "This is an example of doing multisampled rendering to multiple render targets" << std::endl <<
         "in OSG. It uses osgWorks only to check for OSG version and configure the" << std::endl <<
         "destination textures and RTT Cameras appropriately." << std::endl;

    osg::ref_ptr< osg::Group > root( new osg::Group );
    root->addChild( osgDB::readNodeFile( "cow.osg" ) );
    if( root->getNumChildren() == 0 )
        return( 1 );

    // Do not unbind the FBOs after the BlitFramebuffer call.
    root->setCullCallback( new KeepFBOsBoundCallback() );

    // Set fragment program for MRT.
    mrtStateSet( root->getOrCreateStateSet() );


    osgViewer::Viewer viewer;
    viewer.getCamera()->setClearColor( osg::Vec4( 0., 0., 0., 1. ) );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator );
    viewer.setUpViewInWindow( 10, 30, winW, winH );
    viewer.setSceneData( root.get() );
    viewer.realize();

    root->addChild( postRender( viewer ) );


    // Required for osgViewer threading model support.
    // Render at least 2 frames before NULLing the cull callback.
    int frameCount( 2 );

    while( !viewer.done() )
    {
        viewer.frame();

        if( frameCount > 0 )
        {
            frameCount--;
            if( frameCount == 0 )
                // After rendering, set cull callback to NULL.
                root->setCullCallback( NULL );
        }
    }
    return( 0 );
}


/** \page msmrt The msmrt Example
msmrt demonstrates rendering multisampled to multiple render targets.

This example demonstrates how to render to multiple multisampled render
targets. Typically, apps require that all multisampled color buffers
get resolved. However, OSG currently inhibits this because it uses a
single glBlitFramebuffer call where multiple calls would be required, one
for each attached color buffer.

To fix this issue, we leverage the Camera post draw callback to execute a
second glBlitFramebuffer call after OSG executes the first blit. The
MSMRTCallback class contains the post-draw callback. The code
assumes two color buffers are attached, and assumes OSG has already resolved
the first color buffer (attachment 0). The code saves the texture attached
as attachment 0, sets it to NULL (leaving only attachment 1), sets the draw
and read buffers to attachment 1, then performs a blit. This resolves
the multisampling in attachment 1. The callback then restores attachment 0.

During development, it was discovered that OSG unbinds the FBOs after it
performs a blit but before it calls the post-draw callback. However, our
callback requires that the FBOs be bound. OSG's RenderStage class can be
configured to keep these FBOs bound, with setDisableFboAfterRender( false ).
It is difficult to call directly into a RenderStage. The only way to execute
this call is with a cull callback, which can query the RenderStage from
the CullVisitor. The KeepFBOsBoundCallback class is responsible
for doing this. In theory, it needs to be done once per cull thread, during
the first frame (for example), then the cull callback can be removed.
However, osgViewer's threading models require this be done for at least
two frames.
*/
