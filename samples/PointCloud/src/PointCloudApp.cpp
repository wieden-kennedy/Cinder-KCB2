/*
* 
* Copyright (c) 2014, Wieden+Kennedy
* Stephen Schieberl
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include "cinder/app/App.h"
#include "cinder/MayaCamUI.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/params/Params.h"

#include "Kinect2.h"

class PointCloudApp : public ci::app::App
{
public:
	void						draw() override;
	void 						mouseDrag( ci::app::MouseEvent event ) override;
	void						resize() override;
	void						setup() override;
	void						update() override;
private:
	ci::Channel16uRef			mChannelDepth;
	Kinect2::DeviceRef			mDevice;
	ci::Surface8uRef			mSurfaceColor;
	ci::Surface32fRef			mSurfaceDepthToCameraTable;
	ci::Surface32fRef			mSurfaceDepthToColorTable;
	long long					mTimeStamp;
	long long					mTimeStampPrev;

	void						loadGlsl();
	ci::gl::GlslProgRef			mGlslProg;
	ci::gl::TextureRef			mTextureColor;
	ci::gl::TextureRef			mTextureDepth;
	ci::gl::TextureRef			mTextureDepthToCameraTable;
	ci::gl::TextureRef			mTextureDepthToColorTable;
	ci::gl::VboMeshRef			mVboMesh;

	ci::MayaCamUI				mMayaCam;

	float						mFrameRate;
	bool						mFullScreen;
	ci::params::InterfaceGlRef	mParams;
};

#include "cinder/app/RendererGl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void PointCloudApp::draw()
{
	gl::viewport( getWindowSize() );
	gl::clear();
	gl::setMatrices( mMayaCam.getCamera() );
	gl::enableAlphaBlending();
	gl::enableDepthRead();
	gl::enableDepthWrite();
	
	if ( mSurfaceColor ) {
		if ( mTextureColor ) {
			mTextureColor->update( *mSurfaceColor );
		} else {
			mTextureColor = gl::Texture::create( *mSurfaceColor );
		}
		mTextureColor->bind( 0 );
	}
	if ( mChannelDepth ) {
		if ( mTextureDepth ) {
			gl::ScopedTextureBind scopeTextureBind( mTextureDepth->getTarget(), mTextureDepth->getId() );
			glTexSubImage2D( mTextureDepth->getTarget(), 0, 0, 0,
				mTextureDepth->getWidth(), mTextureDepth->getHeight(),
				GL_RED_INTEGER,	GL_UNSIGNED_SHORT, mChannelDepth->getData() );
		} else {
			mTextureDepth = gl::Texture::create( 
				mChannelDepth->getWidth(), mChannelDepth->getHeight(), 
				gl::Texture::Format().dataType( GL_UNSIGNED_SHORT ).internalFormat( GL_R16UI ) );
		}
		mTextureDepth->bind( 1 );
	}
	if ( mSurfaceDepthToCameraTable && !mTextureDepthToCameraTable ) {
		mTextureDepthToCameraTable = gl::Texture::create( *mSurfaceDepthToCameraTable );
		mTextureDepthToCameraTable->bind( 2 );
	}
	if ( mSurfaceDepthToColorTable ) {
		if ( mTextureDepthToColorTable ) {
			mTextureDepthToColorTable->update( *mSurfaceDepthToColorTable );
		} else {
			mTextureDepthToColorTable = gl::Texture::create( 
				*mSurfaceDepthToColorTable,
				gl::Texture::Format().dataType( GL_FLOAT ) );
		}
		mTextureDepthToColorTable->bind( 3 );
	}

	gl::ScopedGlslProg scopeGlsl( mGlslProg );
	gl::setDefaultShaderVars();
	mGlslProg->uniform( "uTextureColor",				0 );
	mGlslProg->uniform( "uTextureDepth",				1 );
	mGlslProg->uniform( "uTextureDepthToCameraTable",	2 );
	mGlslProg->uniform( "uTextureDepthToColorTable",	3 );

	gl::draw( mVboMesh );
	
	if ( mTextureColor ) {
		mTextureColor->unbind();
	}
	if ( mTextureDepth ) {
		mTextureDepth->unbind();
	}
	if ( mTextureDepthToCameraTable ) {
		mTextureDepthToCameraTable->unbind();
	}
	if ( mTextureDepthToColorTable ) {
		mTextureDepthToColorTable->unbind();
	}

	mParams->draw();
}

void PointCloudApp::loadGlsl()
{
	try {
		mGlslProg = gl::GlslProg::create( gl::GlslProg::Format()
										  .vertex( loadAsset( "cloud.vert" ) )
										  .fragment( loadAsset( "cloud.frag" ) ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		console() << "GLSL Error: " << ex.what() << endl;
		quit();
	} catch ( gl::GlslNullProgramExc ex ) {
		console() << "GLSL Error: " << ex.what() << endl;
		quit();
	} catch ( ... ) {
		console() << "Unknown GLSL Error" << endl;
		quit();
	}
}

void PointCloudApp::mouseDrag( MouseEvent event )
{
	bool middle = event.isMiddleDown()	|| ( event.isMetaDown()		&& event.isLeftDown() );
	bool right	= event.isRightDown()	|| ( event.isControlDown()	&& event.isLeftDown() );
	mMayaCam.mouseDrag( event.getPos(), event.isLeftDown() && !middle && !right, middle, right );
}

void PointCloudApp::resize()
{
	CameraPersp cam = mMayaCam.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam( cam );

	gl::enableVerticalSync();
}

void PointCloudApp::setup()
{	
	gl::enable( GL_TEXTURE_2D );
	
	mFrameRate		= 0.0f;
	mFullScreen		= false;
	mTimeStamp		= 0L;
	mTimeStampPrev	= mTimeStamp;
	
	loadGlsl();

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectColorEventHandler( [ & ]( const Kinect2::ColorFrame& frame )
	{
		mSurfaceColor	= frame.getSurface();
	} );
	mDevice->connectDepthEventHandler( [ & ]( const Kinect2::DepthFrame& frame )
	{
		mChannelDepth	= frame.getChannel();
		mTimeStamp		= frame.getTimeStamp();
	} );

	//////////////////////////////////////////////////////////////////////////////////////////////

	ivec2 sz = Kinect2::DepthFrame().getSize();
	vector<vec2> vertices;
	for ( int32_t x = 0; x < sz.x; ++x ) {
		for ( int32_t y = 0; y < sz.y; ++y ) {
			vertices.push_back( vec2( x, y ) / vec2( sz ) );
		}
	}

	gl::VboRef vbo = gl::Vbo::create( GL_ARRAY_BUFFER, vertices.size() * sizeof( vec2 ), &vertices[ 0 ], GL_STATIC_DRAW );

	geom::BufferLayout layout;
	layout.append( geom::Attrib::POSITION, 2, sizeof( vec2 ), 0 );
	vector<pair<geom::BufferLayout, gl::VboRef>> vertexArrayBuffers = { make_pair( layout, vbo ) };

	mVboMesh = gl::VboMesh::create( vertices.size(), GL_POINTS, vertexArrayBuffers );

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	mParams = params::InterfaceGl::create( "Params", ivec2( 200, 120 ) );
	mParams->addParam( "Frame rate",	&mFrameRate,				"", true );
	mParams->addParam( "Full screen",	&mFullScreen ).key( "f" );
	mParams->addButton( "Load GLSL",	[ & ]() { loadGlsl(); },	"key=g" );
	mParams->addButton( "Quit",			[ & ]() { quit(); },		"key=q" );

	resize();
}

void PointCloudApp::update()
{
	mFrameRate = getAverageFps();
	
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}

	if ( !mSurfaceDepthToCameraTable ) {
		mSurfaceDepthToCameraTable = mDevice->mapDepthToCameraTable();
	}

	if ( ( mTimeStamp != mTimeStampPrev ) && mSurfaceColor && mChannelDepth ) {
		mTimeStampPrev = mTimeStamp;

		mSurfaceDepthToColorTable	= Surface32f::create( mChannelDepth->getWidth(), mChannelDepth->getHeight(), false, SurfaceChannelOrder::RGB );
		vector<ivec2> positions		= mDevice->mapDepthToColor( mChannelDepth );

		vec2 sz( Kinect2::ColorFrame().getSize() );

		Surface32f::Iter iter		= mSurfaceDepthToColorTable->getIter();
		vector<ivec2>::iterator v	= positions.begin();
		while ( iter.line() ) {
			while ( iter.pixel() ) {
				iter.r() = (float)v->x / sz.x;
				iter.g() = 1.0f - (float)v->y / sz.y;
				iter.b() = 0.0f;
				++v;
			}
		}
	}
}

CINDER_APP( PointCloudApp, RendererGl( RendererGl::Options().coreProfile() ), []( App::Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 1280, 960 ).title( "Point Cloud App" ) );
	settings->setFrameRate( 60.0f );
} )
 