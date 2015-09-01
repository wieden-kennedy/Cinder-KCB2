/*
* 
* Copyright (c) 2015, Wieden+Kennedy
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
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"

#include "Kinect2.h"

class PointCloudApp : public ci::app::App
{
public:
	PointCloudApp();

	void						draw() override;
	void						resize() override;
	void						update() override;
private:
	ci::Channel16uRef			mChannelDepth;
	Kinect2::DeviceRef			mDevice;
	ci::Surface8uRef			mSurfaceColor;
	ci::Surface32fRef			mSurfaceDepthToCameraTable;
	ci::Surface32fRef			mSurfaceDepthToColorTable;
	long long					mTimeStamp;
	long long					mTimeStampPrev;

	ci::gl::Texture2dRef		mTextureColor;
	ci::gl::Texture2dRef		mTextureDepth;
	ci::gl::Texture2dRef		mTextureDepthToCameraTable;
	ci::gl::Texture2dRef		mTextureDepthToColorTable;
	ci::gl::BatchRef			mBatchPointCloud;

	ci::CameraPersp				mCamera;
	ci::CameraUi				mCamUi;

	float						mFrameRate;
	bool						mFullScreen;
	ci::params::InterfaceGlRef	mParams;
};

#include "cinder/app/RendererGl.h"
#include "cinder/Log.h"

using namespace ci;
using namespace ci::app;
using namespace std;

PointCloudApp::PointCloudApp()
{	
	mFrameRate		= 0.0f;
	mFullScreen		= false;
	mTimeStamp		= 0L;
	mTimeStampPrev	= mTimeStamp;

	mCamera = CameraPersp( getWindowWidth(), getWindowHeight(), 60.0f, 1.0f, 5000.0f );
	mCamera.lookAt( vec3( 0.0f ), vec3( 0.0f, 0.0f, 1.0f ) );
	mCamUi	= CameraUi( &mCamera, getWindow() );

	gl::GlslProgRef glsl;
	try {
		glsl = gl::GlslProg::create( gl::GlslProg::Format()
			.version( 330 )
			.vertex( loadAsset( "cloud.vert" ) )
			.fragment( loadAsset( "cloud.frag" ) ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		CI_LOG_V( "GLSL Error: " << ex.what() );
	} catch ( gl::GlslNullProgramExc ex ) {
		CI_LOG_V( "GLSL Error: " << ex.what() );
	} catch ( ... ) {
		CI_LOG_V( "Unknown GLSL Error" );
	}
	if ( !glsl ) {
		quit();
		return;
	}

	glsl->uniform( "uTextureColor",					0 );
	glsl->uniform( "uTextureDepth",					1 );
	glsl->uniform( "uTextureDepthToCameraTable",	2 );
	glsl->uniform( "uTextureDepthToColorTable",		3 );

	gl::VertBatch vertBatch;
	ivec2 sz = Kinect2::DepthFrame().getSize();
	for ( int32_t x = 0; x < sz.x; ++x ) {
		for ( int32_t y = 0; y < sz.y; ++y ) {
			const vec2 v = vec2( x, y ) / vec2( sz );
			vertBatch.texCoord0( v );
			vertBatch.vertex( v );
		}
	}
	mBatchPointCloud = gl::Batch::create( vertBatch, glsl );

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectColorEventHandler( [ & ]( const Kinect2::ColorFrame& frame )
	{
		mSurfaceColor = frame.getSurface();
	} );
	mDevice->connectDepthEventHandler( [ & ]( const Kinect2::DepthFrame& frame )
	{
		mChannelDepth = frame.getChannel();
		mTimeStamp = frame.getTimeStamp();
	} );

	mParams = params::InterfaceGl::create( "Params", ivec2( 200, 120 ) );
	mParams->addParam( "Frame rate",	&mFrameRate,			"", true );
	mParams->addParam( "Full screen",	&mFullScreen ).key( "f" );
	mParams->addButton( "Quit",			[ & ]() { quit(); },	"key=q" );

	resize();
	
	gl::enableVerticalSync();
}

void PointCloudApp::draw()
{
	const gl::ScopedViewport scopedViewport( ivec2( 0 ), getWindowSize() );
	const gl::ScopedMatrices scopedMatrices;
	const gl::ScopedBlendAlpha scopedBlendAlpha;
	gl::clear();
	gl::setMatrices( mCamera );
	
	if ( mSurfaceColor ) {
		if ( mTextureColor ) {
			mTextureColor->update( *mSurfaceColor );
		} else {
			mTextureColor = gl::Texture::create( *mSurfaceColor );
		}
	}
	if ( mChannelDepth ) {
		if ( mTextureDepth ) {
			mTextureDepth->update( *mChannelDepth );
		} else {
			mTextureDepth = gl::Texture2d::create( *mChannelDepth );
		}
	}
	if ( mSurfaceDepthToCameraTable && !mTextureDepthToCameraTable ) {
		if ( mTextureDepthToCameraTable ) {
			mTextureDepthToCameraTable->update( *mSurfaceDepthToCameraTable );
		} else {
			mTextureDepthToCameraTable = gl::Texture::create( *mSurfaceDepthToCameraTable );
		}
	}
	if ( mSurfaceDepthToColorTable ) {
		if ( mTextureDepthToColorTable ) {
			mTextureDepthToColorTable->update( *mSurfaceDepthToColorTable );
		} else {
			mTextureDepthToColorTable = gl::Texture::create( *mSurfaceDepthToColorTable );
		}
	}

	if ( mTextureColor && mTextureDepth && mTextureDepthToCameraTable && mTextureDepthToColorTable ) {
		const gl::ScopedTextureBind scopedTextureBind0( mTextureColor,				0 );
		const gl::ScopedTextureBind scopedTextureBind1( mTextureDepth,				1 );
		const gl::ScopedTextureBind scopedTextureBind2( mTextureDepthToCameraTable, 2 );
		const gl::ScopedTextureBind scopedTextureBind3( mTextureDepthToColorTable,	3 );
		gl::setDefaultShaderVars();
		mBatchPointCloud->draw();
	}
	
	mParams->draw();
}

void PointCloudApp::resize()
{
	mCamera.setAspectRatio( getWindowAspectRatio() );
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

		mSurfaceDepthToColorTable		= Surface32f::create( mChannelDepth->getWidth(), mChannelDepth->getHeight(), false, SurfaceChannelOrder::RGB );
		const vector<ivec2> positions	= mDevice->mapDepthToColor( mChannelDepth );

		vec2 sz( Kinect2::ColorFrame().getSize() );

		Surface32f::Iter iter			= mSurfaceDepthToColorTable->getIter();
		vector<ivec2>::const_iterator v	= positions.begin();
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
	settings->disableFrameRate();
} )
 