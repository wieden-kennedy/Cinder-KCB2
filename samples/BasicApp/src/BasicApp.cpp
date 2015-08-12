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
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"

#include "Kinect2.h"

class BasicApp : public ci::app::App 
{
public:
	void						draw() override;
	void						setup() override;
	void						update() override;
private:
	Kinect2::DeviceRef			mDevice;
	ci::Channel8uRef			mChannelBodyIndex;
	ci::Channel16uRef			mChannelDepth;
	ci::Channel16uRef			mChannelInfrared;
	ci::Surface8uRef			mSurfaceColor;

	float						mFrameRate;
	bool						mFullScreen;
	ci::params::InterfaceGlRef	mParams;
};

#include "cinder/app/RendererGl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void BasicApp::draw()
{
	gl::viewport( getWindowSize() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );
	gl::enableAlphaBlending();
	
	if ( mSurfaceColor ) {
		gl::TextureRef tex = gl::Texture::create( *mSurfaceColor );
		gl::draw( tex, tex->getBounds(), Rectf( vec2( 0.0f ), getWindowCenter() ) );
	}
	if ( mChannelDepth ) {
		gl::TextureRef tex = gl::Texture::create( *Kinect2::channel16To8( mChannelDepth ) );
		gl::draw( tex, tex->getBounds(), Rectf( getWindowCenter().x, 0.0f, (float)getWindowWidth(), getWindowCenter().y ) );
	}
	if ( mChannelInfrared ) {
		gl::TextureRef tex = gl::Texture::create( *mChannelInfrared );
		gl::draw( tex, tex->getBounds(), Rectf( 0.0f, getWindowCenter().y, getWindowCenter().x, (float)getWindowHeight() ) );
	}
	if ( mChannelBodyIndex ) {
		gl::TextureRef tex = gl::Texture::create( *Kinect2::colorizeBodyIndex( mChannelBodyIndex ) );
		gl::draw( tex, tex->getBounds(), Rectf( getWindowCenter(), vec2( getWindowSize() ) ) );
	}

	mParams->draw();
}

void BasicApp::setup()
{	
	gl::enable( GL_TEXTURE_2D );
	
	mFrameRate	= 0.0f;
	mFullScreen	= false;

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyIndexEventHandler( [ & ]( const Kinect2::BodyIndexFrame& frame )
	{
		mChannelBodyIndex = frame.getChannel();
	} );
	mDevice->connectColorEventHandler( [ & ]( const Kinect2::ColorFrame& frame )
	{
		mSurfaceColor = frame.getSurface();
	} );
	mDevice->connectDepthEventHandler( [ & ]( const Kinect2::DepthFrame& frame )
	{
		mChannelDepth = frame.getChannel();
	} );
	mDevice->connectInfraredEventHandler( [ & ]( const Kinect2::InfraredFrame& frame )
	{
		mChannelInfrared = frame.getChannel();
	} );
	
	mParams = params::InterfaceGl::create( "Params", ivec2( 200, 100 ) );
	mParams->addParam( "Frame rate",	&mFrameRate,			"", true );
	mParams->addParam( "Full screen",	&mFullScreen ).key( "f" );
	mParams->addButton( "Quit",			[ & ]() { quit(); } ,	"key=q" );
}

void BasicApp::update()
{
	mFrameRate = getAverageFps();
	
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}
}

CINDER_APP( BasicApp, RendererGl, []( App::Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 800, 600 ).title( "Basic App" ) );
	settings->setFrameRate( 60.0f );
} )
	