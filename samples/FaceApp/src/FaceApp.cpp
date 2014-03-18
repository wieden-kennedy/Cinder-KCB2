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

#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"
#include "cinder/ip/Resize.h"
#include "cinder/params/Params.h"

#include "FaceTracker.h"
#include "Kinect2.h"

class FaceApp : public ci::app::AppBasic
{
public:
	void						draw();
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();
	void						update();
private:
	ci::Channel16u				mChannelInfrared;
	Kinect2::DeviceRef			mDevice;
	Kinect2::FaceTracker::Face	mFace;
	Kinect2::FaceTrackerRef		mFaceTracker;
	Kinect2::Frame				mFrame;
	ci::TriMesh2d				mMeshFace;

	float						mFrameRate;
	bool						mFullScreen;
	ci::params::InterfaceGlRef	mParams;
};

#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void FaceApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );
	gl::setMatricesWindow( getWindowSize() );
	gl::enableAlphaBlending();
	gl::color( ColorAf::white() );

	if ( mChannelInfrared ) {
		gl::TextureRef tex = gl::Texture::create( mChannelInfrared );
		gl::enable( GL_TEXTURE_2D );
		tex->bind();
		gl::drawSolidRect( Rectf( getWindowBounds() ) );
		tex->unbind();
		gl::disable( GL_TEXTURE_2D );

		if ( mMeshFace.getNumVertices() > 0 ) {
			gl::pushMatrices();
			gl::scale( Vec2f( getWindowSize() ) / Vec2f( mChannelInfrared.getSize() ) );
			gl::enableWireframe();
			gl::draw( mFace.getMesh2d() );
			gl::disableWireframe();
			gl::popMatrices();
		}
	}
	mParams->draw();
}

void FaceApp::prepareSettings( Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 1280, 720 ).title( "Face App" ) );
	settings->setFrameRate( 60.0f );
}

void FaceApp::setup()
{
	gl::enable( GL_TEXTURE_2D );
	
	mFrameRate	= 0.0f;
	mFullScreen	= false;
	
	mDevice = Kinect2::Device::create();
	try {
		mDevice->start( Kinect2::DeviceOptions().enableBodyIndex( false ).enableColor( false ).enableDepth( false ).enableInfrared() );
	} catch ( Kinect2::Device::ExcDeviceNotAvailable ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect2::Device::ExcDeviceOpenFailed ex ) {
		console() << ex.what() << endl;
	}

	mFaceTracker = Kinect2::FaceTracker::create();
	mFaceTracker->enableCalcMesh( false );
	mFaceTracker->enableCalcMesh2d();
	mFaceTracker->connectEventHander( [ & ]( Kinect2::FaceTracker::Face face )
	{
		mFace = face;
		if ( mFace.getBounds().getWidth() > 0 && mFace.getMesh2d().getNumVertices() > 0 ) {
			mMeshFace = mFace.getMesh2d();
		}
	} );
	try {
		mFaceTracker->start();
	} catch ( Kinect2::FaceTracker::ExcFaceTrackerCreate ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect2::FaceTracker::ExcFaceTrackerCreateImage ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect2::FaceTracker::ExcFaceTrackerCreateResult ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect2::FaceTracker::ExcFaceTrackerInit ex ) {
		console() << ex.what() << endl;
	} catch ( Kinect2::FaceTracker::ExcFaceTrackerNuiInit ex ) {
		console() << ex.what() << endl;
	}
	
	console() << Kinect2::getDeviceCount() << " device(s) connected." << endl;
	map<size_t, string> deviceMap = Kinect2::getDeviceMap();
	for ( const auto& device : deviceMap ) {
		console() << "Index: " << device.first << ", ID: " << device.second << endl;
	}

	mParams = params::InterfaceGl::create( "Params", Vec2i( 200, 100 ) );
	mParams->addParam( "Frame rate",	&mFrameRate,			"", true );
	mParams->addParam( "Full screen",	&mFullScreen,			"key=f" );
	mParams->addButton( "Quit",			[ & ]() { quit(); },	"key=q" );
}

void FaceApp::update()
{
	mFrameRate = getAverageFps();
	
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}
	
	if ( mDevice && mDevice->getFrame().getTimeStamp( Kinect2::Frame::TimeStamp::TIMESTAMP_INFRARED ) > mFrame.getTimeStamp( Kinect2::Frame::TimeStamp::TIMESTAMP_INFRARED ) ) {
		mFrame = mDevice->getFrame();

		if ( mFaceTracker && mFrame.getInfrared() ) {
			mChannelInfrared = mFrame.getInfrared();

			mFaceTracker->update( mChannelInfrared );
		}
	}
}

CINDER_APP_BASIC( FaceApp, RendererGl )
