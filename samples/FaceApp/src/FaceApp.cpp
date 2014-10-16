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
#include "cinder/params/Params.h"

#include "Kinect2.h"

// NOTE There is a memory leak in the Kinect SDK 1409 and lower 
//		when the model vertices are generated. Please do not use
//		this feature in production until this issue is resolved.

class FaceApp : public ci::app::AppBasic 
{
public:
	void							draw();
	void							prepareSettings( ci::app::AppBasic::Settings* settings );
	void							setup();
	void							update();
private:
	std::vector<Kinect2::Body>		mBodies;
	Kinect2::DeviceRef				mDevice;
	bool							mEnabledFace2d;
	bool							mEnabledFace3d;
	ci::Surface8u					mSurface;

	float							mFrameRate;
	bool							mFullScreen;
	ci::params::InterfaceGlRef		mParams;
};

using namespace ci;
using namespace ci::app;
using namespace std;

void FaceApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );

	if ( mSurface ) {	
		gl::color( Colorf::white() );
		gl::enable( GL_TEXTURE_2D );
		gl::TextureRef tex = gl::Texture::create( mSurface );
		gl::draw( tex, tex->getBounds(), Rectf( getWindowBounds() ) );
	
		gl::disable( GL_TEXTURE_2D );
		gl::pushMatrices();
		gl::scale( Vec2f( getWindowSize() ) / Vec2f( mSurface.getSize() ) );
		for ( const Kinect2::Body& body : mBodies ) {
			if ( body.isTracked() ) {

				gl::color( Colorf::white() );
				const Kinect2::Body::Face3d& face3d = body.getFace3d();
				if ( face3d.isTracked() ) {

					const TriMesh& mesh = face3d.getMesh();
					if ( mesh.getNumIndices() > 0 ) {
						vector<Vec2f> verts;
						for ( const Vec3f& i : mesh.getVertices() ) {
							Vec2f v = mDevice->mapCameraToColor( i );
							verts.push_back( v );
						}

						gl::lineWidth( 0.5f );
						gl::enableWireframe();
						TriMesh2d mesh2d;
						mesh2d.appendIndices( &mesh.getIndices()[ 0 ], mesh.getNumIndices() );
						mesh2d.appendVertices( &verts[ 0 ], mesh.getNumVertices() );
						gl::draw( mesh2d );
						gl::disableWireframe();
					}
				}

				if ( mEnabledFace3d ) {
					gl::color( Colorf( 1.0f, 0.0f, 0.0f ) );
				} else {
					gl::lineWidth( 2.0f );
				}
				const Kinect2::Body::Face2d& face2d = body.getFace2d();
				if ( face2d.isTracked() ) {
					gl::drawStrokedRect( face2d.getBoundsColor() );
					for ( const Vec2f& i : face2d.getPointsColor() ) {
						gl::drawSolidCircle( i, 3.0f, 16 );
					}
				}
			}
		}
		gl::popMatrices();
	}

	mParams->draw();
}

void FaceApp::prepareSettings( Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 960, 540 ).title( "Face App" ) );
	settings->setFrameRate( 60.0f );
}

void FaceApp::setup()
{	
	gl::enableAlphaBlending();
	
	mEnabledFace2d	= true;
	mEnabledFace3d	= true;
	mFrameRate		= 0.0f;
	mFullScreen		= false;

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler( [ & ]( const Kinect2::BodyFrame& frame )
	{
		mBodies = frame.getBodies();
	} );
	mDevice->connectColorEventHandler( [ & ]( const Kinect2::ColorFrame frame )
	{
		mSurface = frame.getSurface();
	} );
		
	mParams = params::InterfaceGl::create( "Params", Vec2i( 230, 130 ) );
	mParams->addParam( "Frame rate",		&mFrameRate,			"", true );
	mParams->addParam( "Full screen",		&mFullScreen ).key( "f" );
	mParams->addParam( "2d face tracking",	&mEnabledFace2d ).key( "2" );
	mParams->addParam( "3d face tracking",	&mEnabledFace3d ).key( "3" );
	mParams->addButton( "Quit",				[ & ]() { quit(); } ,	"key=q" );
}

void FaceApp::update()
{
	mFrameRate = getAverageFps();
	
	mDevice->enableFaceTracking2d( mEnabledFace2d );
	mDevice->enableFaceTracking3d( mEnabledFace3d );

	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}
}

CINDER_APP_BASIC( FaceApp, RendererGl )
	