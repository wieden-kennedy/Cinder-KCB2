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
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"

#include "Kinect2.h"

class FaceApp : public ci::app::App 
{
public:
	void							draw() override;
	void							setup() override;
	void							update() override;
private:
	Kinect2::DeviceRef				mDevice;
	bool							mEnabledBody;
	bool							mEnabledFace2d;
	bool							mEnabledFace3d;
	bool							mEnabledBodyAndFace2d;
	std::vector<Kinect2::Face2d>	mFaces2d;
	std::vector<Kinect2::Face3d>	mFaces3d;
	ci::Surface8uRef				mSurface;

	float							mFrameRate;
	bool							mFullScreen;
	ci::params::InterfaceGlRef		mParams;

	int mBodyCount = 0;

	int colorFrameCount = 0;
	float colorFps = 0;
	int bodyFrameCount = 0;
	float bodyFps = 0;
	int bodyAndFace2dFrameCount = 0;
	float bodyAndFace2dFps = 0;
	int face2dFrameCount = 0;
	float face2dFps = 0;
	int face3dFrameCount = 0;
	float face3dFps = 0;
	double timeLastFrame = 0;
	double timeSinceAvg = 0;

	float sumBodyFaceFps = 0;
};

#include "cinder/app/RendererGl.h"
#include "cinder/gl/VboMesh.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void FaceApp::draw()
{
	gl::viewport( getWindowSize() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );

	if ( mSurface ) {	
		gl::color( Colorf::white() );
		gl::enable( GL_TEXTURE_2D );
		gl::TextureRef tex = gl::Texture::create( *mSurface );
		gl::draw( tex, tex->getBounds(), Rectf( getWindowBounds() ) );
	
		gl::disable( GL_TEXTURE_2D );
		gl::pushMatrices();
		gl::scale( vec2( getWindowSize() ) / vec2( mSurface->getSize() ) );
		
		for ( const Kinect2::Face3d& face : mFaces3d ) {
			const TriMeshRef& mesh = face.getMesh();
			if ( mesh && mesh->getNumIndices() > 0 ) {
						
				// Map face points to color image
				vector<vec2> v2;
				vec3* v3 = mesh->getPositions<3>();
				for ( size_t i = 0; i < mesh->getNumVertices(); ++i ) {
					v2.push_back( mDevice->mapCameraToColor( v3[ i ] ) );
				}

				// Create VBO mesh from TriMesh indices and 2D vertices
				geom::BufferLayout bufferLayout;
				bufferLayout.append( geom::Attrib::POSITION, 2, 0, 0 );
				vector<pair<geom::BufferLayout, gl::VboRef>> vertexArrayBuffers = { 
					make_pair( bufferLayout, gl::Vbo::create( GL_ARRAY_BUFFER, mesh->getNumVertices() * sizeof( vec2 ), (void*)&v2[ 0 ] ) ) 
				};
				gl::VboMeshRef vboMesh = gl::VboMesh::create( 
					mesh->getNumVertices(), 
					mesh->getPrimitive(), 
					vertexArrayBuffers, 
					mesh->getNumIndices(), 
					GL_UNSIGNED_INT, 
					gl::Vbo::create( GL_ELEMENT_ARRAY_BUFFER, mesh->getNumIndices() * sizeof( uint32_t ), (void*)mesh->getIndices().data() ) 
					);

				gl::lineWidth( 0.5f );
				gl::enableWireframe();
				gl::draw( vboMesh );
				gl::disableWireframe();
			}
		}
		
		if ( mEnabledFace3d ) {
			gl::color( Colorf( 1.0f, 0.0f, 0.0f ) );
		} else {
			gl::lineWidth( 2.0f );
		}
		for ( const Kinect2::Face2d& face : mFaces2d ) {
			if ( face.isTracked() ) {
				gl::drawStrokedRect( face.getBoundsColor() );
				for ( const vec2& i : face.getPointsColor() ) {
					gl::drawSolidCircle( i, 3.0f, 16 );
				}
			}
		}
		gl::popMatrices();
	}

	mParams->draw();
}

void FaceApp::setup()
{	
	gl::enableAlphaBlending();
	
	mEnabledFace2d	= false;
	mEnabledFace3d	= false;
	mEnabledBody = false;
	mEnabledBodyAndFace2d = false;
	mFrameRate		= 0.0f;
	mFullScreen		= false;

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->enableFaceMesh();
	mDevice->connectColorEventHandler( [ & ]( const Kinect2::ColorFrame frame )
	{
		mSurface = frame.getSurface();
		colorFrameCount++;
	} );

	// Starting the device here instead, with argument, disables all but color thread 
	// who's handler is enabled before this point --> fewer threads
	//mDevice->start(true);
		
	mParams = params::InterfaceGl::create( "Params", ivec2( 230, 300 ) );
	mParams->addParam("App fps", &mFrameRate, "", true);
	mParams->addParam("Color fps", &colorFps, "", true);
	mParams->addParam("Body fps", &bodyFps, "", true);
	mParams->addParam("Face2d fps", &face2dFps, "", true);
	mParams->addParam("Face3d fps", &face3dFps, "", true);
	mParams->addParam("BodyAndFace2d fps", &bodyAndFace2dFps, "", true);
	mParams->addParam("Sum Body fps + Face2d fps", &sumBodyFaceFps, "", true);
	mParams->addParam("Full screen", &mFullScreen).key("f");
	mParams->addParam("body tracking", &mEnabledBody).key("1");
	mParams->addParam("2d face tracking", &mEnabledFace2d).key("2");
	mParams->addParam("3d face tracking", &mEnabledFace3d).key("3");
	mParams->addParam("Body and face2d tracking", &mEnabledBodyAndFace2d).key("4");
	mParams->addParam("Bodies", &mBodyCount, "", true);
	mParams->addButton("Quit", [&]() { quit(); }, "key=q");
}

void FaceApp::update()
{
	mFrameRate = getAverageFps();

	double dt = app::getElapsedSeconds() - timeLastFrame;
	timeSinceAvg += dt;
	timeLastFrame += dt;
	if (timeSinceAvg > 1.)
	{
		colorFps = colorFrameCount / timeSinceAvg;
		bodyFps = bodyFrameCount / timeSinceAvg;
		face2dFps = face2dFrameCount / timeSinceAvg;
		face3dFps = face3dFrameCount / timeSinceAvg;
		bodyAndFace2dFps = bodyAndFace2dFrameCount / timeSinceAvg;
		colorFrameCount = 0;
		bodyFrameCount = 0;
		face2dFrameCount = 0;
		face3dFrameCount = 0;
		bodyAndFace2dFrameCount = 0;

		sumBodyFaceFps = bodyFps + face2dFps;

		timeSinceAvg = 0;
	}

	
	// Toggles streams by connecting and disconnecting events

	if (mEnabledBody && !mDevice->isBodyEventHandlerConnected()) {
		mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame& frame)
		{
			mBodyCount = 0;
			for (auto body : frame.getBodies())
			{
				if (body.isTracked())
					mBodyCount++;
			}
			bodyFrameCount++;
		});
	}
	else if (!mEnabledBody && mDevice->isBodyEventHandlerConnected()) {
		mDevice->disconnectBodyEventHandler();
		mBodyCount = 0;
	}

	if ( mEnabledFace2d && !mDevice->isFace2dEventHandlerConnected() ) {
		mDevice->connectFace2dEventHandler( [ & ]( const Kinect2::Face2dFrame& frame )
		{
			if (!frame.getFaces().empty()) {
				mFaces2d = frame.getFaces();
			}
			else
				;//app::console() << "EMPTY FACE2" << endl;
			face2dFrameCount++;
		} );
	} else if ( !mEnabledFace2d && mDevice->isFace2dEventHandlerConnected() ) {
		mDevice->disconnectFace2dEventHandler();
		mFaces2d.clear();
	}

	if ( mEnabledFace3d && !mDevice->isFace3dEventHandlerConnected() ) {
		mDevice->connectFace3dEventHandler( [ & ]( const Kinect2::Face3dFrame& frame )
		{
			if (!frame.getFaces().empty()) {
				mFaces3d = frame.getFaces();
			}
			else
				;// app::console() << "EMPTY FACE3" << endl;
			face3dFrameCount++;
		} );
	} else if ( !mEnabledFace3d && mDevice->isFace3dEventHandlerConnected() ) {
		mDevice->disconnectFace3dEventHandler();
		mFaces3d.clear();
	}

	if (mEnabledBodyAndFace2d && !mDevice->isBodyAndFace2dEventHandlerConnected()) {
		mDevice->connectBodyAndFace2dEventHandler([&](const Kinect2::BodyFrame& frameBody, const Kinect2::Face2dFrame& frameFace)
		{
			if (!frameFace.getFaces().empty()) {
				mFaces2d = frameFace.getFaces();
			}

			mBodyCount = 0;
			for (auto body : frameBody.getBodies())
			{
				if (body.isTracked())
					mBodyCount++;
			}
			bodyAndFace2dFrameCount++;
		});
	}
	else if (!mEnabledBodyAndFace2d && mDevice->isBodyAndFace2dEventHandlerConnected()) {
		mDevice->disconnectBodyAndFace2dEventHandler();
		mBodyCount = 0;
	}

	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}
}

CINDER_APP( FaceApp, RendererGl, []( App::Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 960, 540 ).title( "Face App" ) );
	settings->setFrameRate( 60.0f );
} )
 