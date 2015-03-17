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
	bool							mEnabledFace2d;
	bool							mEnabledFace3d;
	std::vector<Kinect2::Face2d>	mFaces2d;
	std::vector<Kinect2::Face3d>	mFaces3d;
	ci::Surface8uRef				mSurface;

	float							mFrameRate;
	bool							mFullScreen;
	ci::params::InterfaceGlRef		mParams;
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
	
	mEnabledFace2d	= true;
	mEnabledFace3d	= true;
	mFrameRate		= 0.0f;
	mFullScreen		= false;

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->enableFaceMesh();
	mDevice->connectBodyEventHandler( [ & ]( const Kinect2::BodyFrame frame )
	{
	} );
	mDevice->connectColorEventHandler( [ & ]( const Kinect2::ColorFrame frame )
	{
		mSurface = frame.getSurface();
	} );
		
	mParams = params::InterfaceGl::create( "Params", ivec2( 230, 130 ) );
	mParams->addParam( "Frame rate",		&mFrameRate,			"", true );
	mParams->addParam( "Full screen",		&mFullScreen ).key( "f" );
	mParams->addParam( "2d face tracking",	&mEnabledFace2d ).key( "2" );
	mParams->addParam( "3d face tracking",	&mEnabledFace3d ).key( "3" );
	mParams->addButton( "Quit",				[ & ]() { quit(); } ,	"key=q" );
}

void FaceApp::update()
{
	mFrameRate = getAverageFps();
	
	// Toggles streams by connecting and disconnecting events
	if ( mEnabledFace2d && !mDevice->isFace2dEventHandlerConnected() ) {
		mDevice->connectFace2dEventHandler( [ & ]( const Kinect2::Face2dFrame& frame )
		{
			if ( !frame.getFaces().empty() ) {
				mFaces2d = frame.getFaces();
			}
		} );
	} else if ( !mEnabledFace2d && mDevice->isFace2dEventHandlerConnected() ) {
		mDevice->disconnectFace2dEventHandler();
		mFaces2d.clear();
	}

	if ( mEnabledFace3d && !mDevice->isFace3dEventHandlerConnected() ) {
		mDevice->connectFace3dEventHandler( [ & ]( const Kinect2::Face3dFrame& frame )
		{
			if ( !frame.getFaces().empty() ) {
				mFaces3d = frame.getFaces();
			}
		} );
	} else if ( !mEnabledFace3d && mDevice->isFace3dEventHandlerConnected() ) {
		mDevice->disconnectFace3dEventHandler();
		mFaces3d.clear();
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
 