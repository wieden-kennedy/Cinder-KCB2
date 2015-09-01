#include "cinder/app/App.h"
#include "cinder/gl/gl.h"

#include "Kinect2.h"

class _TBOX_PREFIX_App : public ci::app::App
{
public:
	_TBOX_PREFIX_App();

	void draw() override;
private:
	ci::Channel16uRef mChannel;
	Kinect2::DeviceRef mDevice;
};

#include "cinder/app/RendererGl.h"

_TBOX_PREFIX_App::_TBOX_PREFIX_App()
{	
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectDepthEventHandler( [ & ]( const Kinect2::DepthFrame& frame )
	{
		mChannel = frame.getChannel();
	} );
}

void _TBOX_PREFIX_App::draw()
{
	if ( mChannel ) {
		const ci::gl::TextureRef tex = ci::gl::Texture::create( *Kinect2::channel16To8( mChannel ) );
		ci::gl::draw( tex, tex->getBounds(), getWindowBounds() );
	}
}

CINDER_APP( _TBOX_PREFIX_App, ci::app::RendererGl )
	