#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"

#include "Kinect2.h"

class _TBOX_PREFIX_App : public ci::app::AppBasic 
{
public:
	void draw();
	void setup();
private:
	ci::Channel16u mChannel;
	Kinect2::DeviceRef mDevice;
};

void _TBOX_PREFIX_App::draw()
{
	if ( mChannel ) {
		ci::gl::TextureRef tex = ci::gl::Texture::create( Kinect2::channel16To8( mChannel ) );
		ci::gl::draw( tex, tex->getBounds(), getWindowBounds() );
	}
}

void _TBOX_PREFIX_App::setup()
{	
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectDepthEventHandler( [ & ]( const Kinect2::DepthFrame& frame )
	{
		mChannel = frame.getChannel();
	} );
}

CINDER_APP_BASIC( _TBOX_PREFIX_App, ci::app::RendererGl )
	