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

#include "Kinect2.h"
#include "cinder/app/App.h"

#include <comutil.h>

namespace Kinect2 {

using namespace ci;
using namespace app;
using namespace std;

Channel8u channel16To8( const Channel16u& channel, uint8_t bytes )
{
	Channel8u channel8;
	if ( channel ) {
		channel8						= Channel8u( channel.getWidth(), channel.getHeight() );
		Channel16u::ConstIter iter16	= channel.getIter();
		Channel8u::Iter iter8			= channel8.getIter();
		while ( iter8.line() && iter16.line() ) {
			while ( iter8.pixel() && iter16.pixel() ) {
				iter8.v()				= static_cast<uint8_t>( iter16.v() >> bytes );
			}
		}
	}
	return channel8;
}

Surface8u colorizeBodyIndex( const Channel8u& bodyIndexChannel )
{
	Surface8u surface;
	if ( bodyIndexChannel ) {
		surface = Surface8u( bodyIndexChannel.getWidth(), bodyIndexChannel.getHeight(), true, SurfaceChannelOrder::RGBA );
		Channel8u::ConstIter iterChannel	= bodyIndexChannel.getIter();
		Surface8u::Iter iterSurface			= surface.getIter();
		while ( iterChannel.line() && iterSurface.line() ) {
			while ( iterChannel.pixel() && iterSurface.pixel() ) {
				size_t index				= (size_t)iterChannel.v();
				ColorA8u color( getBodyColor( index ), 0xFF );
				if ( index == 0 || index > BODY_COUNT ) {
					color.a					= 0x00;
				}
				iterSurface.r()				= color.r;
				iterSurface.g()				= color.g;
				iterSurface.b()				= color.b;
				iterSurface.a()				= color.a;
			}
		}
	}
	return surface;
}

Color8u getBodyColor( size_t index )
{
	switch ( index ) {
	case 0:
		return Color8u::black();
	case 1:
		return Color8u( 0xFF, 0x00, 0x00 );
	case 2:
		return Color8u( 0x00, 0xFF, 0x00 );
	case 3:
		return Color8u( 0x00, 0x00, 0xFF );
	case 4:
		return Color8u( 0xFF, 0xFF, 0x00 );
	case 5:
		return Color8u( 0x00, 0xFF, 0xFF );
	case 6:
		return Color8u( 0xFF, 0x00, 0xFF );
	default:
		return Color8u::white();
	}
}

// FOR FUTURE USE
size_t getDeviceCount()
{
	size_t count								= 0;
	//IKinectSensorCollection* sensorCollection	= 0;
	//long hr = GetKinectSensorCollection( &sensorCollection );
	//if ( SUCCEEDED( hr ) && sensorCollection != 0 ) {
	//	IEnumKinectSensor* sensorEnum = 0;
	//	hr = sensorCollection->get_Enumerator( &sensorEnum );
	//	if ( SUCCEEDED( hr ) || sensorEnum != 0 ) {
	//		size_t i = 0;
	//		while ( SUCCEEDED( hr ) && i < 8 ) {
	//			IKinectSensor* sensor = 0;
	//			hr = sensorEnum->GetNext( &sensor );
	//			if ( sensor != 0 ) {
	//				++count;
	//			}
	//			++i;
	//		}
	//	}
	//}
	return count;
}

map<size_t, string> getDeviceMap()
{
	map<size_t, string> deviceMap;
	//IKinectSensorCollection* sensorCollection	= 0;
	//long hr = GetKinectSensorCollection( &sensorCollection );
	//if ( SUCCEEDED( hr ) && sensorCollection != 0 ) {
	//	IEnumKinectSensor* sensorEnum = 0;
	//	hr = sensorCollection->get_Enumerator( &sensorEnum );
	//	if ( SUCCEEDED( hr ) || sensorEnum != 0 ) {
	//		size_t i = 0;
	//		while ( SUCCEEDED( hr ) && i < 8 ) {
	//			IKinectSensor* sensor = 0;
	//			hr = sensorEnum->GetNext( &sensor );
	//			if ( sensor != 0 ) {
	//				wchar_t wid[ 48 ];
	//				if ( SUCCEEDED( sensor->get_UniqueKinectId( 48, wid ) ) ) {
	//					string id = wcharToString( wid );
	//					if ( !id.empty() ) {
	//						deviceMap[ i ] = string( id );
	//					}
	//				}
	//			}
	//			++i;
	//		}
	//	}
	//}
	return deviceMap;
}

CameraSpacePoint toCameraSpacePoint( const Vec3f& v )
{
	CameraSpacePoint p;
	p.X = v.x;
	p.Y = v.y;
	p.Z = v.z;
	return p;
}

ColorSpacePoint	toColorSpacePoint( const Vec2f& v )
{
	ColorSpacePoint p;
	p.X = v.x;
	p.Y = v.y;
	return p;
}

DepthSpacePoint	toDepthSpacePoint( const Vec2f& v )
{
	DepthSpacePoint p;
	p.X = v.x;
	p.Y = v.y;
	return p;
}

PointF toPointF( const Vec2f& v )
{
	PointF p;
	p.X = v.x;
	p.Y = v.y;
	return p;
}

Vector4 toVector4( const Quatf& q )
{
	Vector4 p;
	p.w = q.w;
	p.x = q.v.x;
	p.y = q.v.y;
	p.z = q.v.z;
	return p;
}

Vector4 toVector4( const Vec4f& v )
{
	Vector4 p;
	p.w = v.w;
	p.x = v.x;
	p.y = v.y;
	p.z = v.z;
	return p;
}

Quatf toQuatf( const Vector4& v )
{
	return Quatf( v.w, v.x, v.y, v.z );
}

Vec2f toVec2f( const PointF& v )
{
	return Vec2f( v.X, v.Y );
}

Vec2f toVec2f( const ColorSpacePoint& v )
{
	return Vec2f( v.X, v.Y );
}

Vec2f toVec2f( const DepthSpacePoint& v )
{
	return Vec2f( v.X, v.Y );
}

Vec3f toVec3f( const CameraSpacePoint& v )
{
	return Vec3f( v.X, v.Y, v.Z );
}

Vec4f toVec4f( const Vector4& v )
{
	return Vec4f( v.x, v.y, v.z, v.w );
}

string wcharToString( wchar_t* v )
{
	string str = "";
	wchar_t* id = ::SysAllocString( v );
	_bstr_t idStr( id );
	if ( idStr.length() > 0 ) {
		str = string( idStr );
	}
	::SysFreeString( id );
	return str;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Body::Joint::Joint()
: mOrientation( Quatf() ), mParentJoint( JointType::JointType_Count ), mPosition( Vec3f::zero() ), 
mTrackingState( TrackingState::TrackingState_NotTracked )
{
}

Body::Joint::Joint( const Vec3f& position, const Quatf& orientation, TrackingState trackingState,
	JointType parentJoint )
: mOrientation( orientation ), mPosition( position ), mParentJoint( parentJoint ), 
mTrackingState( trackingState )
{
}

JointType Body::Joint::getParentJoint() const
{
	return mParentJoint;
}

const Vec3f& Body::Joint::getPosition() const
{
	return mPosition;
}

const Quatf& Body::Joint::getOrientation() const
{
	return mOrientation;
}

TrackingState Body::Joint::getTrackingState() const
{
	return mTrackingState;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Body::Body()
: mId( 0 ), mIndex( 0 ), mTracked( false )
{
}

Body::Body( uint64_t id, uint8_t index, const map<JointType, Body::Joint>& jointMap )
: mId( id ), mIndex( index ), mJointMap( jointMap ), mTracked( true )
{
}

float Body::calcConfidence( bool weighted ) const
{
	float c = 0.0f;
	if ( weighted ) {
		static map<JointType, float> weights;
		if ( weights.empty() ) {
			weights[ JointType::JointType_SpineBase ]		= 0.042553191f;
			weights[ JointType::JointType_SpineMid ]		= 0.042553191f;
			weights[ JointType::JointType_Neck ]			= 0.021276596f;
			weights[ JointType::JointType_Head ]			= 0.042553191f;
			weights[ JointType::JointType_ShoulderLeft ]	= 0.021276596f;
			weights[ JointType::JointType_ElbowLeft ]		= 0.010638298f;
			weights[ JointType::JointType_WristLeft ]		= 0.005319149f;
			weights[ JointType::JointType_HandLeft ]		= 0.042553191f;
			weights[ JointType::JointType_ShoulderRight ]	= 0.021276596f;
			weights[ JointType::JointType_ElbowRight ]		= 0.010638298f;
			weights[ JointType::JointType_WristRight ]		= 0.005319149f;
			weights[ JointType::JointType_HandRight ]		= 0.042553191f;
			weights[ JointType::JointType_HipLeft ]			= 0.021276596f;
			weights[ JointType::JointType_KneeLeft ]		= 0.010638298f;
			weights[ JointType::JointType_AnkleLeft ]		= 0.005319149f;
			weights[ JointType::JointType_FootLeft ]		= 0.042553191f;
			weights[ JointType::JointType_HipRight ]		= 0.021276596f;
			weights[ JointType::JointType_KneeRight ]		= 0.010638298f;
			weights[ JointType::JointType_AnkleRight ]		= 0.005319149f;
			weights[ JointType::JointType_FootRight ]		= 0.042553191f;
			weights[ JointType::JointType_SpineShoulder ]	= 0.002659574f;
			weights[ JointType::JointType_HandTipLeft ]		= 0.002659574f;
			weights[ JointType::JointType_ThumbLeft ]		= 0.002659574f;
			weights[ JointType::JointType_HandTipRight ]	= 0.002659574f;
			weights[ JointType::JointType_ThumbRight ]		= 0.521276596f;
		}
		for ( map<JointType, Body::Joint>::const_iterator iter = mJointMap.begin(); iter != mJointMap.end(); ++iter ) {
			if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
				c += weights[ iter->first ];
			}
		}
	} else {
		for ( map<JointType, Body::Joint>::const_iterator iter = mJointMap.begin(); iter != mJointMap.end(); ++iter ) {
			if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
				c += 1.0f;
			}
		}
		c /= (float)JointType::JointType_Count;
	}
	return c;
}

uint64_t Body::getId() const 
{ 
	return mId; 
}

uint8_t Body::getIndex() const 
{ 
	return mIndex; 
}

const map<JointType, Body::Joint>& Body::getJointMap() const 
{ 
	return mJointMap; 
}

bool Body::isTracked() const 
{ 
	return mTracked; 
}

//////////////////////////////////////////////////////////////////////////////////////////////

Face::Face()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////

Frame::Frame()
: mTimeStamp( 0L )
{
}

long long Frame::getTimeStamp() const
{
	return mTimeStamp;
}

//////////////////////////////////////////////////////////////////////////////////////////////

CameraFrame::CameraFrame()
: mFovDiagonal( 0.0f ), mFovHorizontal( 0.0f ), 
mFovVertical( 0.0f ), mSize( Vec2i::zero() )
{
}

float CameraFrame::getFovDiagonal() const
{
	return mFovDiagonal;
}

float CameraFrame::getFovHorizontal() const
{
	return mFovHorizontal;
}

float CameraFrame::getFovVertical() const
{
	return mFovVertical;
}

const Vec2i& CameraFrame::getSize() const
{
	return mSize;
}

//////////////////////////////////////////////////////////////////////////////////////////////

AudioFrame::AudioFrame()
	: mBeamAngle( 0.0f ), mBeamAngleConfidence( 0.0f ), mBuffer( nullptr ), 
	mBufferSize( 0 )
{
}

AudioFrame::~AudioFrame()
{
	if ( mBuffer != nullptr ) {
		delete [] mBuffer;
		mBuffer = nullptr;
	}
}

float AudioFrame::getBeamAngle() const
{
	return mBeamAngle;
}

float AudioFrame::getBeamAngleConfidence() const
{
	return mBeamAngleConfidence;
}

uint8_t* AudioFrame::getBuffer() const
{
	return mBuffer;
}

unsigned long AudioFrame::getBufferSize() const
{
	return mBufferSize;
}

WAVEFORMATEX AudioFrame::getFormat() const
{
	return mFormat;
}

//////////////////////////////////////////////////////////////////////////////////////////////

template<typename T> 
ChannelFrameT<T>::ChannelFrameT()
: Frame()
{
}

template<typename T> 
const ChannelT<T>& ChannelFrameT<T>::getChannel() const
{
	return mChannel;
}

template class ChannelFrameT<uint8_t>;
template class ChannelFrameT<uint16_t>;

//////////////////////////////////////////////////////////////////////////////////////////////

ColorFrame::ColorFrame()
: CameraFrame(), Frame()
{
}

const Surface8u& ColorFrame::getSurface() const
{
	return mSurface;
}

//////////////////////////////////////////////////////////////////////////////////////////////

DepthFrame::DepthFrame()
: CameraFrame(), ChannelFrame16u()
{}

//////////////////////////////////////////////////////////////////////////////////////////////

BodyFrame::BodyFrame()
: Frame()
{
}

const vector<Body>& BodyFrame::getBodies() const
{
	return mBodies;
}

//////////////////////////////////////////////////////////////////////////////////////////////

FaceFrame::FaceFrame()
: Frame()
{
}

const vector<Face>& FaceFrame::getFaces() const
{
	return mFaces;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Device::Process::Process()
: mNewData( atomic<bool>( false ) ), mRunning( atomic<bool>( false ) ), 
mThreadCallback( nullptr )
{
}

Device::Process::~Process()
{
	stop();
}

void Device::Process::start()
{
	stop();
	if ( mThreadCallback != nullptr ) {
		mNewData	= false;
		mRunning	= true;
		mThread		= shared_ptr<thread>( new thread( mThreadCallback ) );
	}
}

void Device::Process::stop()
{
	mRunning = false;
	if ( mThread ) {
		mThread->join();
		mThread.reset();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceRef Device::create()
{
	return DeviceRef( new Device() );
}

Device::Device()
	: mEventHandlerAudio( nullptr ), mEventHandlerBody( nullptr ), 
	mEventHandlerBodyIndex( nullptr ), mEventHandlerColor( nullptr ), 
	mEventHandlerDepth( nullptr ), mEventHandlerFace( nullptr ), 
	mEventHandlerInfrared( nullptr ), mEventHandlerInfraredLongExposure( nullptr ), 
	mKinect(KCB_INVALID_HANDLE)
{
	App::get()->getSignalUpdate().connect( bind( &Device::update, this ) );
}

Device::~Device()
{
	stop();
}

void Device::connectAudioEventHandler( const function<void ( const AudioFrame& )>& eventHandler )
{
	mEventHandlerAudio = eventHandler;
}

void Device::connectBodyEventHandler( const function<void ( const BodyFrame& )>& eventHandler )
{
	mEventHandlerBody = eventHandler;
}

void Device::connectBodyIndexEventHandler( const function<void ( const BodyIndexFrame& )>& eventHandler )
{
	mEventHandlerBodyIndex = eventHandler;
}

void Device::connectColorEventHandler( const function<void ( const ColorFrame& )>& eventHandler )
{
	mEventHandlerColor = eventHandler;
}

void Device::connectDepthEventHandler( const function<void ( const DepthFrame& )>& eventHandler )
{
	mEventHandlerDepth = eventHandler;
}

void Device::connectFaceEventHandler( const function<void ( const FaceFrame& )>& eventHandler )
{
	mEventHandlerFace = eventHandler;
}

void Device::connectInfraredEventHandler( const function<void ( const InfraredFrame& )>& eventHandler )
{
	mEventHandlerInfrared = eventHandler;
}

void Device::connectInfraredLongExposureEventHandler( const function<void ( const InfraredLongExposureFrame& )>& eventHandler )
{
	mEventHandlerInfraredLongExposure = eventHandler;
}

void Device::disconnectAudioEventHandler()
{
	mEventHandlerAudio = nullptr;
}

void Device::disconnectBodyEventHandler()
{
	mEventHandlerBody = nullptr;
}

void Device::disconnectBodyIndexEventHandler()
{
	mEventHandlerBodyIndex = nullptr;
}

void Device::disconnectColorEventHandler()
{
	mEventHandlerColor = nullptr;
}

void Device::disconnectDepthEventHandler()
{
	mEventHandlerDepth = nullptr;
}

void Device::disconnectFaceEventHandler()
{
	mEventHandlerFace = nullptr;
}

void Device::disconnectInfraredEventHandler()
{
	mEventHandlerInfrared = nullptr;
}

void Device::disconnectInfraredLongExposureEventHandler()
{
	mEventHandlerInfraredLongExposure = nullptr;
}

Vec2i Device::mapCameraToColor( const Vec3f& v ) const
{
	ColorSpacePoint p;
	KCBMapCameraPointToColorSpace( mKinect, toCameraSpacePoint( v ), &p ); 
	return Vec2i( toVec2f( p ) );
}

vector<Vec2i> Device::mapCameraToColor( const vector<Vec3f>& v ) const
{
	vector<Vec2i> p;
	vector<CameraSpacePoint> camera;
	vector<ColorSpacePoint> color;
	for_each( v.begin(), v.end(), [ &camera ]( const Vec3f& i )
	{
		camera.push_back( toCameraSpacePoint( i ) );
	} );
	KCBMapCameraPointsToColorSpace( mKinect, camera.size(), &camera[ 0 ], color.size(), &color[ 0 ] );
	for_each( color.begin(), color.end(), [ &p ]( const ColorSpacePoint& i )
	{
		p.push_back( Vec2i( toVec2f( i ) ) );
	} );
	return p;
}

Vec2i Device::mapCameraToDepth( const Vec3f& v ) const
{
	DepthSpacePoint p;
	KCBMapCameraPointToDepthSpace( mKinect, toCameraSpacePoint( v ), &p ); 
	return Vec2i( toVec2f( p ) );
}

vector<Vec2i> Device::mapCameraToDepth( const vector<Vec3f>& v ) const
{
	vector<Vec2i> p;
	vector<CameraSpacePoint> camera;
	vector<DepthSpacePoint> depth( v.size() );
	for_each( v.begin(), v.end(), [ &camera ]( const Vec3f& i )
	{
		camera.push_back( toCameraSpacePoint( i ) );
	} );
	KCBMapCameraPointsToDepthSpace( mKinect, camera.size(), &camera[ 0 ], depth.size(), &depth[ 0 ] );
	for_each( depth.begin(), depth.end(), [ &p ]( const DepthSpacePoint& i )
	{
		p.push_back( Vec2i( toVec2f( i ) ) );
	} );
	return p;
}

Vec3f Device::mapDepthToCamera( const Vec2i& v, const Channel16u& depth ) const
{
	CameraSpacePoint p;
	if ( depth ) {
		uint16_t d = depth.getValue( v );
		KCBMapDepthPointToCameraSpace( mKinect, toDepthSpacePoint( v ), d, &p ); 
	}
	return toVec3f( p );
}

vector<Vec3f> Device::mapDepthToCamera( const vector<Vec2i>& v, const Channel16u& depth ) const
{
	vector<Vec3f> p;
	if ( depth ) {
		vector<CameraSpacePoint> camera( v.size() );
		vector<DepthSpacePoint> depthPos;
		vector<uint16_t> depthVal;
		for_each( v.begin(), v.end(), [ &depth, &depthPos, &depthVal ]( const Vec2i& i )
		{
			depthPos.push_back( toDepthSpacePoint( i ) );
			depthVal.push_back( depth.getValue( i ) );
		} );
		KCBMapDepthPointsToCameraSpace( mKinect, depthPos.size(), &depthPos[ 0 ], depthPos.size(), &depthVal[ 0 ], camera.size(), &camera[ 0 ] );
		for_each( camera.begin(), camera.end(), [ &p ]( const CameraSpacePoint& i )
		{
			p.push_back( toVec3f( i ) );
		} );
	}
	return p;
}

vector<Vec3f> Device::mapDepthToCamera( const Channel16u& depth ) const
{
	vector<Vec3f> p;
	if ( depth ) {
		vector<CameraSpacePoint> camera( depth.getWidth() * depth.getHeight() );
		KCBMapDepthFrameToCameraSpace( mKinect, camera.size(), depth.getData(), camera.size(), &camera[ 0 ] );
		for_each( camera.begin(), camera.end(), [ &p ]( const CameraSpacePoint& i )
		{
			p.push_back( toVec3f( i ) );
		} );
	}
	return p;
}

Vec2i Device::mapDepthToColor( const Vec2i& v, const Channel16u& depth ) const
{
	ColorSpacePoint p;
	if ( depth ) {
		uint16_t d = depth.getValue( v );
		KCBMapDepthPointToColorSpace( mKinect, toDepthSpacePoint( v ), d, &p ); 
	}
	return Vec2i( toVec2f( p ) );
}

vector<Vec2i> Device::mapDepthToColor( const Channel16u& depth ) const
{
	vector<Vec2i> p;
	if ( depth ) {
		vector<ColorSpacePoint> color( depth.getWidth() * depth.getHeight() );
		KCBMapDepthFrameToColorSpace( mKinect, color.size(), depth.getData(), color.size(), &color[ 0 ] );
		for_each( color.begin(), color.end(), [ &p ]( const ColorSpacePoint& i )
		{
			p.push_back( Vec2i( toVec2f( i ) ) );
		} );
	}
	return p;
}

vector<Vec2i> Device::mapDepthToColor( const vector<Vec2i>& v, const Channel16u& depth ) const
{
	vector<Vec2i> p;
	if ( depth ) {
		vector<ColorSpacePoint> color( v.size() );
		vector<DepthSpacePoint> depthPos;
		vector<uint16_t> depthVal;
		for_each( v.begin(), v.end(), [ &depth, &depthPos, &depthVal ]( const Vec2i& i )
		{
			depthPos.push_back( toDepthSpacePoint( i ) );
			depthVal.push_back( depth.getValue( i ) );
		} );
		KCBMapDepthPointsToColorSpace( mKinect, depthPos.size(), &depthPos[ 0 ], depthPos.size(), &depthVal[ 0 ], color.size(), &color[ 0 ] );
		for_each( color.begin(), color.end(), [ &p ]( const ColorSpacePoint& i )
		{
			p.push_back( Vec2i( toVec2f( i ) ) );
		} );
	}
	return p;
}

void Device::start()
{
	long hr = S_OK;
	
	/*IKinectSensorCollection* sensorCollection = nullptr;
	IKinectSensorCollection
	hr = GetKinectSensorCollection( &sensorCollection );
	if ( FAILED( hr ) || sensorCollection == 0 ) {
		throw ExcDeviceNotAvailable( hr );
	}
	if ( sensorCollection != nullptr ) {
		sensorCollection->Release();
		sensorCollection = nullptr;
	}*/

	mKinect = KCBOpenDefaultSensor();
	if ( mKinect == KCB_INVALID_HANDLE ) {
		throw ExcDeviceOpenFailed();
	}

	for ( size_t i = (size_t)FrameType_Audio; i < (size_t)FrameType_InfraredLongExposure; ++i ) {
		FrameType frameType		= (FrameType)i;
		mProcesses[ frameType ] = Process();
		Process& process		= mProcesses.at( frameType );
		switch( frameType ) {
		case FrameType_Audio:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerAudio != nullptr && KCBIsFrameReady( mKinect, FrameSourceTypes_Audio ) ) {
						AudioFrame frame;
						WAVEFORMATEX format;
						long hr = KCBGetAudioFormat( mKinect, &format );
						if ( SUCCEEDED( hr ) ) {
							KCBAudioFrame* audioFrame		= new KCBAudioFrame();
							audioFrame->cAudioBufferSize	= 4;
							audioFrame->pAudioBuffer		= new uint8_t[ audioFrame->cAudioBufferSize * format.nBlockAlign ];
							hr = KCBGetAudioFrame( mKinect, audioFrame );
							if ( SUCCEEDED( hr ) ) {
								frame.mBeamAngle			= audioFrame->fBeamAngle;
								frame.mBeamAngleConfidence	= audioFrame->fBeamAngleConfidence;
								frame.mBufferSize			= audioFrame->ulBytesRead;
								if ( audioFrame->ulBytesRead > 0 ) {
									frame.mBuffer			= new uint8_t[ frame.mBufferSize ];
									memcpy( frame.mBuffer, audioFrame->pAudioBuffer, frame.mBufferSize );
								}
							}
							delete [] audioFrame->pAudioBuffer;
							delete audioFrame;
						}

						if ( frame.getTimeStamp() > mFrameAudio.getTimeStamp() ) {
							mFrameAudio			= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_Body:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerBody != nullptr && KCBIsFrameReady(mKinect, FrameSourceTypes_Body ) ) {
						BodyFrame frame;
						int64_t timeStamp					= 0L;
						IBody* kinectBodies[ BODY_COUNT ]	= { 0 };
		
						long hr = KCBGetBodyData( mKinect, BODY_COUNT, kinectBodies, &timeStamp );
						if ( SUCCEEDED( hr ) ) {
							for ( uint8_t i = 0; i < 6; ++i ) {
								IBody* kinectBody = kinectBodies[ i ];
								if ( kinectBody != nullptr ) {
									uint8_t isTracked	= false;
									hr					= kinectBody->get_IsTracked( &isTracked );
									if ( SUCCEEDED( hr ) && isTracked ) {
										Joint joints[ JointType_Count ];
										kinectBody->GetJoints( JointType_Count, joints );

										JointOrientation jointOrientations[ JointType_Count ];
										kinectBody->GetJointOrientations( JointType_Count, jointOrientations );

										uint64_t id = 0;
										kinectBody->get_TrackingId( &id );

										map<JointType, Body::Joint> jointMap;
										for ( int32_t j = 0; j < JointType_Count; ++j ) {
											JointType parentJoint = (JointType)j;
											switch ( (JointType)j ) {
											case JointType::JointType_AnkleLeft:
												parentJoint = JointType_KneeLeft;
												break;
											case JointType::JointType_AnkleRight:
												parentJoint = JointType_KneeRight;
												break;
											case JointType::JointType_ElbowLeft:
												parentJoint = JointType_ShoulderLeft;
												break;
											case JointType::JointType_ElbowRight:
												parentJoint = JointType_ShoulderRight;
												break;
											case JointType::JointType_FootLeft:
												parentJoint = JointType_AnkleLeft;
												break;
											case JointType::JointType_FootRight:
												parentJoint = JointType_AnkleRight;
												break;
											case JointType::JointType_HandLeft:
												parentJoint = JointType_WristLeft;
												break;
											case JointType::JointType_HandRight:
												parentJoint = JointType_WristRight;
												break;
											case JointType::JointType_HandTipLeft:
												parentJoint = JointType_HandLeft;
												break;
											case JointType::JointType_HandTipRight:
												parentJoint = JointType_HandRight;
												break;
											case JointType::JointType_Head:
												parentJoint = JointType_Neck;
												break;
											case JointType::JointType_HipLeft:
												parentJoint = JointType_SpineBase;
												break;
											case JointType::JointType_HipRight:
												parentJoint = JointType_SpineBase;
												break;
											case JointType::JointType_KneeLeft:
												parentJoint = JointType_HipLeft;
												break;
											case JointType::JointType_KneeRight:
												parentJoint = JointType_HipRight;
												break;
											case JointType::JointType_Neck:
												parentJoint = JointType_SpineShoulder;
												break;
											case JointType::JointType_ShoulderLeft:
												parentJoint = JointType_SpineShoulder;
												break;
											case JointType::JointType_ShoulderRight:
												parentJoint = JointType_SpineShoulder;
												break;
											case JointType::JointType_SpineBase:
												parentJoint = JointType_SpineBase;
												break;
											case JointType::JointType_SpineMid:
												parentJoint = JointType_SpineBase;
												break;
											case JointType::JointType_SpineShoulder:
												parentJoint = JointType_SpineMid;
												break;
											case JointType::JointType_ThumbLeft:
												parentJoint = JointType_HandLeft;
												break;
											case JointType::JointType_ThumbRight:
												parentJoint = JointType_HandRight;
												break;
											case JointType::JointType_WristLeft:
												parentJoint = JointType_ElbowLeft;
												break;
											case JointType::JointType_WristRight:
												parentJoint = JointType_ElbowRight;
												break;
											}

											Body::Joint joint( 
												toVec3f( joints[ j ].Position ), 
												toQuatf( jointOrientations[ j ].Orientation ), 
												joints[ j ].TrackingState, 
												parentJoint
												);
											jointMap.insert( pair<JointType, Body::Joint>( static_cast<JointType>( j ), joint ) );
										}
										Body body( id, i, jointMap );
										frame.mBodies.push_back( body );
									}
									kinectBody->Release();
									kinectBody = nullptr;
								}
							}
							frame.mTimeStamp	= static_cast<long long>( timeStamp );
						}
						if ( frame.getTimeStamp() > mFrameBody.getTimeStamp() ) {
							mFrameBody			= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_BodyIndex:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerBodyIndex != nullptr && KCBIsFrameReady( mKinect, FrameSourceTypes_BodyIndex ) ) {
						BodyIndexFrame frame;
						KCBFrameDescription frameDescription;
						int64_t timeStamp = 0L;
		
						long hr = KCBGetBodyIndexFrameDescription( mKinect, &frameDescription );
						if ( SUCCEEDED( hr ) ) {
							IBodyIndexFrame* bodyIndexFrame = nullptr;
			
							hr = KCBGetIBodyIndexFrame( mKinect, &bodyIndexFrame );
							if ( SUCCEEDED( hr ) ) {
								hr = bodyIndexFrame->get_RelativeTime( &timeStamp );
								if ( SUCCEEDED( hr ) ) {
									frame.mTimeStamp	= static_cast<long long>( timeStamp );
									int32_t h			= frameDescription.height;
									int32_t w			= frameDescription.width;
									frame.mChannel		= Channel8u( w, h );
									uint32_t capacity	= (uint32_t)( w * h );
									uint8_t* buffer		= frame.mChannel.getData();
									bodyIndexFrame->CopyFrameDataToArray( capacity, buffer );
								}
							}
							if ( bodyIndexFrame != nullptr ) {
								bodyIndexFrame->Release();
								bodyIndexFrame = nullptr;
							}
						}

						if ( frame.getTimeStamp() > mFrameBodyIndex.getTimeStamp() ) {
							mFrameBodyIndex		= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_Color:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerColor != nullptr && KCBIsFrameReady( mKinect, FrameSourceTypes_Color ) ) {
						ColorFrame frame;
						KCBFrameDescription frameDescription;
						int64_t timeStamp	= 0L;
		
						long hr = KCBGetColorFrameDescription( mKinect, ColorImageFormat_Bgra, &frameDescription );
						if ( SUCCEEDED( hr ) ) {
							IColorFrame* colorFrame = nullptr;
			
							frame.mFovDiagonal		= frameDescription.diagonalFieldOfView;
							frame.mFovHorizontal	= frameDescription.horizontalFieldOfView;
							frame.mFovVertical		= frameDescription.verticalFieldOfView;
							frame.mSize				= Vec2i( frameDescription.width, frameDescription.height );
							hr = KCBGetIColorFrame( mKinect, &colorFrame );
							if ( SUCCEEDED( hr ) ) {
								hr = colorFrame->get_RelativeTime( &timeStamp );
								if ( SUCCEEDED( hr ) ) {
									frame.mTimeStamp	= static_cast<long long>( timeStamp );
									frame.mSurface		= Surface8u( frame.getSize().x, frame.getSize().y, false, SurfaceChannelOrder::BGRA );
									uint32_t capacity	= frame.getSize().x * frame.getSize().y * frameDescription.bytesPerPixel;
									uint8_t* buffer		= frame.mSurface.getData();
									colorFrame->CopyConvertedFrameDataToArray( capacity, buffer, ColorImageFormat_Bgra );
								}
							}
							if (colorFrame != nullptr) {
								colorFrame->Release();
								colorFrame = nullptr;
							}
						}

						if ( frame.getTimeStamp() > mFrameColor.getTimeStamp() ) {
							mFrameColor			= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_Depth:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerDepth != nullptr && KCBIsFrameReady( mKinect, FrameSourceTypes_Depth ) ) {
						DepthFrame frame;
						KCBFrameDescription frameDescription;
						int64_t timeStamp = 0L;
		
						long hr = KCBGetDepthFrameDescription( mKinect, &frameDescription );
						if ( SUCCEEDED( hr ) ) {
							IDepthFrame* depthFrame = nullptr;

							frame.mFovDiagonal		= frameDescription.diagonalFieldOfView;
							frame.mFovHorizontal	= frameDescription.horizontalFieldOfView;
							frame.mFovVertical		= frameDescription.verticalFieldOfView;
							frame.mSize				= Vec2i( frameDescription.width, frameDescription.height );
							hr = KCBGetIDepthFrame( mKinect, &depthFrame );
							if ( SUCCEEDED( hr ) ) {
								hr = depthFrame->get_RelativeTime( &timeStamp );
								if ( SUCCEEDED( hr ) ) {
									frame.mTimeStamp	= static_cast<long long>( timeStamp );
									frame.mChannel		= Channel16u( frame.getSize().x, frame.getSize().y );
									uint32_t capacity	= frame.getSize().x * frame.getSize().y;
									uint16_t* buffer	= frame.mChannel.getData();
									depthFrame->CopyFrameDataToArray( capacity, buffer );
								}
							}
							if ( depthFrame != nullptr ) {
								depthFrame->Release();
								depthFrame = nullptr;
							}
						}

						if ( frame.getTimeStamp() > mFrameDepth.getTimeStamp() ) {
							mFrameDepth			= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_Face:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerFace != nullptr ) {
						FaceFrame frame;

						// TODO get faces
						
						if ( frame.getTimeStamp() > mFrameFace.getTimeStamp() ) {
							mFrameFace			= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_Infrared:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerInfrared != nullptr && KCBIsFrameReady( mKinect, FrameSourceTypes_Infrared ) ) {
						InfraredFrame frame;
						KCBFrameDescription frameDescription;
						int64_t timeStamp	= 0L;
		
						long hr = KCBGetInfraredFrameDescription( mKinect, &frameDescription );
						if ( SUCCEEDED( hr ) ) {
							IInfraredFrame* infraredFrame = nullptr;

							hr = KCBGetIInfraredFrame( mKinect, &infraredFrame );
							if ( SUCCEEDED( hr ) ) {
								hr = infraredFrame->get_RelativeTime( &timeStamp );
								if ( SUCCEEDED( hr ) ) {
									frame.mTimeStamp	= static_cast<long long>( timeStamp );
									int32_t h			= frameDescription.height;
									int32_t w			= frameDescription.width;
									frame.mChannel		= Channel16u( w, h );
									uint32_t capacity	= (uint32_t)( w * h );
									uint16_t* buffer	= frame.mChannel.getData();
									infraredFrame->CopyFrameDataToArray( capacity, buffer );
								}
							}
							if ( infraredFrame != nullptr ) {
								infraredFrame->Release();
								infraredFrame = nullptr;
							}
						}

						if ( frame.getTimeStamp() > mFrameInfrared.getTimeStamp() ) {
							mFrameInfrared		= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_InfraredLongExposure:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE ) {
						std::this_thread::yield();
						continue;
					}

					if ( mEventHandlerInfraredLongExposure != nullptr && KCBIsFrameReady( mKinect, FrameSourceTypes_LongExposureInfrared ) ) {
						InfraredLongExposureFrame frame;
						KCBFrameDescription frameDescription;
						Vec2i sz			= Vec2i::zero();
						int64_t timeStamp	= 0L;
		
						long hr = KCBGetInfraredFrameDescription( mKinect, &frameDescription );
						if ( SUCCEEDED( hr ) ) {
							ILongExposureInfraredFrame* infraredLongExposureFrame = nullptr;

							hr = KCBGetILongExposureInfraredFrame( mKinect, &infraredLongExposureFrame );
							if ( SUCCEEDED( hr ) ) {
								hr = infraredLongExposureFrame->get_RelativeTime( &timeStamp );
								if ( SUCCEEDED( hr ) ) {
									frame.mTimeStamp	= static_cast<long long>( timeStamp );
									int32_t h			= frameDescription.height;
									int32_t w			= frameDescription.width;
									frame.mChannel		= Channel16u( w, h );
									uint32_t capacity	= (uint32_t)( w * h );
									uint16_t* buffer	= frame.mChannel.getData();
									infraredLongExposureFrame->CopyFrameDataToArray( capacity, buffer );
								}
							}
							if ( infraredLongExposureFrame != nullptr ) {
								infraredLongExposureFrame->Release();
								infraredLongExposureFrame = nullptr;
							}
						}

						if ( frame.getTimeStamp() > mFrameInfraredLongExposure.getTimeStamp() ) {
							mFrameInfraredLongExposure	= frame;
							process.mNewData			= true;
						}
					}
				}
			};
			break;
		}
		process.start();
	}
}

void Device::stop()
{
	if ( mKinect != KCB_INVALID_HANDLE ) {
		long hr = KCBCloseSensor( &mKinect );
		if ( FAILED( hr ) ) {
			throw ExcDeviceCloseFailed( hr );
		} else {
			mKinect = KCB_INVALID_HANDLE;
		}
	}

	for ( size_t i = (size_t)FrameType_Audio; i < (size_t)FrameType_InfraredLongExposure; ++i ) {
		mProcesses.at( (FrameType)i ).stop();
	}
}

void Device::update()
{
	for ( size_t i = (size_t)FrameType_Audio; i < (size_t)FrameType_InfraredLongExposure; ++i ) {
		FrameType frameType	= (FrameType)i;
		Process& process	= mProcesses.at( frameType );
		switch( frameType ) {
		case FrameType_Audio:
			if ( mEventHandlerAudio != nullptr && process.mNewData ) {
				mEventHandlerAudio( mFrameAudio );
				process.mNewData = false;
			}
			break;
		case FrameType_Body:
			if ( mEventHandlerBody != nullptr && process.mNewData ) {
				mEventHandlerBody( mFrameBody );
				process.mNewData = false;
			}
			break;
		case FrameType_BodyIndex:
			if ( mEventHandlerBodyIndex != nullptr && process.mNewData ) {
				mEventHandlerBodyIndex( mFrameBodyIndex );
				process.mNewData = false;
			}
			break;
		case FrameType_Color:
			if ( mEventHandlerColor != nullptr && process.mNewData ) {
				mEventHandlerColor( mFrameColor );
				process.mNewData = false;
			}
			break;
		case FrameType_Depth:
			if ( mEventHandlerDepth != nullptr && process.mNewData ) {
				mEventHandlerDepth( mFrameDepth );
				process.mNewData = false;
			}
			break;
		case FrameType_Face:
			if ( mEventHandlerFace != nullptr && process.mNewData ) {
				mEventHandlerFace( mFrameFace );
				process.mNewData = false;
			}
			break;
		case FrameType_Infrared:
			if ( mEventHandlerInfrared != nullptr && process.mNewData ) {
				mEventHandlerInfrared( mFrameInfrared );
				process.mNewData = false;
			}
			break;
		case FrameType_InfraredLongExposure:
			if ( mEventHandlerInfraredLongExposure != nullptr && process.mNewData ) {
				mEventHandlerInfraredLongExposure( mFrameInfraredLongExposure );
				process.mNewData = false;
			}
			break;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

const char* Device::Exception::what() const throw()
{
	return mMessage;
}

Device::ExcDeviceCloseFailed::ExcDeviceCloseFailed( long hr ) throw()
{
	sprintf( mMessage, "Unable to close device. Error: %i", hr );
}

Device::ExcDeviceNotAvailable::ExcDeviceNotAvailable( long hr ) throw()
{
	sprintf( mMessage, "No devices are available. Error: %i", hr );
}

Device::ExcDeviceOpenFailed::ExcDeviceOpenFailed() throw()
{
	sprintf( mMessage, "Unable to open device." );
}

}
 