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

Channel8u channel16To8( const Channel16u& channel )
{
	Channel8u channel8;
	if ( channel ) {
		channel8						= Channel8u( channel.getWidth(), channel.getHeight() );
		Channel16u::ConstIter iter16	= channel.getIter();
		Channel8u::Iter iter8			= channel8.getIter();
		while ( iter8.line() && iter16.line() ) {
			while ( iter8.pixel() && iter16.pixel() ) {
				iter8.v()				= iter16.v() >> 4;
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

size_t getDeviceCount()
{
	size_t count								= 0;
	IKinectSensorCollection* sensorCollection	= 0;
	long hr = GetKinectSensorCollection( &sensorCollection );
	if ( SUCCEEDED( hr ) && sensorCollection != 0 ) {
		IEnumKinectSensor* sensorEnum = 0;
		hr = sensorCollection->get_Enumerator( &sensorEnum );
		if ( SUCCEEDED( hr ) || sensorEnum != 0 ) {
			size_t i = 0;
			while ( SUCCEEDED( hr ) && i < 8 ) {
				IKinectSensor* sensor = 0;
				hr = sensorEnum->GetNext( &sensor );
				if ( sensor != 0 ) {
					++count;
				}
				++i;
			}
		}
	}
	return count;
}

map<size_t, string> getDeviceMap()
{
	map<size_t, string> deviceMap;
	IKinectSensorCollection* sensorCollection	= 0;
	long hr = GetKinectSensorCollection( &sensorCollection );
	if ( SUCCEEDED( hr ) && sensorCollection != 0 ) {
		IEnumKinectSensor* sensorEnum = 0;
		hr = sensorCollection->get_Enumerator( &sensorEnum );
		if ( SUCCEEDED( hr ) || sensorEnum != 0 ) {
			size_t i = 0;
			while ( SUCCEEDED( hr ) && i < 8 ) {
				IKinectSensor* sensor = 0;
				hr = sensorEnum->GetNext( &sensor );
				if ( sensor != 0 ) {
					wchar_t wid[ 48 ];
					if ( SUCCEEDED( sensor->get_UniqueKinectId( 48, wid ) ) ) {
						string id = wcharToString( wid );
						if ( !id.empty() ) {
							deviceMap[ i ] = string( id );
						}
					}
				}
				++i;
			}
		}
	}
	return deviceMap;
}

//string getStatusMessage( KinectStatus status )
//{
//	switch ( status ) {
//	case KinectStatus::KinectStatus_Connected:
//		return "Connected";
//	case KinectStatus::KinectStatus_DeviceNotGenuine:
//		return "Device not genuine";
//	case KinectStatus::KinectStatus_DeviceNotSupported:
//		return "Device not supported";
//	case KinectStatus::KinectStatus_Disconnected:
//		return "Disconnected";
//	case KinectStatus::KinectStatus_Error:
//		return "Error";
//	case KinectStatus::KinectStatus_Initializing:
//		return "Initializing";
//	case KinectStatus::KinectStatus_InsufficientBandwidth:
//		return "Insufficient bandwidth";
//	case KinectStatus::KinectStatus_InUseAsExclusive:
//		return "In use as exclusive";
//	case KinectStatus::KinectStatus_InUseAsShared:
//		return "In use as shared";
//	case KinectStatus::KinectStatus_NotPowered:
//		return "Not powered";
//	case KinectStatus::KinectStatus_NotReady:
//		return "Not ready";
//	default:
//		return "Undefined";
//	}
//}

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

DeviceOptions::DeviceOptions()
: mEnabledAudio( false ), mEnabledBody( false ), 
mEnabledBodyIndex( false ), mEnabledColor( true ), mEnabledDepth( true ), 
mEnabledInfrared( false ), mEnabledInfraredLongExposure( false )
{
}

DeviceOptions& DeviceOptions::enableAudio( bool enable )
{
	mEnabledAudio = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableBody( bool enable )
{
	mEnabledBody = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableBodyIndex( bool enable )
{
	mEnabledBodyIndex = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableColor( bool enable )
{
	mEnabledColor = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableDepth( bool enable )
{
	mEnabledDepth = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableInfrared( bool enable )
{
	mEnabledInfrared = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableInfraredLongExposure( bool enable )
{
	mEnabledInfraredLongExposure = enable;
	return *this;
}

bool DeviceOptions::isAudioEnabled() const
{
	return mEnabledAudio;
}

bool DeviceOptions::isBodyEnabled() const
{
	return mEnabledBody;
}

bool DeviceOptions::isBodyIndexEnabled() const
{
	return mEnabledBodyIndex;
}

bool DeviceOptions::isColorEnabled() const
{
	return mEnabledColor;
}

bool DeviceOptions::isDepthEnabled() const
{
	return mEnabledDepth;
}

bool DeviceOptions::isInfraredEnabled() const
{
	return mEnabledInfrared;
}

bool DeviceOptions::isInfraredLongExposureEnabled() const
{
	return mEnabledInfraredLongExposure;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Body::Joint::Joint()
: mOrientation( Quatf() ), mPosition( Vec3f::zero() ), mTrackingState( TrackingState::TrackingState_NotTracked )
{
}

Body::Joint::Joint( const Vec3f& position, const Quatf& orientation, TrackingState trackingState )
: mOrientation( orientation ), mPosition( position ), mTrackingState( trackingState )
{
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

Audio::Audio()
	: mBeamAngle(0.0f), mBeamAngleConfidence(0.0f), mBuffer(nullptr), 
	mBufferSize( 0 )
{
}

Audio::~Audio()
{
	if (mBuffer != nullptr) {
		delete[] mBuffer;
		mBuffer = nullptr;
	}
}

float Audio::getBeamAngle() const
{
	return mBeamAngle;
}

float Audio::getBeamAngleConfidence() const
{
	return mBeamAngleConfidence;
}

uint8_t* Audio::getBuffer() const
{
	return mBuffer;
}

unsigned long Audio::getBufferSize() const
{
	return mBufferSize;
}

WAVEFORMATEX Audio::getFormat() const
{
	return mFormat;
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

Frame::Frame()
: mFovDiagonalColor( 0.0f ), mFovHorizontalColor( 0.0f ), mFovVerticalColor( 0.0f ), 
mFovDiagonalDepth( 0.0f ), mFovHorizontalDepth( 0.0f ), mFovVerticalDepth( 0.0f )
{
	for ( size_t i = 0; i < 6; ++i ) {
		mTimeStamp[ (TimeStamp)i ] = 0L;
	}
}

const AudioRef& Frame::getAudio() const
{
	return mAudio;
}

Vec2i Frame::getColorSize()
{
	return Vec2i( 1920, 1080 ); 
}

Vec2i Frame::getDepthSize() 
{ 
	return Vec2i( 512, 424 ); 
}

const vector<Body>& Frame::getBodies() const
{
	return mBodies;
}

const Channel8u& Frame::getBodyIndex() const
{
	return mChannelBodyIndex;
}

const Surface8u& Frame::getColor() const
{
	return mSurfaceColor;
}

float Frame::getColorFovDiagonal() const
{
	return mFovDiagonalColor;
}

float Frame::getColorFovHorizontal() const
{
	return mFovHorizontalColor;
}

float Frame::getColorFovVertical() const
{
	return mFovVerticalColor;
}

const Channel16u& Frame::getDepth() const
{
	return mChannelDepth;
}

float Frame::getDepthFovDiagonal() const
{
	return mFovDiagonalDepth;
}

float Frame::getDepthFovHorizontal() const
{
	return mFovHorizontalDepth;
}

float Frame::getDepthFovVertical() const
{
	return mFovVerticalDepth;
}

const Channel16u& Frame::getInfrared() const
{
	return mChannelInfrared;
}

const Channel16u& Frame::getInfraredLongExposure() const
{
	return mChannelInfraredLongExposure;
}

long long Frame::getTimeStamp( TimeStamp timeStamp ) const
{
	if ( mTimeStamp.find( timeStamp ) == mTimeStamp.end() ) {
		timeStamp = TimeStamp::TIMESTAMP_DEFAULT;
	}
	return mTimeStamp.at( timeStamp );
}

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceRef Device::create()
{
	return DeviceRef( new Device() );
}

Device::Device()
	: mAudioReadTime( 0.0 ), mKinect(KCB_INVALID_HANDLE)//, mStatus( KinectStatus::KinectStatus_Undefined )
{
	App::get()->getSignalUpdate().connect( bind( &Device::update, this ) );
}

Device::~Device()
{
	stop();
}

const DeviceOptions& Device::getDeviceOptions() const
{
	return mDeviceOptions;
}

const Frame& Device::getFrame() const
{
	return mFrame;
}

//KinectStatus Device::getStatus() const
//{
//	return mStatus;
//}

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

void Device::start( const DeviceOptions& deviceOptions )
{
	long hr = S_OK;
	mDeviceOptions = deviceOptions;
	
	IKinectSensorCollection* sensorCollection = nullptr;
	hr = GetKinectSensorCollection( &sensorCollection );
	if ( FAILED( hr ) || sensorCollection == 0 ) {
		throw ExcDeviceNotAvailable( hr );
	}
	if ( sensorCollection != nullptr ) {
		sensorCollection->Release();
		sensorCollection = nullptr;
	}

	mKinect = KCBOpenDefaultSensor();
	if ( mKinect == KCB_INVALID_HANDLE ) {
		throw ExcDeviceOpenFailed();
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
}

void Device::update()
{
	if ( mKinect == KCB_INVALID_HANDLE ) {
		return;
	}

	Frame frame;
	
	std::function<Vec2i( KCBFrameDescription*, bool )> processFrame = [ &frame ]( KCBFrameDescription* frameDescription, bool colorFrame )
	{
		Vec2i sz = Vec2i::zero();
		sz.x = frameDescription->width;
		sz.y = frameDescription->height;
		if ( colorFrame ) {
			if ( frame.mFovDiagonalColor <= 0.0f ) {
				frame.mFovDiagonalColor = frameDescription->diagonalFieldOfView;
			}
			if ( frame.mFovHorizontalColor <= 0.0f ) {
				frame.mFovHorizontalColor = frameDescription->horizontalFieldOfView;
			}
			if ( frame.mFovVerticalColor <= 0.0f ) {
				frame.mFovVerticalColor = frameDescription->verticalFieldOfView;
			}
		} else {
			if ( frame.mFovDiagonalDepth <= 0.0f ) {
				frame.mFovDiagonalDepth = frameDescription->diagonalFieldOfView;
			}
			if ( frame.mFovHorizontalDepth <= 0.0f ) {
				frame.mFovHorizontalDepth = frameDescription->horizontalFieldOfView;
			}
			if ( frame.mFovVerticalDepth <= 0.0f ) {
				frame.mFovVerticalDepth = frameDescription->verticalFieldOfView;
			}
		}
		return sz;
	};

	AudioRef audio;
	double e = app::getElapsedSeconds();
	if ( mDeviceOptions.isAudioEnabled() && e - mAudioReadTime > 50.0 ) { //KCBIsFrameReady( mKinect, FrameSourceTypes_Audio ) ) {
		WAVEFORMATEX format;
		long hr = KCBGetAudioFormat( mKinect, &format );
		if ( SUCCEEDED( hr ) ) {
			KCBAudioFrame* audioFrame		= new KCBAudioFrame();
			audioFrame->cAudioBufferSize	= 4;
			audioFrame->pAudioBuffer		= new uint8_t[ audioFrame->cAudioBufferSize * format.nBlockAlign ];
			hr = KCBGetAudioFrame( mKinect, audioFrame );
			if ( SUCCEEDED( hr ) ) {
				audio = AudioRef( new Audio() );
				audio->mBeamAngle			= audioFrame->fBeamAngle;
				audio->mBeamAngleConfidence	= audioFrame->fBeamAngleConfidence;
				audio->mBufferSize			= audioFrame->ulBytesRead;
				if ( audioFrame->ulBytesRead > 0 ) {
					//audio->mBuffer				= new uint8_t[ audio->mBufferSize ];
					//memcpy( frame.mAudio->mBuffer, audioFrame->pAudioBuffer, audio->mBufferSize );
				}
			}
			delete [] audioFrame->pAudioBuffer;
			delete audioFrame;
		}
		mAudioReadTime = e;
	}

	if (mDeviceOptions.isBodyEnabled() && KCBIsFrameReady(mKinect, FrameSourceTypes_Body)) {
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
							Body::Joint joint( 
								toVec3f( joints[ j ].Position ), 
								toQuatf( jointOrientations[ j ].Orientation ), 
								joints[ j ].TrackingState
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
			frame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_BODY ] = timeStamp;
		}
		if ( timeStamp > frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) ) {
			frame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_DEFAULT ] = timeStamp;
		}
	}

	if (mDeviceOptions.isBodyIndexEnabled() && KCBIsFrameReady(mKinect, FrameSourceTypes_BodyIndex)) {
		KCBFrameDescription frameDescription;
		Vec2i sz			= Vec2i::zero();
		int64_t timeStamp	= 0L;
		
		long hr = KCBGetBodyIndexFrameDescription( mKinect, &frameDescription );
		if ( SUCCEEDED( hr ) ) {
			sz = processFrame( &frameDescription, false );
			IBodyIndexFrame* bodyIndexFrame = nullptr;
			hr = KCBGetIBodyIndexFrame( mKinect, &bodyIndexFrame );
			if (SUCCEEDED(hr)) {
				hr = bodyIndexFrame->get_RelativeTime(&timeStamp);
				if (SUCCEEDED(hr)) {
					frame.mTimeStamp[Frame::TimeStamp::TIMESTAMP_BODY_INDEX] = timeStamp;
					frame.mChannelBodyIndex = Channel8u(sz.x, sz.y);
					uint32_t capacity = sz.x * sz.y;
					uint8_t* buffer = frame.mChannelBodyIndex.getData();
					bodyIndexFrame->CopyFrameDataToArray(capacity, buffer);
				}
			}
			if (bodyIndexFrame != nullptr) {
				bodyIndexFrame->Release();
				bodyIndexFrame = nullptr;
			}
		}
		if ( timeStamp > frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) ) {
			frame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_DEFAULT ] = timeStamp;
		}
	}

	if (mDeviceOptions.isColorEnabled() && KCBIsFrameReady(mKinect, FrameSourceTypes_Color)) {
		KCBFrameDescription frameDescription;
		Vec2i sz			= Vec2i::zero();
		int64_t timeStamp	= 0L;
		
		long hr = KCBGetColorFrameDescription( mKinect, ColorImageFormat_Bgra, &frameDescription );
		if ( SUCCEEDED( hr ) ) {
			sz = processFrame( &frameDescription, true );
			IColorFrame* colorFrame = nullptr;
			hr = KCBGetIColorFrame( mKinect, &colorFrame );
			if (SUCCEEDED(hr)) {
				hr = colorFrame->get_RelativeTime(&timeStamp);
				if (SUCCEEDED(hr)) {
					frame.mTimeStamp[Frame::TimeStamp::TIMESTAMP_COLOR] = timeStamp;
					frame.mSurfaceColor = Surface8u(sz.x, sz.y, false, SurfaceChannelOrder::BGRA);
					uint32_t capacity = sz.x * sz.y * frameDescription.bytesPerPixel;
					uint8_t* buffer = frame.mSurfaceColor.getData();
					colorFrame->CopyConvertedFrameDataToArray( capacity, buffer, ColorImageFormat_Bgra);
				}
			}
			if (colorFrame != nullptr) {
				colorFrame->Release();
				colorFrame = nullptr;
			}
		}
		if ( timeStamp > frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) ) {
			frame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_DEFAULT ] = timeStamp;
		}
	}

	if (mDeviceOptions.isDepthEnabled() && KCBIsFrameReady(mKinect, FrameSourceTypes_Depth)) {
		KCBFrameDescription frameDescription;
		Vec2i sz			= Vec2i::zero();
		int64_t timeStamp	= 0L;
		
		long hr = KCBGetDepthFrameDescription( mKinect, &frameDescription );
		if ( SUCCEEDED( hr ) ) {
			sz = processFrame( &frameDescription, false );
			IDepthFrame* depthFrame = nullptr;
			hr = KCBGetIDepthFrame( mKinect, &depthFrame );
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrame->get_RelativeTime( &timeStamp );
				if (SUCCEEDED(hr)) {
					frame.mTimeStamp[Frame::TimeStamp::TIMESTAMP_DEPTH] = timeStamp;
					frame.mChannelDepth = Channel16u(sz.x, sz.y);
					uint32_t capacity = sz.x * sz.y;
					uint16_t* buffer = frame.mChannelDepth.getData();
					depthFrame->CopyFrameDataToArray(capacity, buffer);
				}
			}
			if (depthFrame != nullptr) {
				depthFrame->Release();
				depthFrame = nullptr;
			}
		}
		if ( timeStamp > frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) ) {
			frame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_DEFAULT ] = timeStamp;
		}
	}
	
	if (mDeviceOptions.isInfraredEnabled() && KCBIsFrameReady(mKinect, FrameSourceTypes_Infrared )) {
		KCBFrameDescription frameDescription;
		Vec2i sz			= Vec2i::zero();
		int64_t timeStamp	= 0L;
		
		long hr = KCBGetInfraredFrameDescription( mKinect, &frameDescription );
		sz = processFrame( &frameDescription, false );
		if ( SUCCEEDED( hr ) ) {
			IInfraredFrame* infraredFrame = nullptr;
			hr = KCBGetIInfraredFrame( mKinect, &infraredFrame );
			if (SUCCEEDED(hr)) {
				hr = infraredFrame->get_RelativeTime(&timeStamp);
				if (SUCCEEDED(hr)) {
					frame.mTimeStamp[Frame::TimeStamp::TIMESTAMP_INFRARED] = timeStamp;
					frame.mChannelInfrared = Channel16u(sz.x, sz.y);
					uint32_t capacity = sz.x * sz.y;
					uint16_t* buffer = frame.mChannelInfrared.getData();
					infraredFrame->CopyFrameDataToArray(capacity, buffer);
				}
			}
			if (infraredFrame != nullptr) {
				infraredFrame->Release();
				infraredFrame = nullptr;
			}
		}
		if ( timeStamp > frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) ) {
			frame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_DEFAULT ] = timeStamp;
		}
	}

	if (mDeviceOptions.isInfraredLongExposureEnabled() && KCBIsFrameReady(mKinect, FrameSourceTypes_LongExposureInfrared)) {
		KCBFrameDescription frameDescription;
		Vec2i sz			= Vec2i::zero();
		int64_t timeStamp	= 0L;

		long hr = KCBGetLongExposureInfraredFrameDescription( mKinect, &frameDescription );
		sz = processFrame( &frameDescription, false );
		if ( SUCCEEDED( hr ) ) {
			ILongExposureInfraredFrame* infraredFrame = nullptr;
			hr = KCBGetILongExposureInfraredFrame(mKinect, &infraredFrame);
			if (SUCCEEDED(hr)) {
				hr = infraredFrame->get_RelativeTime(&timeStamp);
				if (SUCCEEDED(hr)) {
					frame.mTimeStamp[Frame::TimeStamp::TIMESTAMP_INFRARED_LONG_EXPOSURE] = timeStamp;
					frame.mChannelInfraredLongExposure = Channel16u(sz.x, sz.y);
					uint32_t capacity = sz.x * sz.y;
					uint16_t* buffer = frame.mChannelInfraredLongExposure.getData();
					infraredFrame->CopyFrameDataToArray(capacity, buffer);
				}
			}
			if (infraredFrame != nullptr) {
				infraredFrame->Release();
				infraredFrame = nullptr;
			}
		}
		if ( timeStamp > frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) ) {
			frame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_DEFAULT ] = timeStamp;
		}
	}

	if ( frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) > mFrame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT ) ) {
		mFrame.mAudio												= audio;
		mFrame.mFovDiagonalColor									= frame.mFovDiagonalColor;
		mFrame.mFovHorizontalColor									= frame.mFovHorizontalColor;
		mFrame.mFovVerticalColor									= frame.mFovVerticalColor;
		mFrame.mFovDiagonalDepth									= frame.mFovDiagonalDepth;
		mFrame.mFovHorizontalDepth									= frame.mFovHorizontalDepth;
		mFrame.mFovVerticalDepth									= frame.mFovVerticalDepth;
		mFrame.mTimeStamp[ Frame::TimeStamp::TIMESTAMP_DEFAULT ]	= frame.getTimeStamp( Frame::TimeStamp::TIMESTAMP_DEFAULT );
		for ( size_t i = 0; i < 6; ++i ) {
			Frame::TimeStamp ts = (Frame::TimeStamp)i;
			if ( frame.getTimeStamp( ts ) > mFrame.getTimeStamp( ts ) ) {
				mFrame.mTimeStamp[ ts ]	= frame.mTimeStamp[ ts ];
				switch ( ts ) {
				case Frame::TimeStamp::TIMESTAMP_BODY:
					mFrame.mBodies = frame.mBodies;
					break;
				case Frame::TimeStamp::TIMESTAMP_BODY_INDEX:
					mFrame.mChannelBodyIndex = frame.mChannelBodyIndex;
					break;
				case Frame::TimeStamp::TIMESTAMP_COLOR:
					mFrame.mSurfaceColor = frame.mSurfaceColor;
					break;
				case Frame::TimeStamp::TIMESTAMP_DEPTH:
					mFrame.mChannelDepth = frame.mChannelDepth;
					break;
				case Frame::TimeStamp::TIMESTAMP_INFRARED:
					mFrame.mChannelInfrared = frame.mChannelInfrared;
					break;
				case Frame::TimeStamp::TIMESTAMP_INFRARED_LONG_EXPOSURE:
					mFrame.mChannelInfraredLongExposure = frame.mChannelInfraredLongExposure;
					break;
				}
			}
		}
	}

	//KCBSensorStatus( mKinect, &mStatus );
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
 