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
using namespace ci::app;
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

string getStatusMessage( KinectStatus status )
{
	switch ( status ) {
	case KinectStatus::KinectStatus_Connected:
		return "Connected";
	case KinectStatus::KinectStatus_DeviceNotGenuine:
		return "Device not genuine";
	case KinectStatus::KinectStatus_DeviceNotSupported:
		return "Device not supported";
	case KinectStatus::KinectStatus_Disconnected:
		return "Disconnected";
	case KinectStatus::KinectStatus_Error:
		return "Error";
	case KinectStatus::KinectStatus_Initializing:
		return "Initializing";
	case KinectStatus::KinectStatus_InsufficientBandwidth:
		return "Insufficient bandwidth";
	case KinectStatus::KinectStatus_InUseAsExclusive:
		return "In use as exclusive";
	case KinectStatus::KinectStatus_InUseAsShared:
		return "In use as shared";
	case KinectStatus::KinectStatus_NotPowered:
		return "Not powered";
	case KinectStatus::KinectStatus_NotReady:
		return "Not ready";
	default:
		return "Undefined";
	}
}

Vec2i mapBodyCoordToColor( const Vec3f& v, ICoordinateMapper* mapper )
{
	if ( mapper != 0 ) {
		CameraSpacePoint cameraSpacePoint;
		cameraSpacePoint.X = v.x;
		cameraSpacePoint.Y = v.y;
		cameraSpacePoint.Z = v.z;

		ColorSpacePoint colorSpacePoint;
		long hr = mapper->MapCameraPointToColorSpace( cameraSpacePoint, &colorSpacePoint );
		if ( SUCCEEDED( hr ) ) {
			return Vec2i( static_cast<int32_t>( colorSpacePoint.X ), static_cast<int32_t>( colorSpacePoint.Y ) );
		}
	}
	return Vec2i();
}

Vec2i mapBodyCoordToDepth( const Vec3f& v, ICoordinateMapper* mapper )
{
	if ( mapper != 0 ) {
		CameraSpacePoint cameraSpacePoint;
		cameraSpacePoint.X = v.x;
		cameraSpacePoint.Y = v.y;
		cameraSpacePoint.Z = v.z;

		DepthSpacePoint depthSpacePoint;
		long hr = mapper->MapCameraPointToDepthSpace( cameraSpacePoint, &depthSpacePoint );
		if ( SUCCEEDED( hr ) ) {
			return Vec2i( toVec2f( depthSpacePoint ) );
		}
	}
	return Vec2i();
}

Vec2i mapDepthCoordToColor( const Vec2i& v, uint16_t depth, ICoordinateMapper* mapper )
{
	if ( mapper != 0 ) {
		DepthSpacePoint depthSpacePoint;
		depthSpacePoint.X = (float)v.x;
		depthSpacePoint.Y = (float)v.y;

		ColorSpacePoint colorSpacePoint;
		long hr = mapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );
		if ( SUCCEEDED( hr ) ) {
			return Vec2i( toVec2f( colorSpacePoint ) );
		}
	}
	return Vec2i();
}

Channel16u mapDepthFrameToColor( const Channel16u& depth, ICoordinateMapper* mapper )
{
	Channel16u channel( depth.getWidth(), depth.getHeight() );
	if ( mapper != 0 ) {
		size_t numPoints = depth.getWidth() * depth.getHeight();
		vector<ColorSpacePoint> colorSpacePoints( numPoints );
		long hr = mapper->MapDepthFrameToColorSpace( (UINT)numPoints, depth.getData(), (UINT)numPoints, &colorSpacePoints[ 0 ] );
		if ( SUCCEEDED( hr ) ) {
			Channel16u::Iter iter = channel.getIter();
			size_t i = 0;
			while ( iter.line() ) {
				while ( iter.pixel() ) {
					Vec2i pos = Vec2i( toVec2f( colorSpacePoints[ i ] ) );
					uint16_t v = 0x0000;
					if ( pos.x >= 0 && pos.x < depth.getWidth() && pos.y >= 0 && pos.y < depth.getHeight() ) {
						v = depth.getValue( pos );
					}
					iter.v() = v;
				}
			}
		}
	}
	return channel;
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

Body::Body()
: mId( 0 ), mIndex( 0 ), mTracked( false )
{
}

Body::Body( uint64_t id, uint8_t index, const map<JointType, Body::Joint>& jointMap )
: mId( id ), mIndex( index ), mJointMap( jointMap ), mTracked( true )
{
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
: mDepthMaxReliableDistance( 0 ), mDepthMinReliableDistance( 0 ), mFovDiagonal( 0.0f ), 
mFovHorizontal( 0.0f ), mFovVertical( 0.0f ), mTimeStamp( 0L )
{
}

Frame::Frame( long long time, const string& deviceId, const Surface8u& color,
			  const Channel16u& depth, const Channel16u& infrared, 
			  const Channel16u& infraredLongExposure )
: mSurfaceColor( color ), mChannelDepth( depth ), mChannelInfrared( infrared ), 
mChannelInfraredLongExposure( infraredLongExposure ), mTimeStamp( time )
{
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

const Channel16u& Frame::getDepth() const
{
	return mChannelDepth;
}

float Frame::getFovDiagonal() const
{
	return mFovDiagonal;
}

float Frame::getFovHorizontal() const
{
	return mFovHorizontal;
}

float Frame::getFovVertical() const
{
	return mFovVertical;
}

const Channel16u& Frame::getInfrared() const
{
	return mChannelInfrared;
}

const Channel16u& Frame::getInfraredLongExposure() const
{
	return mChannelInfraredLongExposure;
}

uint16_t Frame::getMaxReliableDepthDistance() const
{
	return mDepthMinReliableDistance;
}

uint16_t Frame::getMinReliableDepthDistance() const
{
	return mDepthMinReliableDistance;
}

int64_t Frame::getTimeStamp() const
{
	return mTimeStamp;
}

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceRef Device::create()
{
	return DeviceRef( new Device() );
}

Device::Device()
: mCoordinateMapper( 0 ), mKinect( KCB_INVALID_HANDLE ), mStatus( KinectStatus::KinectStatus_Undefined )
{
	App::get()->getSignalUpdate().connect( bind( &Device::update, this ) );
}

Device::~Device()
{
	stop();
}

ICoordinateMapper* Device::getCoordinateMapper() const
{
	return mCoordinateMapper;
}

const DeviceOptions& Device::getDeviceOptions() const
{
	return mDeviceOptions;
}

const Frame& Device::getFrame() const
{
	return mFrame;
}

KinectStatus Device::getStatus() const
{
	return mStatus;
}

void Device::start( const DeviceOptions& deviceOptions )
{
	long hr = S_OK;
	mDeviceOptions = deviceOptions;
	
	IKinectSensorCollection* sensorCollection = 0;
	hr = GetKinectSensorCollection( &sensorCollection );
	if ( FAILED( hr ) || sensorCollection == 0 ) {
		throw ExcDeviceNotAvailable( hr );
	}

	mKinect = KCBOpenDefaultSensor();
	if ( mKinect == KCB_INVALID_HANDLE ) {
		throw ExcDeviceOpenFailed();
	}

	/*hr = KCBGetCoordinateMapper( mKinect, mCoordinateMapper );
	if ( FAILED( hr ) ) {
		throw ExcGetCoordinateMapperFailed( hr );
	}*/
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

	if ( mDeviceOptions.isAudioEnabled() ) {
		IAudioBeamFrame* audioFrame = nullptr;
		if ( audioFrame != nullptr ) {
			// TODO audio
			audioFrame->Release();
			audioFrame = nullptr;
		}
	}

	if ( mDeviceOptions.isBodyEnabled() ) {
		IBodyFrame* bodyFrame = nullptr;
		long hr = KCBGetBodyFrame( mKinect, &bodyFrame );
		if ( SUCCEEDED( hr ) && bodyFrame != nullptr ) {
			int64_t timeStamp					= 0L;
			IBody* kinectBodies[ BODY_COUNT ]	= { 0 };
		
			hr = bodyFrame->GetAndRefreshBodyData( BODY_COUNT, kinectBodies );
			if ( SUCCEEDED( hr ) ) {
				hr = bodyFrame->get_RelativeTime( &timeStamp );
			}
			if ( SUCCEEDED( hr ) ) {
				for ( uint8_t i = 0; i < 6; ++i ) {
					IBody* kinectBody = kinectBodies[ i ];
					if ( kinectBody != 0 ) {
						uint8_t isTracked	= false;
						hr					= kinectBody->get_IsTracked( &isTracked );
						if ( SUCCEEDED( hr ) && isTracked ) {
							Joint joints[ JointType_Count ];
							kinectBody->GetJoints( JointType_Count, joints );

							JointOrientation jointOrientations[ JointType_Count ];
							kinectBody->GetJointOrientations( JointType_Count, jointOrientations );

							uint64_t id = 0;
							kinectBody->get_TrackingId( &id );

							std::map<JointType, Body::Joint> jointMap;
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
					}
				}
			}
			bodyFrame->Release();
			bodyFrame = nullptr;
		}
	}

	if ( mDeviceOptions.isBodyIndexEnabled() ) {
		IBodyIndexFrame* bodyIndexFrame = nullptr;
		long hr = KCBGetBodyIndexFrame( mKinect, &bodyIndexFrame );
		if ( SUCCEEDED( hr ) && bodyIndexFrame != nullptr ) {
			IFrameDescription* frameDescription	= 0;
			int32_t width						= 0;
			int32_t height						= 0;
			int64_t timeStamp					= 0L;
		
			hr	= bodyIndexFrame->get_FrameDescription( &frameDescription );
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Width( &width );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Height( &height );
			}
			if ( SUCCEEDED( hr ) ) {
				frame.mChannelBodyIndex = Channel8u( width, height );
				KCBGetBodyIndexFrameBuffer( mKinect, width * height * sizeof( uint8_t ), frame.mChannelBodyIndex.getData(), &timeStamp );
			}
			if ( frameDescription != 0 ) {
				frameDescription->Release();
				frameDescription = 0;
			}
			if ( timeStamp > frame.mTimeStamp ) {
				frame.mTimeStamp = timeStamp;
			}
			bodyIndexFrame->Release();
			bodyIndexFrame = nullptr;
		}
	}

	if ( mDeviceOptions.isColorEnabled() ) {
		IColorFrame* colorFrame	= nullptr;
		long hr = KCBGetColorFrame( mKinect, &colorFrame );
		if ( SUCCEEDED( hr ) && colorFrame != nullptr ) {
			IFrameDescription* frameDescription	= 0;
			int32_t width						= 0;
			int32_t height						= 0;
			ColorImageFormat imageFormat		= ColorImageFormat_None;
			int64_t timeStamp					= 0L;
		
			hr = colorFrame->get_FrameDescription( &frameDescription );
			if ( SUCCEEDED( hr ) ) {
				hr = colorFrame->get_RawColorImageFormat( &imageFormat );
			}
			if ( SUCCEEDED( hr ) && frame.mFovDiagonal <= 0.0f ) {
				hr = frameDescription->get_DiagonalFieldOfView( &frame.mFovDiagonal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovHorizontal <= 0.0f ) {
				hr = frameDescription->get_HorizontalFieldOfView( &frame.mFovHorizontal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovVertical ) {
				hr = frameDescription->get_VerticalFieldOfView( &frame.mFovVertical );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Width( &width );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Height( &height );
			}
			if ( SUCCEEDED( hr ) ) {
				frame.mSurfaceColor = Surface8u( width, height, false, SurfaceChannelOrder::BGRA );
				KCBGetColorFrameAsBGRA( mKinect, width * height * sizeof( uint8_t ) * 4, frame.mSurfaceColor.getData(), &timeStamp );
			}
			if ( frameDescription != 0 ) {
				frameDescription->Release();
				frameDescription = 0;
			}
			if ( timeStamp > frame.mTimeStamp ) {
				frame.mTimeStamp = timeStamp;
			}
			colorFrame->Release();
			colorFrame = nullptr;
		}
	}

	if ( mDeviceOptions.isDepthEnabled() ) {
		IDepthFrame* depthFrame = nullptr;
		long hr = KCBGetDepthFrame( mKinect, &depthFrame );
		if ( SUCCEEDED( hr ) && depthFrame != nullptr ) {
			IFrameDescription* frameDescription	= 0;
			int32_t width						= 0;
			int32_t height						= 0;
			int64_t timeStamp					= 0L;

			hr = depthFrame->get_FrameDescription( &frameDescription );
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrame->get_DepthMaxReliableDistance( &frame.mDepthMaxReliableDistance );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrame->get_DepthMinReliableDistance( &frame.mDepthMinReliableDistance );
			}
			if ( SUCCEEDED( hr ) && frame.mFovDiagonal <= 0.0f ) {
				hr = frameDescription->get_DiagonalFieldOfView( &frame.mFovDiagonal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovHorizontal <= 0.0f ) {
				hr = frameDescription->get_HorizontalFieldOfView( &frame.mFovHorizontal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovVertical ) {
				hr = frameDescription->get_VerticalFieldOfView( &frame.mFovVertical );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Width( &width );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Height( &height );
			}
			if ( SUCCEEDED( hr ) ) {
				frame.mChannelDepth = Channel16u( width, height );
				hr = KCBGetDepthFrameBuffer( mKinect, width * height, frame.mChannelDepth.getData(), &timeStamp );
			}
			if ( frameDescription != 0 ) {
				frameDescription->Release();
				frameDescription = 0;
			}
			if ( timeStamp > frame.mTimeStamp ) {
				frame.mTimeStamp = timeStamp;
			}
			depthFrame->Release();
			depthFrame = nullptr;
		}
	}

	if ( mDeviceOptions.isInfraredEnabled() ) {
		IInfraredFrame* infraredFrame = nullptr;
		long hr = KCBGetInfraredFrame( mKinect, &infraredFrame );
		if ( SUCCEEDED( hr ) && infraredFrame != nullptr ) {
			IFrameDescription* frameDescription	= 0;
			int32_t width						= 0;
			int32_t height						= 0;
			int64_t timeStamp					= 0L;
		
			hr = infraredFrame->get_FrameDescription( &frameDescription );
			if ( SUCCEEDED( hr ) && frame.mFovDiagonal <= 0.0f ) {
				hr = frameDescription->get_DiagonalFieldOfView( &frame.mFovDiagonal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovHorizontal <= 0.0f ) {
				hr = frameDescription->get_HorizontalFieldOfView( &frame.mFovHorizontal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovVertical ) {
				hr = frameDescription->get_VerticalFieldOfView( &frame.mFovVertical );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Width( &width );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Height( &height );
			}
			if ( SUCCEEDED( hr ) ) {
				frame.mChannelInfrared = Channel16u( width, height );
				KCBGetInfraredFrameBuffer( mKinect, width * height, frame.mChannelInfrared.getData(), &timeStamp );
			}
			if ( frameDescription != 0 ) {
				frameDescription->Release();
				frameDescription = 0;
			}
			if ( timeStamp > frame.mTimeStamp ) {
				frame.mTimeStamp = timeStamp;
			}
			infraredFrame->Release();
			infraredFrame = nullptr;
		}
	}

	if ( mDeviceOptions.isInfraredLongExposureEnabled() ) {
		ILongExposureInfraredFrame* infraredLongExposureFrame = nullptr;
		long hr = KCBGetLongExposureInfraredFrame( mKinect, &infraredLongExposureFrame );
		if ( SUCCEEDED( hr ) && infraredLongExposureFrame != nullptr ) {
			IFrameDescription* frameDescription	= 0;
			int32_t width						= 0;
			int32_t height						= 0;
			int64_t timeStamp					= 0L;

			hr = infraredLongExposureFrame->get_FrameDescription( &frameDescription );
			if ( SUCCEEDED( hr ) && frame.mFovDiagonal <= 0.0f ) {
				hr = frameDescription->get_DiagonalFieldOfView( &frame.mFovDiagonal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovHorizontal <= 0.0f ) {
				hr = frameDescription->get_HorizontalFieldOfView( &frame.mFovHorizontal );
			}
			if ( SUCCEEDED( hr ) && frame.mFovVertical ) {
				hr = frameDescription->get_VerticalFieldOfView( &frame.mFovVertical );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Width( &width );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = frameDescription->get_Height( &height );
			}
			if ( SUCCEEDED( hr ) ) {
				frame.mChannelInfraredLongExposure = Channel16u( width, height );
				KCBGetLongExposureInfraredFrameBuffer( mKinect, width * height, frame.mChannelInfraredLongExposure.getData(), &timeStamp );
			}
			if ( frameDescription != 0 ) {
				frameDescription->Release();
				frameDescription = 0;
			}
			if ( timeStamp > frame.mTimeStamp ) {
				frame.mTimeStamp = timeStamp;
			}
			infraredLongExposureFrame->Release();
			infraredLongExposureFrame = nullptr;
		}
	}

	if ( frame.mTimeStamp > 0L ) {
		mFrame = frame;
	}

	KCBSensorStatus( mKinect, &mStatus );
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

Device::ExcGetCoordinateMapperFailed::ExcGetCoordinateMapperFailed( long hr ) throw()
{
	sprintf( mMessage, "Unable to get device coordinate mapper. Error: %i", hr );
}

}
 