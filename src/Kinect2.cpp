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

#include "Kinect2.h"
#include "cinder/app/App.h"

#include <comutil.h>

namespace Kinect2 {

using namespace ci;
using namespace app;
using namespace std;

static const unsigned long kFaceFrameFeatures = 
	FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace	| 
	FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace			|
	FaceFrameFeatures::FaceFrameFeatures_RotationOrientation		| 
	FaceFrameFeatures::FaceFrameFeatures_Happy						| 
	FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed				| 
	FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed				| 
	FaceFrameFeatures::FaceFrameFeatures_MouthOpen					| 
	FaceFrameFeatures::FaceFrameFeatures_MouthMoved					| 
	FaceFrameFeatures::FaceFrameFeatures_LookingAway				| 
	FaceFrameFeatures::FaceFrameFeatures_Glasses					| 
	FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

Channel8uRef channel16To8( const Channel16uRef& channel, uint8_t bytes )
{
	Channel8uRef channel8;
	if ( channel ) {
		channel8				= Channel8u::create( channel->getWidth(), channel->getHeight() );
		Channel16u::Iter iter16	= channel->getIter();
		Channel8u::Iter iter8	= channel8->getIter();
		while ( iter8.line() && iter16.line() ) {
			while ( iter8.pixel() && iter16.pixel() ) {
				iter8.v()				= static_cast<uint8_t>( iter16.v() >> bytes );
			}
		}
	}
	return channel8;
}

Surface8uRef colorizeBodyIndex( const Channel8uRef& bodyIndexChannel )
{
	Surface8uRef surface;
	if ( bodyIndexChannel ) {
		surface = Surface8u::create( bodyIndexChannel->getWidth(), bodyIndexChannel->getHeight(), true, SurfaceChannelOrder::RGBA );
		Channel8u::Iter iterChannel	= bodyIndexChannel->getIter();
		Surface8u::Iter iterSurface	= surface->getIter();
		while ( iterChannel.line() && iterSurface.line() ) {
			while ( iterChannel.pixel() && iterSurface.pixel() ) {
				size_t index				= (size_t)iterChannel.v();
				ColorA8u color( getBodyColor( index ), 0xFF );
				if ( index == 0 || index > BODY_COUNT ) {
					color.a		= 0x00;
				}
				iterSurface.r()	= color.r;
				iterSurface.g()	= color.g;
				iterSurface.b()	= color.b;
				iterSurface.a()	= color.a;
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
	//IKinectSensorCollection* sensorCollection	= nullptr;
	//long hr = GetKinectSensorCollection( &sensorCollection );
	//if ( SUCCEEDED( hr ) && sensorCollection != nullptr ) {
	//	IEnumKinectSensor* sensorEnum = nullptr;
	//	hr = sensorCollection->get_Enumerator( &sensorEnum );
	//	if ( SUCCEEDED( hr ) || sensorEnum != nullptr ) {
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
	//IKinectSensorCollection* sensorCollection	= nullptr;
	//long hr = GetKinectSensorCollection( &sensorCollection );
	//if ( SUCCEEDED( hr ) && sensorCollection != 0 ) {
	//	IEnumKinectSensor* sensorEnum = nullptr;
	//	hr = sensorCollection->get_Enumerator( &sensorEnum );
	//	if ( SUCCEEDED( hr ) || sensorEnum != nullptr ) {
	//		size_t i = 0;
	//		while ( SUCCEEDED( hr ) && i < 8 ) {
	//			IKinectSensor* sensor = nullptr;
	//			hr = sensorEnum->GetNext( &sensor );
	//			if ( sensor != nullptr ) {
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

CameraSpacePoint toCameraSpacePoint( const vec3& v )
{
	CameraSpacePoint p;
	p.X = v.x;
	p.Y = v.y;
	p.Z = v.z;
	return p;
}

ColorSpacePoint	toColorSpacePoint( const vec2& v )
{
	ColorSpacePoint p;
	p.X = v.x;
	p.Y = v.y;
	return p;
}

DepthSpacePoint	toDepthSpacePoint( const vec2& v )
{
	DepthSpacePoint p;
	p.X = v.x;
	p.Y = v.y;
	return p;
}

PointF toPointF( const vec2& v )
{
	PointF p;
	p.X = v.x;
	p.Y = v.y;
	return p;
}

Vector4 toVector4( const quat& q )
{
	Vector4 p;
	p.w = q.w;
	p.x = q.x;
	p.y = q.y;
	p.z = q.z;
	return p;
}

Vector4 toVector4( const vec4& v )
{
	Vector4 p;
	p.w = v.w;
	p.x = v.x;
	p.y = v.y;
	p.z = v.z;
	return p;
}

quat toQuat( const Vector4& v )
{
	return quat( v.w, v.x, v.y, v.z );
}

Rectf toRectf( const RectI& v )
{
	Rectf r;
	r.x1 = (float)v.Left;
	r.y1 = (float)v.Top;
	r.x2 = (float)v.Right;
	r.y2 = (float)v.Bottom;
	return r;
}

vec2 toVec2( const PointF& v )
{
	return vec2( v.X, v.Y );
}

vec2 toVec2( const ColorSpacePoint& v )
{
	return vec2( v.X, v.Y );
}

vec2 toVec2( const DepthSpacePoint& v )
{
	return vec2( v.X, v.Y );
}

vec3 toVec3( const CameraSpacePoint& v )
{
	return vec3( v.X, v.Y, v.Z );
}

vec4 toVec4( const Vector4& v )
{
	return vec4( v.x, v.y, v.z, v.w );
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

IFace::IFace()
: mId( 0 ), mIndex( 0 ), mTracked( false )
{
}

uint64_t IFace::getId() const
{
	return mId;
}

uint8_t IFace::getIndex() const
{
	return mIndex;
}

bool IFace::isTracked() const
{
	return mTracked;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Face2d::Face2d()
: IFace()
{
	for ( size_t i = 0; i < (size_t)FaceProperty_Count; ++i ) {
		mFaceProperties[ (FaceProperty)i ] = DetectionResult_Unknown;
	}
}

const Rectf& Face2d::getBoundsColor() const
{
	return mBoundsColor;
}

const Rectf& Face2d::getBoundsInfrared() const
{
	return mBoundsInfrared;
}

const map<FaceProperty, DetectionResult>& Face2d::getFaceProperties() const
{
	return mFaceProperties;
}

const vector<vec2>& Face2d::getPointsColor() const
{
	return mPointsColor;
}

const vector<vec2>& Face2d::getPointsInfrared() const
{
	return mPointsInfrared;
}

const quat& Face2d::getRotation() const
{
	return mRotation;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Face3d::Face3d()
: IFace(), mColorHair( ColorA8u::hex( 0x00000000 ) ), mColorSkin( ColorA8u::hex( 0x00000000 ) ), 
mFaceAlignmentQuality( FaceAlignmentQuality_Low ), mHeadPivotPoint( vec3( 0.0f ) ), 
mScale( 0.0f )
{
	for ( size_t i = 0; i < (size_t)FaceShapeAnimations_Count; ++i ) {
		mFaceShapeAnimations[ (FaceShapeAnimations)i ] = 0.0f;
	}
	for ( size_t i = 0; i < (size_t)FaceShapeDeformations_Count; ++i ) {
		mFaceShapeDeformations[ (FaceShapeDeformations)i ] = 0.0f;
	}
}

const Rectf& Face3d::getBounds() const
{
	return mBounds;
}

FaceAlignmentQuality Face3d::getFaceAlignmentQuality() const
{
	return mFaceAlignmentQuality;
}

const map<FaceShapeAnimations, float>& Face3d::getFaceShapeAnimations() const
{
	return mFaceShapeAnimations;
}

const map<FaceShapeDeformations, float>& Face3d::getFaceShapeDeformations() const
{
	return mFaceShapeDeformations;
}

const ColorA8u& Face3d::getHairColor() const
{
	return mColorHair;
}

const vec3& Face3d::getHeadPivotPoint() const
{
	return mHeadPivotPoint;
}

const TriMeshRef& Face3d::getMesh() const
{
	return mMesh;
}

const quat& Face3d::getOrientation() const
{
	return mOrientation;
}

float Face3d::getScale() const
{
	return mScale;
}

const ColorA8u& Face3d::getSkinColor() const
{
	return mColorSkin;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Body::Hand::Hand()
: mConfidence( TrackingConfidence_Low ), mState( HandState_Unknown )
{
}

TrackingConfidence Body::Hand::getConfidence() const
{
	return mConfidence;
}

HandState Body::Hand::getState() const
{
	return mState;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Body::Joint::Joint()
: mOrientation( quat() ), mParentJoint( JointType::JointType_Count ), mPosition( vec3( 0.0f ) ), 
mTrackingState( TrackingState_NotTracked )
{
}

Body::Joint::Joint( const vec3& position, const quat& orientation, TrackingState trackingState,
	JointType parentJoint )
: mOrientation( orientation ), mPosition( position ), mParentJoint( parentJoint ), 
mTrackingState( trackingState )
{
}

JointType Body::Joint::getParentJoint() const
{
	return mParentJoint;
}

const vec3& Body::Joint::getPosition() const
{
	return mPosition;
}

const quat& Body::Joint::getOrientation() const
{
	return mOrientation;
}

TrackingState Body::Joint::getTrackingState() const
{
	return mTrackingState;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Body::Body()
: mEngaged( DetectionResult_Unknown ), mId( 0 ), mIndex( 0 ), 
mLean( vec2( 0.0f ) ), mLeanTrackingState( TrackingState_NotTracked ), 
mRestricted( false ), mTracked( false )
{
	for ( size_t i = 0; i < (size_t)Activity_Count; ++i ) {
		mActivities[ (Activity)i ] = DetectionResult_Unknown;
	}
	for ( size_t i = 0; i < (size_t)Appearance_Count; ++i ) {
		mAppearances[ (Appearance)i ] = DetectionResult_Unknown;
	}
	for ( size_t i = 0; i < (size_t)Expression_Count; ++i ) {
		mExpressions[ (Expression)i ] = DetectionResult_Unknown;
	}
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
		for ( map<JointType, Joint>::const_iterator iter = mJointMap.begin(); iter != mJointMap.end(); ++iter ) {
			if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
				c += weights[ iter->first ];
			}
		}
	} else {
		for ( map<JointType, Joint>::const_iterator iter = mJointMap.begin(); iter != mJointMap.end(); ++iter ) {
			if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
				c += 1.0f;
			}
		}
		c /= (float)JointType::JointType_Count;
	}
	return c;
}

const map<Activity, DetectionResult>& Body::getActivities() const
{
	return mActivities;
}

const map<Appearance, DetectionResult>& Body::getAppearances() const
{
	return mAppearances;
}

const map<Expression, DetectionResult>& Body::getExpressions() const
{
	return mExpressions;
}

const Body::Hand& Body::getHandLeft() const
{
	return mHands[ 0 ];
}

const Body::Hand& Body::getHandRight() const
{
	return mHands[ 1 ];
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

const vec2& Body::getLean() const
{
	return mLean;
}

TrackingState Body::getLeanTrackingState() const
{
	return mLeanTrackingState;
}

DetectionResult Body::isEngaged() const
{
	return mEngaged;
}

bool Body::isTracked() const 
{ 
	return mTracked; 
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
mFovVertical( 0.0f ), mSize( ivec2( 0 ) )
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

const ivec2& CameraFrame::getSize() const
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

BodyFrame::BodyFrame()
: Frame()
{
}

const vector<Body>& BodyFrame::getBodies() const
{
	return mBodies;
}

//////////////////////////////////////////////////////////////////////////////////////////////

template<typename T> 
ChannelFrameT<T>::ChannelFrameT()
: Frame()
{
}

template<typename T> 
const std::shared_ptr<ChannelT<T> >& ChannelFrameT<T>::getChannel() const
{
	return mChannel;
}

template class ChannelFrameT<uint8_t>;
template class ChannelFrameT<uint16_t>;

//////////////////////////////////////////////////////////////////////////////////////////////

ColorFrame::ColorFrame()
: CameraFrame(), Frame()
{
	mSize = ivec2( 1920, 1080 );
}

const Surface8uRef& ColorFrame::getSurface() const
{
	return mSurface;
}

//////////////////////////////////////////////////////////////////////////////////////////////

DepthFrame::DepthFrame()
: CameraFrame(), ChannelFrame16u()
{
	mSize = ivec2( 512, 424 );
}

//////////////////////////////////////////////////////////////////////////////////////////////

Face2dFrame::Face2dFrame()
: Frame()
{
}

const vector<Face2d>& Face2dFrame::getFaces() const
{
	return mFaces;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Face3dFrame::Face3dFrame()
: Frame()
{
}

const vector<Face3d>& Face3dFrame::getFaces() const
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

static const long long kThreadSleepDuration = 30L;

uint32_t Device::sFaceModelIndexCount	= 0;
uint32_t Device::sFaceModelVertexCount	= 0;

DeviceRef Device::create()
{
	return DeviceRef( new Device() );
}

Device::Device()
	: mEnabledFaceMesh( false ), mEnabledHandTracking( false ), 
	mEnabledJointTracking( true ), mEventHandlerAudio( nullptr ), 
	mEventHandlerBody( nullptr ), mEventHandlerBodyIndex( nullptr ), 
	mEventHandlerColor( nullptr ), mEventHandlerDepth( nullptr ), 
	mEventHandlerFace2d( nullptr ), mEventHandlerFace3d( nullptr ), 
	mEventHandlerInfrared( nullptr ), mEventHandlerInfraredLongExposure( nullptr ), 
	mKinect( KCB_INVALID_HANDLE ), mSensor( nullptr )
{
	if ( sFaceModelIndexCount == 0 ) {
		GetFaceModelTriangleCount( &sFaceModelIndexCount );
		sFaceModelIndexCount *= 3;
	}
	if ( sFaceModelVertexCount == 0 ) {
		GetFaceModelVertexCount( &sFaceModelVertexCount );
	}
	for ( size_t i = 0; i < FaceShapeDeformations_Count; ++i ) {
		mFaceShapeDeformations[ i ] = 0.0f;
	}
	
	App::get()->getSignalUpdate().connect( bind( &Device::update, this ) );
}

Device::~Device()
{
	stop();
	if ( mSensor != nullptr ) {
		mSensor->Release();
		mSensor = nullptr;
	}
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

void Device::connectFace2dEventHandler( const function<void ( const Face2dFrame& )>& eventHandler )
{
	mEventHandlerFace2d = eventHandler;
}

void Device::connectFace3dEventHandler( const function<void ( const Face3dFrame& )>& eventHandler )
{
	mEventHandlerFace3d = eventHandler;
}

void Device::connectInfraredEventHandler( const function<void ( const InfraredFrame& )>& eventHandler )
{
	mEventHandlerInfrared = eventHandler;
}

void Device::connectInfraredLongExposureEventHandler( const function<void ( const InfraredFrame& )>& eventHandler )
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

void Device::disconnectFace2dEventHandler()
{
	mEventHandlerFace2d = nullptr;
}

void Device::disconnectFace3dEventHandler()
{
	mEventHandlerFace3d = nullptr;
}

void Device::disconnectInfraredEventHandler()
{
	mEventHandlerInfrared = nullptr;
}

void Device::disconnectInfraredLongExposureEventHandler()
{
	mEventHandlerInfraredLongExposure = nullptr;
}

bool Device::isAudioEventHandlerConnected() const
{
	return mEventHandlerAudio != nullptr;
}

bool Device::isBodyEventHandlerConnected() const
{
	return mEventHandlerBody != nullptr;
}

bool Device::isBodyIndexEventHandlerConnected() const
{
	return mEventHandlerBodyIndex != nullptr;
}

bool Device::isColorEventHandlerConnected() const
{
	return mEventHandlerColor != nullptr;
}

bool Device::isDepthEventHandlerConnected() const
{
	return mEventHandlerDepth != nullptr;
}

bool Device::isFace2dEventHandlerConnected() const
{
	return mEventHandlerFace2d != nullptr;
}

bool Device::isFace3dEventHandlerConnected() const
{
	return mEventHandlerFace3d != nullptr;
}

bool Device::isInfraredEventHandlerConnected() const
{
	return mEventHandlerInfrared != nullptr;
}

bool Device::isInfraredLongExposureEventHandlerConnected() const
{
	return mEventHandlerInfraredLongExposure != nullptr;
}

void Device::enableFaceMesh( bool enable )
{
	mEnabledFaceMesh = enable;
}

void Device::enableHandTracking( bool enable )
{
	mEnabledHandTracking = enable;
}

void Device::enableJointTracking( bool enable )
{
	mEnabledJointTracking = enable;
}

bool Device::isFaceMeshEnabled() const
{
	return mEnabledFaceMesh;
}

bool Device::isHandTrackingEnabled() const
{
	return mEnabledHandTracking;
}

bool Device::isJointTrackingEnabled() const
{
	return mEnabledJointTracking;
}

ivec2 Device::mapCameraToColor( const vec3& v ) const
{
	ColorSpacePoint p;
	KCBMapCameraPointToColorSpace( mKinect, toCameraSpacePoint( v ), &p ); 
	return ivec2( toVec2( p ) );
}

vector<ivec2> Device::mapCameraToColor( const vector<vec3>& v ) const
{
	vector<ivec2> p;
	vector<CameraSpacePoint> camera;
	vector<ColorSpacePoint> color;
	for_each( v.begin(), v.end(), [ &camera ]( const vec3& i )
	{
		camera.push_back( toCameraSpacePoint( i ) );
	} );
	KCBMapCameraPointsToColorSpace( mKinect, camera.size(), &camera[ 0 ], color.size(), &color[ 0 ] );
	for_each( color.begin(), color.end(), [ &p ]( const ColorSpacePoint& i )
	{
		p.push_back( ivec2( toVec2( i ) ) );
	} );
	return p;
}

ivec2 Device::mapCameraToDepth( const vec3& v ) const
{
	DepthSpacePoint p;
	KCBMapCameraPointToDepthSpace( mKinect, toCameraSpacePoint( v ), &p ); 
	return ivec2( toVec2( p ) );
}

vector<ivec2> Device::mapCameraToDepth( const vector<vec3>& v ) const
{
	vector<ivec2> p;
	vector<CameraSpacePoint> camera;
	vector<DepthSpacePoint> depth( v.size() );
	for_each( v.begin(), v.end(), [ &camera ]( const vec3& i )
	{
		camera.push_back( toCameraSpacePoint( i ) );
	} );
	KCBMapCameraPointsToDepthSpace( mKinect, camera.size(), &camera[ 0 ], depth.size(), &depth[ 0 ] );
	for_each( depth.begin(), depth.end(), [ &p ]( const DepthSpacePoint& i )
	{
		p.push_back( ivec2( toVec2( i ) ) );
	} );
	return p;
}

vec3 Device::mapDepthToCamera( const ivec2& v, const Channel16uRef& depth ) const
{
	CameraSpacePoint p;
	if ( depth ) {
		uint16_t d = depth->getValue( v );
		KCBMapDepthPointToCameraSpace( mKinect, toDepthSpacePoint( v ), d, &p ); 
	}
	return toVec3( p );
}

vector<vec3> Device::mapDepthToCamera( const vector<ivec2>& v, const Channel16uRef& depth ) const
{
	vector<vec3> p;
	if ( depth ) {
		vector<CameraSpacePoint> camera( v.size() );
		vector<DepthSpacePoint> depthPos;
		vector<uint16_t> depthVal;
		for_each( v.begin(), v.end(), [ &depth, &depthPos, &depthVal ]( const ivec2& i )
		{
			depthPos.push_back( toDepthSpacePoint( i ) );
			depthVal.push_back( depth->getValue( i ) );
		} );
		KCBMapDepthPointsToCameraSpace( mKinect, depthPos.size(), &depthPos[ 0 ], depthPos.size(), &depthVal[ 0 ], camera.size(), &camera[ 0 ] );
		for_each( camera.begin(), camera.end(), [ &p ]( const CameraSpacePoint& i )
		{
			p.push_back( toVec3( i ) );
		} );
	}
	return p;
}

vector<vec3> Device::mapDepthToCamera( const Channel16uRef& depth ) const
{
	vector<vec3> p;
	if ( depth ) {
		vector<CameraSpacePoint> camera( depth->getWidth() * depth->getHeight() );
		KCBMapDepthFrameToCameraSpace( mKinect, camera.size(), depth->getData(), camera.size(), &camera[ 0 ] );
		for_each( camera.begin(), camera.end(), [ &p ]( const CameraSpacePoint& i )
		{
			p.push_back( toVec3( i ) );
		} );
	}
	return p;
}

Surface32fRef Device::mapDepthToCameraTable() const
{
	ivec2 sz = DepthFrame().getSize();
	Surface32fRef surface = Surface32f::create( sz.x, sz.y, false, SurfaceChannelOrder::RGB );

	PointF* table	= nullptr;
	uint32_t count	= 0;
	long hr			= GetDepthFrameToCameraSpaceTable( mKinect, &count, &table );
	if ( SUCCEEDED( hr ) ) {
		Surface32f::Iter iter = surface->getIter();

		size_t i = 0;
		while ( iter.line() ) {
			while ( iter.pixel() ) {
				iter.r() = table[ i ].X;
				iter.g() = table[ i ].Y;
				iter.b() = 0.0f;
				++i;
			}
		}
	}

	return surface;
}

ivec2 Device::mapDepthToColor( const ivec2& v, const Channel16uRef& depth ) const
{
	ColorSpacePoint p;
	if ( depth ) {
		uint16_t d = depth->getValue( v );
		KCBMapDepthPointToColorSpace( mKinect, toDepthSpacePoint( v ), d, &p ); 
	}
	return ivec2( toVec2( p ) );
}

vector<ivec2> Device::mapDepthToColor( const Channel16uRef& depth ) const
{
	vector<ivec2> p;
	if ( depth ) {
		vector<ColorSpacePoint> color( depth->getWidth() * depth->getHeight() );
		KCBMapDepthFrameToColorSpace( mKinect, color.size(), depth->getData(), color.size(), &color[ 0 ] );
		for_each( color.begin(), color.end(), [ &p ]( const ColorSpacePoint& i )
		{
			p.push_back( ivec2( toVec2( i ) ) );
		} );
	}
	return p;
}

vector<ivec2> Device::mapDepthToColor( const vector<ivec2>& v, const Channel16uRef& depth ) const
{
	vector<ivec2> p;
	if ( depth ) {
		vector<ColorSpacePoint> color( v.size() );
		vector<DepthSpacePoint> depthPos;
		vector<uint16_t> depthVal;
		for_each( v.begin(), v.end(), [ &depth, &depthPos, &depthVal ]( const ivec2& i )
		{
			depthPos.push_back( toDepthSpacePoint( i ) );
			depthVal.push_back( depth->getValue( i ) );
		} );
		KCBMapDepthPointsToColorSpace( mKinect, depthPos.size(), &depthPos[ 0 ], depthPos.size(), &depthVal[ 0 ], color.size(), &color[ 0 ] );
		for_each( color.begin(), color.end(), [ &p ]( const ColorSpacePoint& i )
		{
			p.push_back( ivec2( toVec2( i ) ) );
		} );
	}
	return p;
}

void Device::start()
{
	long hr = S_OK;

	//IKinectSensorCollection* sensorCollection = nullptr;
	//hr = GetKinectSensorCollection( &sensorCollection );
	//if ( FAILED( hr ) || sensorCollection == 0 ) {
	//	throw ExcDeviceNotAvailable( hr );
	//}
	//if ( sensorCollection != nullptr ) {
	//	sensorCollection->Release();
	//	sensorCollection = nullptr;
	//}

	mKinect = KCBOpenDefaultSensor();
	if ( mKinect == KCB_INVALID_HANDLE ) {
		throw ExcDeviceOpenFailed();
	}

	uint8_t sensorIsOpen = isSensorOpen();
	for ( size_t frameType = (size_t)FrameType_Audio; frameType < (size_t)FrameType_InfraredLongExposure; ++frameType ) {
		mProcesses[ (FrameType)frameType ]	= Process();
		Process& process					= mProcesses.at( (FrameType)frameType );
		switch( (FrameType)frameType ) {
		case FrameType_Audio:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerAudio == nullptr  ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_Audio ) ) {
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
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerBody == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_Body ) ) {		
						BodyFrame frame;
						int64_t timeStamp					= 0L;
						IBody* kinectBodies[ BODY_COUNT ]	= { 0 };

						long hr = KCBGetBodyData( mKinect, BODY_COUNT, kinectBodies, &timeStamp );
						if ( SUCCEEDED( hr ) ) {
							for ( uint8_t i = 0; i < BODY_COUNT; ++i ) {
								IBody* kinectBody = kinectBodies[ i ];
								if ( kinectBody != nullptr ) {
									Body body;
									body.mIndex			= i;
									uint8_t isTracked	= false;
									hr					= kinectBody->get_IsTracked( &isTracked );
									if ( SUCCEEDED( hr ) && isTracked ) {
										body.mTracked = true;

										if ( mEnabledJointTracking ) {
											Joint joints[ JointType_Count ];
											kinectBody->GetJoints( JointType_Count, joints );

											JointOrientation jointOrientations[ JointType_Count ];
											kinectBody->GetJointOrientations( JointType_Count, jointOrientations );

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
													toVec3( joints[ j ].Position ), 
													toQuat( jointOrientations[ j ].Orientation ), 
													joints[ j ].TrackingState, 
													parentJoint
													);
												body.mJointMap.insert( pair<JointType, Body::Joint>( static_cast<JointType>( j ), joint ) );
											}
										}
										
										PointF lean;
										kinectBody->get_Engaged( &body.mEngaged );
										kinectBody->get_Lean( &lean );
										kinectBody->get_LeanTrackingState( &body.mLeanTrackingState );
										kinectBody->get_TrackingId( &body.mId );

										body.mLean = toVec2( lean );
										
										DetectionResult activities[ Activity_Count ];
										kinectBody->GetActivityDetectionResults( (UINT)Activity_Count, activities );
										for ( size_t j = 0; j < (size_t)Activity_Count; ++j ) {
											body.mActivities[ (Activity)j ] = activities[ j ];
										}

										DetectionResult appearances[ Appearance_Count ];
										kinectBody->GetAppearanceDetectionResults( (UINT)Appearance_Count, appearances );
										for ( size_t j = 0; j < (size_t)Appearance_Count; ++j ) {
											body.mAppearances[ (Appearance)i ] = appearances[ j ];
										}

										DetectionResult expressions[ Expression_Count ];
										kinectBody->GetExpressionDetectionResults( (UINT)Expression_Count, expressions );
										for ( size_t j = 0; j < (size_t)Expression_Count; ++j ) {
											body.mExpressions[ (Expression)j ] = expressions[ j ];
										}

										if ( mEnabledHandTracking ) {
											kinectBody->get_HandLeftConfidence( &body.mHands[ 0 ].mConfidence );
											kinectBody->get_HandLeftState( &body.mHands[ 0 ].mState );
											kinectBody->get_HandRightConfidence( &body.mHands[ 1 ].mConfidence );
											kinectBody->get_HandRightState( &body.mHands[ 1 ].mState );
										}
									}
									kinectBody->Release();
									kinectBody = nullptr;

									frame.mBodies.push_back( body );
								}
							}
							frame.mTimeStamp = static_cast<long long>( timeStamp );
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
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerBodyIndex == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_BodyIndex ) ) {
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
									frame.mChannel		= Channel8u::create( w, h );
									uint32_t capacity	= (uint32_t)( w * h );
									uint8_t* buffer		= frame.mChannel->getData();
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
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerColor == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_Color ) ) {
						ColorFrame frame;
						KCBFrameDescription frameDescription;
						int64_t timeStamp	= 0L;
		
						long hr = KCBGetColorFrameDescription( mKinect, ColorImageFormat_Bgra, &frameDescription );
						if ( SUCCEEDED( hr ) ) {
							IColorFrame* colorFrame = nullptr;
							frame.mFovDiagonal		= frameDescription.diagonalFieldOfView;
							frame.mFovHorizontal	= frameDescription.horizontalFieldOfView;
							frame.mFovVertical		= frameDescription.verticalFieldOfView;
							frame.mSize				= ivec2( frameDescription.width, frameDescription.height );
							hr = KCBGetIColorFrame( mKinect, &colorFrame );
							if ( SUCCEEDED( hr ) ) {
								hr = colorFrame->get_RelativeTime( &timeStamp );
								if ( SUCCEEDED( hr ) ) {
									frame.mTimeStamp	= static_cast<long long>( timeStamp );
									frame.mSurface		= Surface8u::create( frame.getSize().x, frame.getSize().y, false, SurfaceChannelOrder::BGRA );
									uint32_t capacity	= frame.getSize().x * frame.getSize().y * frameDescription.bytesPerPixel;
									uint8_t* buffer		= frame.mSurface->getData();
									colorFrame->CopyConvertedFrameDataToArray( capacity, buffer, ColorImageFormat_Bgra );
								}
							}
							if ( colorFrame != nullptr ) {
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
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerDepth == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_Depth ) ) {
						DepthFrame frame;
						KCBFrameDescription frameDescription;
						int64_t timeStamp = 0L;
		
						long hr = KCBGetDepthFrameDescription( mKinect, &frameDescription );
						if ( SUCCEEDED( hr ) ) {
							IDepthFrame* depthFrame = nullptr;

							frame.mFovDiagonal		= frameDescription.diagonalFieldOfView;
							frame.mFovHorizontal	= frameDescription.horizontalFieldOfView;
							frame.mFovVertical		= frameDescription.verticalFieldOfView;
							frame.mSize				= ivec2( frameDescription.width, frameDescription.height );
							hr = KCBGetIDepthFrame( mKinect, &depthFrame );
							if ( SUCCEEDED( hr ) ) {
								hr = depthFrame->get_RelativeTime( &timeStamp );
								if ( SUCCEEDED( hr ) ) {
									frame.mTimeStamp	= static_cast<long long>( timeStamp );
									frame.mChannel		= Channel16u::create( frame.getSize().x, frame.getSize().y );
									uint32_t capacity	= frame.getSize().x * frame.getSize().y;
									uint16_t* buffer	= frame.mChannel->getData();
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
		case FrameType_Face2d:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerFace2d == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_Body ) ) {		
						Face2dFrame frame;
						int64_t timeStamp					= 0L;
						IBody* kinectBodies[ BODY_COUNT ]	= { 0 };

						bool newFaces	= false;
						long hr			= KCBGetBodyData( mKinect, BODY_COUNT, kinectBodies, &timeStamp );
						if ( SUCCEEDED( hr ) ) {
							uint8_t index = 0;
							for ( FaceDataRef& iter : mFaceData ) {
								IBody* kinectBody = kinectBodies[ index ];
								if ( kinectBody != nullptr ) {
									Face2d face;
									face.mIndex = index;
									uint8_t isTracked	= false;
									hr					= kinectBody->get_IsTracked( &isTracked );
									if ( SUCCEEDED( hr )								&& 
										 isTracked										&& 
										 iter->mFaceFrameSource2d != nullptr	&& 
										 iter->mFaceFrameReader2d != nullptr ) {
										kinectBody->get_TrackingId( &face.mId );
										IFaceFrame* faceFrame	= nullptr;
										hr						= iter->mFaceFrameReader2d->AcquireLatestFrame( &faceFrame );
										if ( SUCCEEDED( hr ) && faceFrame != nullptr ) {
											uint8_t trackingIdValid	= 0;
											hr						= faceFrame->get_IsTrackingIdValid( &trackingIdValid );
											if ( SUCCEEDED( hr ) && trackingIdValid != 0 ) {
												IFaceFrameResult* faceFrameResult	= nullptr;
												hr									= faceFrame->get_FaceFrameResult( &faceFrameResult );
												newFaces							= true;
												if ( SUCCEEDED( hr ) && faceFrameResult != nullptr ) {
													face.mTracked	= true;
	
													RectI faceRectColor = { 0 };
													hr = faceFrameResult->get_FaceBoundingBoxInColorSpace( &faceRectColor );
													if ( SUCCEEDED( hr ) ) {
														face.mBoundsColor = toRectf( faceRectColor );
													}

													RectI faceRectInfrared = { 0 };
													hr = faceFrameResult->get_FaceBoundingBoxInInfraredSpace( &faceRectInfrared );
													if ( SUCCEEDED( hr ) ) {	
														face.mBoundsInfrared = toRectf( faceRectInfrared );
													}
																
													PointF facePointsColor[ FacePointType::FacePointType_Count ];
													hr = faceFrameResult->GetFacePointsInColorSpace( FacePointType_Count, facePointsColor );
													if ( SUCCEEDED( hr ) ) {
														for ( size_t i = 0; i < (size_t)FacePointType_Count; ++i ) {
															face.mPointsColor.push_back( toVec2( facePointsColor[ i ] ) );
														}
													}
																
													PointF facePointsInfrared[ FacePointType::FacePointType_Count ];
													hr = faceFrameResult->GetFacePointsInInfraredSpace( FacePointType_Count, facePointsInfrared );
													if ( SUCCEEDED( hr ) ) {
														for ( size_t i = 0; i < (size_t)FacePointType_Count; ++i ) {
															face.mPointsInfrared.push_back( toVec2( facePointsInfrared[ i ] ) );
														}
													}

													Vector4 faceRotation;
													hr = faceFrameResult->get_FaceRotationQuaternion( &faceRotation );
													if ( SUCCEEDED( hr ) ) {
														face.mRotation = toQuat( faceRotation );
													}

													DetectionResult faceProperties[ FaceProperty::FaceProperty_Count ];
													hr = faceFrameResult->GetFaceProperties( FaceProperty_Count, faceProperties );
													if ( SUCCEEDED( hr ) ) {
														for ( size_t i = 0; i < (size_t)FaceProperty_Count; ++i ) {
															face.mFaceProperties[ (FaceProperty)i ] = faceProperties[ i ];
														}
													}
													faceFrameResult->Release();
													faceFrameResult = nullptr;
												}
											}
											faceFrame->Release();
											faceFrame = nullptr;
										} else {
											iter->mFaceFrameSource2d->put_TrackingId( face.getId() );
										}
									}
									kinectBody->Release();
									kinectBody = nullptr;

									frame.mFaces.push_back( face );
								}
								++index;
							}
							if ( newFaces ) {
								frame.mTimeStamp = static_cast<long long>( timeStamp );
							}
						}
						if ( frame.getTimeStamp() > mFrameFace2d.getTimeStamp() ) {
							mFrameFace2d		= frame;
							process.mNewData	= true;
						}
					}
				}
			};
			break;
		case FrameType_Face3d:
			process.mThreadCallback = [ & ]()
			{
				while ( process.mRunning ) {
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerFace3d == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_Body ) ) {		
						Face3dFrame frame;
						int64_t timeStamp					= 0L;
						IBody* kinectBodies[ BODY_COUNT ]	= { 0 };

						bool newFaces	= false;
						long hr			= KCBGetBodyData( mKinect, BODY_COUNT, kinectBodies, &timeStamp );
						if ( SUCCEEDED( hr ) ) {
							uint8_t index = 0;
							for ( FaceDataRef& iter : mFaceData ) {
								IBody* kinectBody = kinectBodies[ index ];
								if ( kinectBody != nullptr ) {
									Face3d face;
									face.mIndex = index;
									uint8_t isTracked	= false;
									hr					= kinectBody->get_IsTracked( &isTracked );
									if ( SUCCEEDED( hr )						&& 
										 isTracked								&& 
										 iter->mFaceFrameSource3d != nullptr	&& 
										 iter->mFaceFrameReader3d != nullptr ) {
										kinectBody->get_TrackingId( &face.mId );
										IHighDefinitionFaceFrame * faceFrame = nullptr;
										hr = iter->mFaceFrameReader3d->AcquireLatestFrame( &faceFrame );
										if ( SUCCEEDED( hr ) && faceFrame != nullptr ) {
											uint8_t trackingIdValid	= 0;
											hr						= faceFrame->get_IsTrackingIdValid( &trackingIdValid );
											if ( SUCCEEDED( hr )		&& 
												 trackingIdValid != 0	&& 
												 iter->mFaceAlignment != nullptr ) {
												hr			= faceFrame->GetAndRefreshFaceAlignmentResult( iter->mFaceAlignment );
												newFaces	= true;
												if ( SUCCEEDED( hr ) ) {
													face.mTracked = true;

													RectI faceRect	= { 0 };
													hr				= iter->mFaceAlignment->get_FaceBoundingBox( &faceRect );
													if ( SUCCEEDED( hr ) ) {
														face.mBounds = toRectf( faceRect );
													}

													FaceAlignmentQuality faceAlignmentQuality	= FaceAlignmentQuality_Low;
													hr											= iter->mFaceAlignment->get_Quality( &faceAlignmentQuality );
													if ( SUCCEEDED( hr ) ) {
														face.mFaceAlignmentQuality = faceAlignmentQuality;
													}

													float faceShapeAnimations[ FaceShapeAnimations_Count ];
													hr = iter->mFaceAlignment->GetAnimationUnits( FaceShapeAnimations_Count, faceShapeAnimations );
													if ( SUCCEEDED( hr ) ) {
														for ( size_t i = 0; i < (size_t)FaceShapeAnimations_Count; ++i ) {
															face.mFaceShapeAnimations[ (FaceShapeAnimations)i ] = faceShapeAnimations[ i ];
														}
													}
																
													CameraSpacePoint headPivotPoint;
													hr = iter->mFaceAlignment->get_HeadPivotPoint( &headPivotPoint );
													if ( SUCCEEDED( hr ) ) {
														face.mHeadPivotPoint = toVec3( headPivotPoint );
													}

													Vector4 faceOrientation;
													hr = iter->mFaceAlignment->get_FaceOrientation( &faceOrientation );
													if ( SUCCEEDED( hr ) ) {
														face.mOrientation = toQuat( faceOrientation );
													}

													if ( !iter->mFaceModelProduced && iter->mFaceModelBuilder != nullptr ) {
														FaceModelBuilderCollectionStatus status;
														hr = iter->mFaceModelBuilder->get_CollectionStatus( &status );
														if ( status == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete ) {
															IFaceModelData* faceModelData	= nullptr;
															hr								= iter->mFaceModelBuilder->GetFaceData( &faceModelData );
															if( SUCCEEDED( hr ) && faceModelData != nullptr ) {
																hr = faceModelData->ProduceFaceModel( &iter->mFaceModel );
																if ( SUCCEEDED( hr ) && iter->mFaceModel != nullptr ) {
																	iter->mFaceModelProduced = true;
																}
															}
															if ( faceModelData != nullptr ) {
																faceModelData->Release();
																faceModelData = nullptr;
															}
														}
													}

													if ( SUCCEEDED( hr ) && iter->mFaceModel != nullptr ) {
														uint32_t hairColor	= 0x00000000;
														hr					= iter->mFaceModel->get_HairColor( &hairColor );
														if ( SUCCEEDED( hr ) ) {
															face.mColorHair = ColorA8u::hexA( hairColor );
														}

														uint32_t skinColor	= 0x00000000;
														hr					= iter->mFaceModel->get_SkinColor( &skinColor );
														if ( SUCCEEDED( hr ) ) {
															face.mColorSkin = ColorA8u::hexA( skinColor );
														}

														hr = iter->mFaceModel->GetFaceShapeDeformations( FaceShapeDeformations_Count, mFaceShapeDeformations );
														if ( SUCCEEDED( hr ) ) {
															for ( size_t j = 0; j < (size_t)FaceShapeDeformations_Count; ++j ) {
																face.mFaceShapeDeformations[ (FaceShapeDeformations)j ] = mFaceShapeDeformations[ j ];
															}
														}

														float scale = 0.0f;
														hr			= iter->mFaceModel->get_Scale( &scale );
														if ( SUCCEEDED( hr ) ) {
															face.mScale = scale;
														}

														if ( mEnabledFaceMesh && sFaceModelIndexCount > 0 && sFaceModelVertexCount > 0 ) {
															hr = iter->mFaceModel->CalculateVerticesForAlignment( iter->mFaceAlignment, sFaceModelVertexCount, (CameraSpacePoint*)&iter->mFaceModelVertices[ 0 ] );
															if ( SUCCEEDED( hr ) ) {
																face.mMesh = TriMesh::create( TriMesh::Format().positions() ); 
																face.mMesh->appendIndices( &iter->mFaceModelIndices[ 0 ], sFaceModelIndexCount );
																face.mMesh->appendPositions( &iter->mFaceModelVertices[ 0 ], sFaceModelVertexCount );
															}
														}
													}
												}
											}
											faceFrame->Release();
											faceFrame = nullptr;
										} else {
											uint64_t id = 0L;
											kinectBody->get_TrackingId( &id );
											iter->mFaceFrameSource3d->put_TrackingId( id );
										}
									}
									kinectBody->Release();
									kinectBody = nullptr;

									frame.mFaces.push_back( face );
								}
								++index;
							}
							if ( newFaces ) {
								frame.mTimeStamp = static_cast<long long>( timeStamp );
							}
						}
						if ( frame.getTimeStamp() > mFrameFace3d.getTimeStamp() ) {
							mFrameFace3d		= frame;
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
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerInfrared == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_Infrared ) ) {
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
									frame.mChannel		= Channel16u::create( w, h );
									uint32_t capacity	= (uint32_t)( w * h );
									uint16_t* buffer	= frame.mChannel->getData();
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
					if ( process.mNewData || mKinect == KCB_INVALID_HANDLE || mEventHandlerInfraredLongExposure == nullptr ) {
						this_thread::sleep_for( chrono::milliseconds( kThreadSleepDuration ) );
						continue;
					}

					if ( KCBIsFrameReady( mKinect, FrameSourceTypes_LongExposureInfrared ) ) {
						InfraredFrame frame;
						KCBFrameDescription frameDescription;
						ivec2 sz			= ivec2( 0 );
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
									frame.mChannel		= Channel16u::create( w, h );
									uint32_t capacity	= (uint32_t)( w * h );
									uint16_t* buffer	= frame.mChannel->getData();
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
		case FrameType_Face2d:
			if ( mEventHandlerFace2d != nullptr && process.mNewData ) {
				mEventHandlerFace2d( mFrameFace2d );
				process.mNewData = false;
			}
			break;
		case FrameType_Face3d:
			if ( mEventHandlerFace3d != nullptr && process.mNewData ) {
				mEventHandlerFace3d( mFrameFace3d );
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

	if ( mSensor == nullptr ) {
		GetDefaultKinectSensor( &mSensor );
	}

	if ( mFaceData.empty() && isSensorOpen() ) {
		for ( size_t i = 0; i < BODY_COUNT; ++i ) {
			mFaceData.push_back( FaceDataRef( new FaceData( mSensor, mFaceShapeDeformations ) ) );
		}
	}
}

uint8_t Device::isSensorOpen() const
{
	uint8_t sensorIsOpen = 0;
	if ( mSensor != nullptr ) {
		long hr = mSensor->get_IsOpen( &sensorIsOpen );
		if ( SUCCEEDED( hr ) && sensorIsOpen == 0 ) {
			hr = mSensor->Open();
			if ( SUCCEEDED( hr ) ) {
				long hr = mSensor->get_IsOpen( &sensorIsOpen );
			}
		}
	}
	return sensorIsOpen;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Device::FaceData::FaceData( IKinectSensor* sensor, float* faceShapeDeformations )
: mFaceAlignment( nullptr ), mFaceFrameReader2d( nullptr ), mFaceFrameReader3d( nullptr ), 
mFaceFrameSource2d( nullptr ), mFaceFrameSource3d( nullptr ), mFaceModel( nullptr ), 
mFaceModelBuilder( nullptr ), mFaceModelProduced( false )
{
	if ( mFaceFrameSource3d == nullptr ) {
		long hr = CreateFaceFrameSource( sensor, 0, kFaceFrameFeatures, &mFaceFrameSource2d );
		if ( SUCCEEDED( hr ) && mFaceFrameSource2d != nullptr ) {
			hr = mFaceFrameSource2d->OpenReader( &mFaceFrameReader2d );
			if ( SUCCEEDED( hr ) && mFaceFrameSource2d != nullptr ) {
			}
		}

		hr = CreateHighDefinitionFaceFrameSource( sensor, &mFaceFrameSource3d );
		if ( SUCCEEDED( hr ) && mFaceFrameSource3d != nullptr ) {
			hr = mFaceFrameSource3d->OpenReader( &mFaceFrameReader3d );
			if ( SUCCEEDED( hr ) && mFaceFrameSource3d != nullptr ) {
				hr = mFaceFrameSource3d->OpenModelBuilder( FaceModelBuilderAttributes_SkinColor, &mFaceModelBuilder );
				if ( SUCCEEDED( hr ) ) {
					hr = mFaceModelBuilder->BeginFaceDataCollection();
					if ( SUCCEEDED( hr ) ) {
						hr = CreateFaceAlignment( &mFaceAlignment );
						if ( SUCCEEDED( hr ) ) {
							hr = CreateFaceModel( 1.0, FaceShapeDeformations_Count, faceShapeDeformations, &mFaceModel );
							if ( SUCCEEDED( hr ) ) {
							}
						}
					}
				}
			}
		}
	}
	mFaceModelIndices.resize( sFaceModelIndexCount );
	mFaceModelVertices.resize( sFaceModelVertexCount );
	
	GetFaceModelTriangles( Device::sFaceModelIndexCount, &mFaceModelIndices[ 0 ] );
}

Device::FaceData::~FaceData()
{
	if ( mFaceAlignment != nullptr ) {
		mFaceAlignment->Release();
		mFaceAlignment = nullptr;
	}
	if ( mFaceFrameReader2d != nullptr ) {
		mFaceFrameReader2d->Release();
		mFaceFrameReader2d = nullptr;
	}
	if ( mFaceFrameReader3d != nullptr ) {
		mFaceFrameReader3d->Release();
		mFaceFrameReader3d = nullptr;
	}
	if ( mFaceFrameSource2d != nullptr ) {
		mFaceFrameSource2d->Release();
		mFaceFrameSource2d = nullptr;
	}
	if ( mFaceFrameSource3d != nullptr ) {
		mFaceFrameSource3d->Release();
		mFaceFrameSource3d = nullptr;
	}
	if ( mFaceModel != nullptr ) {
		mFaceModel->Release();
		mFaceModel = nullptr;
	}
	if ( mFaceModelBuilder != nullptr ) {
		mFaceModelBuilder->Release();
		mFaceModelBuilder = nullptr;
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
 