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

#pragma once

#include "KCBv2Lib.h"
#include <Kinect.Face.h>
#include "cinder/Exception.h"
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include <atomic>
#include <functional>
#include <map>
#include <thread>
#include "ole2.h"

#if defined( _DEBUG )
#pragma comment( lib, "comsuppwd.lib" )
#else
#pragma comment( lib, "comsuppw.lib" )
#endif
#pragma comment( lib, "wbemuuid.lib" )

namespace Kinect2 {

ci::Channel8u													channel16To8( const ci::Channel16u& channel, uint8_t bytes = 4 );
ci::Surface8u													colorizeBodyIndex( const ci::Channel8u& bodyIndexChannel );

ci::Color8u														getBodyColor( size_t index );
size_t															getDeviceCount();
std::map<size_t, std::string>									getDeviceMap();

CameraSpacePoint												toCameraSpacePoint( const ci::Vec3f& v );
ColorSpacePoint													toColorSpacePoint( const ci::Vec2f& v );
DepthSpacePoint													toDepthSpacePoint( const ci::Vec2f& v );
PointF															toPointF( const ci::Vec2f& v );
Vector4															toVector4( const ci::Quatf& q );
Vector4															toVector4( const ci::Vec4f& v );

ci::Quatf														toQuatf( const Vector4& v );
ci::Vec2f														toVec2f( const PointF& v );
ci::Vec2f														toVec2f( const ColorSpacePoint& v );
ci::Vec2f														toVec2f( const DepthSpacePoint& v );
ci::Vec3f														toVec3f( const CameraSpacePoint& v );
ci::Vec4f														toVec4f( const Vector4& v );

std::string														wcharToString( wchar_t* v );

//////////////////////////////////////////////////////////////////////////////////////////////

class Device;
	
//////////////////////////////////////////////////////////////////////////////////////////////

class Body
{
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Hand
	{
	public:
		Hand();

		TrackingConfidence										getConfidence() const;
		HandState												getState() const;
	protected:
		TrackingConfidence										mConfidence;
		HandState												mState;
		friend class											Device;
	};
	
	//////////////////////////////////////////////////////////////////////////////////////////////

	class Joint
	{
	public:
		Joint();
		
		uint64_t												getId() const;
		const ci::Quatf&										getOrientation() const;
		JointType												getParentJoint() const;
		const ci::Vec3f&										getPosition() const;
		TrackingState											getTrackingState() const;
	protected:
		Joint( const ci::Vec3f& position, const ci::Quatf& orientation, 
			TrackingState trackingState, JointType parentJoint );
		
		ci::Quatf												mOrientation;
		JointType												mParentJoint;
		ci::Vec3f												mPosition;
		TrackingState											mTrackingState;

		friend class											Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	Body();

	float														calcConfidence( bool weighted = false ) const;

	const std::map<Activity, DetectionResult>&					getActivities() const;
	const std::map<Appearance, DetectionResult>&				getAppearances() const;
	const std::map<Expression, DetectionResult>&				getExpressions() const;
	const Hand&													getHandLeft() const;
	const Hand&													getHandRight() const;
	uint64_t													getId() const;
	uint8_t														getIndex() const;
	const std::map<JointType, Body::Joint>&						getJointMap() const;
	const ci::Vec2f&											getLean() const;
	TrackingState												getLeanTrackingState() const;
	DetectionResult												isEngaged() const;
	bool														isRestricted() const;
	bool														isTracked() const;
protected:
	std::map<Activity, DetectionResult>							mActivities;
	std::map<Appearance, DetectionResult>						mAppearances;
	DetectionResult												mEngaged;
	std::map<Expression, DetectionResult>						mExpressions;
	Hand														mHands[ 2 ];
	uint64_t													mId;
	uint8_t														mIndex;
	std::map<JointType, Body::Joint>							mJointMap;
	ci::Vec2f													mLean;
	TrackingState												mLeanTrackingState;
	bool														mRestricted;
	bool														mTracked;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Face2d
{
public:
	Face2d();
protected:
	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Face3d
{
public:
	Face3d();
protected:
	friend class												Device;
};
//////////////////////////////////////////////////////////////////////////////////////////////

class Frame
{
public:
	Frame();

	long long													getTimeStamp() const;
protected:
	long long													mTimeStamp;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class CameraFrame
{
public:
	CameraFrame();
	float														getFovDiagonal() const;
	float														getFovHorizontal() const;
	float														getFovVertical() const;
	const ci::Vec2i&											getSize() const;
protected:
	float														mFovDiagonal;
	float														mFovHorizontal;
	float														mFovVertical;
	ci::Vec2i													mSize;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class AudioFrame : public Frame
{
public:
	AudioFrame();
	~AudioFrame();

	float														getBeamAngle() const;
	float														getBeamAngleConfidence() const;
	uint8_t*													getBuffer() const;
	unsigned long												getBufferSize() const;
	WAVEFORMATEX												getFormat() const;
protected:
	float														mBeamAngle;
	float														mBeamAngleConfidence;
	uint8_t*													mBuffer;
	unsigned long												mBufferSize;
	WAVEFORMATEX												mFormat;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
class ChannelFrameT : public Frame
{
public:
	ChannelFrameT();

	const ci::ChannelT<T>&										getChannel() const;
protected:
	ci::ChannelT<T>												mChannel;

	friend class												Device;
};

typedef ChannelFrameT<uint8_t>									ChannelFrame8u;
typedef ChannelFrameT<uint16_t>									ChannelFrame16u;

//////////////////////////////////////////////////////////////////////////////////////////////

class ColorFrame : public CameraFrame, public Frame
{
public:
	ColorFrame();

	const ci::Surface8u&										getSurface() const;
protected:
	ci::Surface8u												mSurface;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class DepthFrame : public CameraFrame, public ChannelFrame16u
{
public:
	DepthFrame();
protected:
	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

typedef ChannelFrame8u											BodyIndexFrame;
typedef ChannelFrame16u											InfraredFrame;
typedef ChannelFrame16u											InfraredLongExposureFrame;

//////////////////////////////////////////////////////////////////////////////////////////////

class BodyFrame : public Frame
{
public:
	BodyFrame();

	const std::vector<Body>&									getBodies() const;
protected:
	std::vector<Body>											mBodies;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Face2dFrame : public Frame
{
public:
	Face2dFrame();
	const std::vector<Face2d>&									getFaces() const;
protected:
	std::vector<Face2d>											mFaces;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Face3dFrame : public Frame
{
public:
	Face3dFrame();
	const std::vector<Face3d>&									getFaces() const;
protected:
	std::vector<Face3d>											mFaces;

	friend class												Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

typedef std::shared_ptr<Device>	DeviceRef;

class Device
{
protected:

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	class Process
	{
	public:
		Process();
		~Process();

		void													start();
		void													stop();
	protected:
		std::function<void ()>									mThreadCallback;

		std::atomic_bool										mNewData;
		std::atomic_bool										mRunning;
		std::shared_ptr<std::thread>							mThread;

		friend class											Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

public:
	static DeviceRef											create();
	~Device();
	
	void														start();
	void														stop();

	template<typename T, typename Y>
	inline void													connectAudioEventHandler( T eventHandler, Y* obj )
	{
		connectAudioEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void													connectBodyEventHandler( T eventHandler, Y* obj )
	{
		connectBodyEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void													connectBodyIndexEventHandler( T eventHandler, Y* obj )
	{
		connectBodyIndexEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void													connectColorEventHandler( T eventHandler, Y* obj )
	{
		connectColorEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void													connectDepthEventHandler( T eventHandler, Y* obj )
	{
		connectDepthEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void													connectFace2dEventHandler( T eventHandler, Y* obj )
	{
		connectFace2dEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}
	
	template<typename T, typename Y>
	inline void													connectFace3dEventHandler( T eventHandler, Y* obj )
	{
		connectFace3dEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void													connectInfraredEventHandler( T eventHandler, Y* obj )
	{
		connectInfraredEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void													connectInfraredLongExposureEventHandler( T eventHandler, Y* obj )
	{
		connectInfraredLongExposureEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	void														connectAudioEventHandler( const std::function<void ( const AudioFrame& )>& eventHandler );
	void														connectBodyEventHandler( const std::function<void ( const BodyFrame& )>& eventHandler );
	void														connectBodyIndexEventHandler( const std::function<void ( const BodyIndexFrame& )>& eventHandler );
	void														connectColorEventHandler( const std::function<void ( const ColorFrame& )>& eventHandler );
	void														connectDepthEventHandler( const std::function<void ( const DepthFrame& )>& eventHandler );
	void														connectFace2dEventHandler( const std::function<void ( const Face2dFrame& )>& eventHandler );
	void														connectFace3dEventHandler( const std::function<void ( const Face3dFrame& )>& eventHandler );
	void														connectInfraredEventHandler( const std::function<void ( const InfraredFrame& )>& eventHandler );
	void														connectInfraredLongExposureEventHandler( const std::function<void ( const InfraredLongExposureFrame& )>& eventHandler );

	void														disconnectAudioEventHandler();
	void														disconnectBodyEventHandler();
	void														disconnectBodyIndexEventHandler();
	void														disconnectColorEventHandler();
	void														disconnectDepthEventHandler();
	void														disconnectFace2dEventHandler();
	void														disconnectFace3dEventHandler();
	void														disconnectInfraredEventHandler();
	void														disconnectInfraredLongExposureEventHandler();

	ci::Vec2i													mapCameraToColor( const ci::Vec3f& v ) const;
	std::vector<ci::Vec2i>										mapCameraToColor( const std::vector<ci::Vec3f>& v ) const;
	ci::Vec2i													mapCameraToDepth( const ci::Vec3f& v ) const;
	std::vector<ci::Vec2i>										mapCameraToDepth( const std::vector<ci::Vec3f>& v ) const;
	ci::Vec3f													mapDepthToCamera( const ci::Vec2i& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec3f>										mapDepthToCamera( const std::vector<ci::Vec2i>& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec3f>										mapDepthToCamera( const ci::Channel16u& depth ) const;
	ci::Vec2i													mapDepthToColor( const ci::Vec2i& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec2i>										mapDepthToColor( const std::vector<ci::Vec2i>& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec2i>										mapDepthToColor( const ci::Channel16u& depth ) const;
protected:
	enum : size_t
	{
		FrameType_Audio,
		FrameType_Body, 
		FrameType_BodyIndex, 
		FrameType_Color, 
		FrameType_Depth, 
		FrameType_Face2d, 
		FrameType_Face3d, 
		FrameType_Infrared, 
		FrameType_InfraredLongExposure
	} typedef FrameType;
	
	Device();

	virtual void												update();

	KCBHANDLE													mKinect;
	
	std::map<FrameType, Process>								mProcesses;
	
	std::function<void ( const AudioFrame& )>					mEventHandlerAudio;
	std::function<void ( const BodyFrame& )>					mEventHandlerBody;
	std::function<void ( const BodyIndexFrame& )>				mEventHandlerBodyIndex;
	std::function<void ( const ColorFrame& )>					mEventHandlerColor;
	std::function<void ( const DepthFrame& )>					mEventHandlerDepth;
	std::function<void ( const Face2dFrame& )>					mEventHandlerFace2d;
	std::function<void ( const Face3dFrame& )>					mEventHandlerFace3d;
	std::function<void ( const InfraredFrame& )>				mEventHandlerInfrared;
	std::function<void ( const InfraredLongExposureFrame& )>	mEventHandlerInfraredLongExposure;

	AudioFrame													mFrameAudio;
	BodyFrame													mFrameBody;
	BodyIndexFrame												mFrameBodyIndex;
	ColorFrame													mFrameColor;
	DepthFrame													mFrameDepth;
	Face2dFrame													mFrameFace2d;
	Face3dFrame													mFrameFace3d;
	InfraredFrame												mFrameInfrared;
	InfraredLongExposureFrame									mFrameInfraredLongExposure;
	
	// TODO use KCB equivalents when they become available
	IBodyFrameReader*											mBodyFrameReader;
	IFaceFrameReader*											mFaceFrameReader;
	IFaceFrameSource*											mFaceFrameSource;
	IHighDefinitionFaceFrameReader*								mHighDefinitionFaceFrameReader;
	IHighDefinitionFaceFrameSource*								mHighDefinitionFaceFrameSource;
	IKinectSensor*												mSensor;
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Exception : public ci::Exception
	{
	public:
		const char* what() const throw();
	protected:
		char													mMessage[ 2048 ];

		friend class											Device;
	};

	class ExcDeviceCloseFailed : public Exception 
	{
	public:
		ExcDeviceCloseFailed( long hr ) throw();
	};
	
	// FOR FUTURE USE
	class ExcDeviceNotAvailable : public Exception 
	{
	public:
		ExcDeviceNotAvailable( long hr ) throw();
	};

	class ExcDeviceOpenFailed : public Exception 
	{
	public:
		ExcDeviceOpenFailed() throw();
	};
};

}
