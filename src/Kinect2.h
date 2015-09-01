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

#pragma once

#include "KCBv2Lib.h"
#include "Kinect.Face.h"
#include "cinder/Exception.h"
#include "cinder/Quaternion.h"
#include "cinder/Rect.h"
#include "cinder/Surface.h"
#include "cinder/TriMesh.h"
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

class Device;

ci::Channel8uRef									channel16To8( const ci::Channel16uRef& channel, uint8_t bytes = 4 );
ci::Surface8uRef									colorizeBodyIndex( const ci::Channel8uRef& bodyIndexChannel );

ci::Color8u											getBodyColor( size_t index );
size_t												getDeviceCount();
std::map<size_t, std::string>						getDeviceMap();

CameraSpacePoint									toCameraSpacePoint( const ci::vec3& v );
ColorSpacePoint										toColorSpacePoint( const ci::vec2& v );
DepthSpacePoint										toDepthSpacePoint( const ci::vec2& v );
PointF												toPointF( const ci::vec2& v );
Vector4												toVector4( const ci::quat& q );
Vector4												toVector4( const ci::vec4& v );

ci::quat											toQuat( const Vector4& v );
ci::Rectf											toRectf( const RectI& v );
ci::vec2											toVec2( const PointF& v );
ci::vec2											toVec2( const ColorSpacePoint& v );
ci::vec2											toVec2( const DepthSpacePoint& v );
ci::vec3											toVec3( const CameraSpacePoint& v );
ci::vec4											toVec4( const Vector4& v );

std::string											wcharToString( wchar_t* v );

//////////////////////////////////////////////////////////////////////////////////////////////

class IFace
{
public:
	IFace();
	IFace( const IFace& rhs );

	IFace&											operator=( const IFace& rhs );

	uint64_t										getId() const;
	uint8_t											getIndex() const;
	bool											isTracked() const;
protected:
	uint64_t										mId;
	uint8_t											mIndex;
	bool											mTracked;

	friend class									Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Face2d : public IFace
{
public:
	Face2d();
	Face2d( const Face2d& rhs );

	Face2d&											operator=( const Face2d& rhs );

	const ci::Rectf&								getBoundsColor() const;
	const ci::Rectf&								getBoundsInfrared() const;
	const std::map<FaceProperty, DetectionResult>&	getFaceProperties() const;
	const std::vector<ci::vec2>&					getPointsColor() const;
	const std::vector<ci::vec2>&					getPointsInfrared() const;
	const ci::quat&									getRotation() const;
protected:
	ci::Rectf										mBoundsColor;
	ci::Rectf										mBoundsInfrared;
	std::map<FaceProperty, DetectionResult>			mFaceProperties;
	std::vector<ci::vec2>							mPointsColor;
	std::vector<ci::vec2>							mPointsInfrared;
	ci::quat										mRotation;

	friend class									Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Face3d : public IFace
{
public:
	Face3d();
	Face3d( const Face3d& rhs );

	Face3d&											operator=( const Face3d& rhs );

	const ci::Rectf&								getBounds() const;
	FaceAlignmentQuality							getFaceAlignmentQuality() const;
	const std::map<FaceShapeAnimations, float>&		getFaceShapeAnimations() const;
	const std::map<FaceShapeDeformations, float>&	getFaceShapeDeformations() const;
	const ci::ColorA8u&								getHairColor() const;
	const ci::vec3&									getHeadPivotPoint() const;
	const ci::TriMeshRef&							getMesh() const;
	const ci::quat&									getOrientation() const;
	float											getScale() const;
	const ci::ColorA8u&								getSkinColor() const;
protected:
	ci::Rectf										mBounds;
	ci::ColorA8u									mColorHair;
	ci::ColorA8u									mColorSkin;
	FaceAlignmentQuality							mFaceAlignmentQuality;
	std::map<FaceShapeAnimations, float>			mFaceShapeAnimations;
	std::map<FaceShapeDeformations, float>			mFaceShapeDeformations;
	ci::vec3										mHeadPivotPoint;
	ci::TriMeshRef									mMesh;
	ci::quat										mOrientation;
	float											mScale;

	friend class									Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Body
{
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Hand
	{
	public:
		Hand();
		Hand( const Hand& rhs );

		Hand&											operator=( const Hand& rhs );

		TrackingConfidence								getConfidence() const;
		HandState										getState() const;
	protected:
		TrackingConfidence								mConfidence;
		HandState										mState;

		friend class									Device;
	};
	
	//////////////////////////////////////////////////////////////////////////////////////////////

	class Joint
	{
	public:
		Joint();
		Joint( const Joint& rhs );

		Joint&											operator=( const Joint& rhs );
		
		const ci::quat&									getOrientation() const;
		JointType										getParentJoint() const;
		const ci::vec3&									getPosition() const;
		TrackingState									getTrackingState() const;
	protected:
		Joint( const ci::vec3& position, const ci::quat& orientation, 
			TrackingState trackingState, JointType parentJoint );
		
		ci::quat										mOrientation;
		JointType										mParentJoint;
		ci::vec3										mPosition;
		TrackingState									mTrackingState;

		friend class									Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	Body();
	Body( const Body& rhs );

	Body&												operator=( const Body& rhs );

	float												calcConfidence( bool weighted = false ) const;

	const std::map<Activity, DetectionResult>&			getActivities() const;
	const std::map<Appearance, DetectionResult>&		getAppearances() const;
	const std::map<Expression, DetectionResult>&		getExpressions() const;
	const Hand&											getHandLeft() const;
	const Hand&											getHandRight() const;
	uint64_t											getId() const;
	uint8_t												getIndex() const;
	const std::map<JointType, Body::Joint>&				getJointMap() const;
	const ci::vec2&										getLean() const;
	TrackingState										getLeanTrackingState() const;
	DetectionResult										isEngaged() const;
	bool												isRestricted() const;
	bool												isTracked() const;
protected:
	std::map<Activity, DetectionResult>					mActivities;
	std::map<Appearance, DetectionResult>				mAppearances;
	DetectionResult										mEngaged;
	std::map<Expression, DetectionResult>				mExpressions;
	Hand												mHands[ 2 ];
	uint64_t											mId;
	uint8_t												mIndex;
	std::map<JointType, Body::Joint>					mJointMap;
	ci::vec2											mLean;
	TrackingState										mLeanTrackingState;
	bool												mRestricted;
	bool												mTracked;

	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Frame
{
public:
	Frame();
	Frame( const Frame& rhs );

	Frame&												operator=( const Frame& rhs );

	long long											getTimeStamp() const;
protected:
	long long											mTimeStamp;

	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class CameraFrame
{
public:
	CameraFrame();
	CameraFrame( const CameraFrame& rhs );

	CameraFrame&										operator=( const CameraFrame& rhs );

	float												getFovDiagonal() const;
	float												getFovHorizontal() const;
	float												getFovVertical() const;
	const ci::ivec2&									getSize() const;
protected:
	float												mFovDiagonal;
	float												mFovHorizontal;
	float												mFovVertical;
	ci::ivec2											mSize;

	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class AudioFrame : public Frame
{
public:
	AudioFrame();
	AudioFrame( const AudioFrame& rhs );
	
	AudioFrame&											operator=( const AudioFrame& rhs );

	~AudioFrame();

	float												getBeamAngle() const;
	float												getBeamAngleConfidence() const;
	uint8_t*											getBuffer() const;
	unsigned long										getBufferSize() const;
	WAVEFORMATEX										getFormat() const;
protected:
	float												mBeamAngle;
	float												mBeamAngleConfidence;
	uint8_t*											mBuffer;
	unsigned long										mBufferSize;
	WAVEFORMATEX										mFormat;
	
	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class BodyFrame : public Frame
{
public:
	BodyFrame();
	BodyFrame( const BodyFrame& rhs );

	BodyFrame&											operator=( const BodyFrame& rhs );

	const std::vector<Body>&							getBodies() const;
protected:
	std::vector<Body>									mBodies;

	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
class ChannelFrameT : public Frame
{
public:
	ChannelFrameT();
	ChannelFrameT( const ChannelFrameT& rhs );

	ChannelFrameT&										operator=( const ChannelFrameT& rhs );

	const std::shared_ptr<ci::ChannelT<T> >&			getChannel() const;
protected:
	std::shared_ptr<ci::ChannelT<T> >					mChannel;

	friend class										Device;
};

typedef ChannelFrameT<uint8_t>							ChannelFrame8u;
typedef ChannelFrameT<uint16_t>							ChannelFrame16u;

//////////////////////////////////////////////////////////////////////////////////////////////

class ColorFrame : public CameraFrame, public Frame
{
public:
	ColorFrame();
	ColorFrame( const ColorFrame& rhs );

	ColorFrame&											operator=( const ColorFrame& rhs );

	const ci::Surface8uRef&								getSurface() const;
protected:
	ci::Surface8uRef									mSurface;

	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class DepthFrame : public CameraFrame, public ChannelFrame16u
{
public:
	DepthFrame();
	DepthFrame( const DepthFrame& rhs );

	DepthFrame&											operator=( const DepthFrame& rhs );
protected:
	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

typedef ChannelFrame8u									BodyIndexFrame;
typedef ChannelFrame16u									InfraredFrame;

//////////////////////////////////////////////////////////////////////////////////////////////

class Face2dFrame : public Frame
{
public:
	Face2dFrame();
	Face2dFrame( const Face2dFrame& rhs );

	Face2dFrame&										operator=( const Face2dFrame& rhs );

	const std::vector<Face2d>&							getFaces() const;
protected:
	std::vector<Face2d>									mFaces;

	friend class										Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Face3dFrame : public Frame
{
public:
	Face3dFrame();
	Face3dFrame( const Face3dFrame& rhs );

	Face3dFrame&										operator=( const Face3dFrame& rhs );

	const std::vector<Face3d>&							getFaces() const;
protected:
	std::vector<Face3d>									mFaces;

	friend class										Device;
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
		Process( const Process& rhs );

		Process&										operator=( const Process& rhs );

		~Process();

		void											start();
		void											stop();
	protected:
		std::function<void ()>							mThreadCallback;

		std::atomic_bool								mNewData;
		std::atomic_bool								mRunning;
		std::shared_ptr<std::thread>					mThread;

		friend class									Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

public:
	static DeviceRef									create();
	~Device();
	
	void												start();
	void												stop();

	void												enableFaceMesh( bool enable = true );
	void												enableHandTracking( bool enable = true );
	void												enableJointTracking( bool enable = true );
	bool												isFaceMeshEnabled() const;
	bool												isHandTrackingEnabled() const;
	bool												isJointTrackingEnabled() const;

	template<typename T, typename Y>
	inline void											connectAudioEventHandler( T eventHandler, Y* obj )
	{
		connectAudioEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectBodyEventHandler( T eventHandler, Y* obj )
	{
		connectBodyEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectBodyIndexEventHandler( T eventHandler, Y* obj )
	{
		connectBodyIndexEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectColorEventHandler( T eventHandler, Y* obj )
	{
		connectColorEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectDepthEventHandler( T eventHandler, Y* obj )
	{
		connectDepthEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectFace2dEventHandler( T eventHandler, Y* obj )
	{
		connectFace2dEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectFace3dEventHandler( T eventHandler, Y* obj )
	{
		connectFace3dEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectInfraredEventHandler( T eventHandler, Y* obj )
	{
		connectInfraredEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	template<typename T, typename Y>
	inline void											connectInfraredLongExposureEventHandler( T eventHandler, Y* obj )
	{
		connectInfraredLongExposureEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	void												connectAudioEventHandler( const std::function<void ( const AudioFrame& )>& eventHandler );
	void												connectBodyEventHandler( const std::function<void ( const BodyFrame& )>& eventHandler );
	void												connectBodyIndexEventHandler( const std::function<void ( const BodyIndexFrame& )>& eventHandler );
	void												connectColorEventHandler( const std::function<void ( const ColorFrame& )>& eventHandler );
	void												connectDepthEventHandler( const std::function<void ( const DepthFrame& )>& eventHandler );
	void												connectFace2dEventHandler( const std::function<void ( const Face2dFrame& )>& eventHandler );
	void												connectFace3dEventHandler( const std::function<void ( const Face3dFrame& )>& eventHandler );
	void												connectInfraredEventHandler( const std::function<void ( const InfraredFrame& )>& eventHandler );
	void												connectInfraredLongExposureEventHandler( const std::function<void ( const InfraredFrame& )>& eventHandler );

	void												disconnectAudioEventHandler();
	void												disconnectBodyEventHandler();
	void												disconnectBodyIndexEventHandler();
	void												disconnectColorEventHandler();
	void												disconnectDepthEventHandler();
	void												disconnectFace2dEventHandler();
	void												disconnectFace3dEventHandler();
	void												disconnectInfraredEventHandler();
	void												disconnectInfraredLongExposureEventHandler();

	bool												isAudioEventHandlerConnected() const;
	bool												isBodyEventHandlerConnected() const;
	bool												isBodyIndexEventHandlerConnected() const;
	bool												isColorEventHandlerConnected() const;
	bool												isDepthEventHandlerConnected() const;
	bool												isFace2dEventHandlerConnected() const;
	bool												isFace3dEventHandlerConnected() const;
	bool												isInfraredEventHandlerConnected() const;
	bool												isInfraredLongExposureEventHandlerConnected() const;

	ci::ivec2											mapCameraToColor( const ci::vec3& v ) const;
	std::vector<ci::ivec2>								mapCameraToColor( const std::vector<ci::vec3>& v ) const;
	ci::ivec2											mapCameraToDepth( const ci::vec3& v ) const;
	std::vector<ci::ivec2>								mapCameraToDepth( const std::vector<ci::vec3>& v ) const;
	ci::vec3											mapDepthToCamera( const ci::ivec2& v, const ci::Channel16uRef& depth ) const;
	std::vector<ci::vec3>								mapDepthToCamera( const std::vector<ci::ivec2>& v, const ci::Channel16uRef& depth ) const;
	std::vector<ci::vec3>								mapDepthToCamera( const ci::Channel16uRef& depth ) const;
	ci::Surface32fRef									mapDepthToCameraTable() const;
	ci::ivec2											mapDepthToColor( const ci::ivec2& v, const ci::Channel16uRef& depth ) const;
	std::vector<ci::ivec2>								mapDepthToColor( const std::vector<ci::ivec2>& v, const ci::Channel16uRef& depth ) const;
	std::vector<ci::ivec2>								mapDepthToColor( const ci::Channel16uRef& depth ) const;
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

	virtual void										update();

	uint8_t												isSensorOpen() const;
	KCBHANDLE											mKinect;
	IKinectSensor*										mSensor;

	std::map<FrameType, Process>						mProcesses;
	
	std::function<void ( const AudioFrame& )>			mEventHandlerAudio;
	std::function<void ( const BodyFrame& )>			mEventHandlerBody;
	std::function<void ( const BodyIndexFrame& )>		mEventHandlerBodyIndex;
	std::function<void ( const ColorFrame& )>			mEventHandlerColor;
	std::function<void ( const DepthFrame& )>			mEventHandlerDepth;
	std::function<void ( const Face2dFrame& )>			mEventHandlerFace2d;
	std::function<void ( const Face3dFrame& )>			mEventHandlerFace3d;
	std::function<void ( const InfraredFrame& )>		mEventHandlerInfrared;
	std::function<void ( const InfraredFrame& )>		mEventHandlerInfraredLongExposure;

	AudioFrame											mFrameAudio;
	BodyFrame											mFrameBody;
	BodyIndexFrame										mFrameBodyIndex;
	ColorFrame											mFrameColor;
	DepthFrame											mFrameDepth;
	Face2dFrame											mFrameFace2d;
	Face3dFrame											mFrameFace3d;
	InfraredFrame										mFrameInfrared;
	InfraredFrame										mFrameInfraredLongExposure;

	bool												mEnabledFaceMesh;
	bool												mEnabledHandTracking;
	bool												mEnabledJointTracking;

	static uint32_t										sFaceModelIndexCount;
	static uint32_t										sFaceModelVertexCount;

	class FaceData
	{
	public:
		FaceData( IKinectSensor* sensor, float* faceShapeDeformations );
		~FaceData();

		IFaceAlignment*									mFaceAlignment;
		IFaceFrameReader*								mFaceFrameReader2d;
		IHighDefinitionFaceFrameReader*					mFaceFrameReader3d;
		IFaceFrameSource*								mFaceFrameSource2d;
		IHighDefinitionFaceFrameSource*					mFaceFrameSource3d;
		IFaceModel*										mFaceModel;
		IFaceModelBuilder*								mFaceModelBuilder;
		bool											mFaceModelProduced;
		std::vector<uint32_t>							mFaceModelIndices;
		std::vector<ci::vec3>							mFaceModelVertices;

		friend class									Device;
	};
	typedef std::shared_ptr<FaceData>					FaceDataRef;

	std::list<FaceDataRef>								mFaceData;
	float												mFaceShapeDeformations[ FaceShapeDeformations::FaceShapeDeformations_Count ];
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Exception : public ci::Exception
	{
	public:
		const char* what() const throw();
	protected:
		char											mMessage[ 2048 ];

		friend class									Device;
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
