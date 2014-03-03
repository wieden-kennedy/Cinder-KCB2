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
 #include "cinder/Exception.h"
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include <functional>
#include <map>
#include "ole2.h"

#if defined( _DEBUG )
#pragma comment( lib, "comsuppwd.lib" )
#else
#pragma comment( lib, "comsuppw.lib" )
#endif
#pragma comment( lib, "wbemuuid.lib" )

namespace Kinect2 {

ci::Channel8u									channel16To8( const ci::Channel16u& channel );
ci::Surface8u									colorizeBodyIndex( const ci::Channel8u& bodyIndexChannel );

ci::Color8u										getBodyColor( size_t index );
size_t											getDeviceCount();
std::map<size_t, std::string>					getDeviceMap();
std::string										getStatusMessage( KinectStatus status );

CameraSpacePoint								toCameraSpacePoint( const ci::Vec3f& v );
ColorSpacePoint									toColorSpacePoint( const ci::Vec2f& v );
DepthSpacePoint									toDepthSpacePoint( const ci::Vec2f& v );
PointF											toPointF( const ci::Vec2f& v );
Vector4											toVector4( const ci::Quatf& q );
Vector4											toVector4( const ci::Vec4f& v );

ci::Quatf										toQuatf( const Vector4& v );
ci::Vec2f										toVec2f( const PointF& v );
ci::Vec2f										toVec2f( const ColorSpacePoint& v );
ci::Vec2f										toVec2f( const DepthSpacePoint& v );
ci::Vec3f										toVec3f( const CameraSpacePoint& v );
ci::Vec4f										toVec4f( const Vector4& v );

std::string										wcharToString( wchar_t* v );

//////////////////////////////////////////////////////////////////////////////////////////////

class DeviceOptions
{
public:
	DeviceOptions();
	
	DeviceOptions&								enableAudio( bool enable = true );
	DeviceOptions&								enableBody( bool enable = true );
	DeviceOptions&								enableBodyIndex( bool enable = true );
	DeviceOptions&								enableColor( bool enable = true );
	DeviceOptions&								enableDepth( bool enable = true );
	DeviceOptions&								enableInfrared( bool enable = true );
	DeviceOptions&								enableInfraredLongExposure( bool enable = true );

	bool										isAudioEnabled() const;
	bool										isBodyEnabled() const;
	bool										isBodyIndexEnabled() const;
	bool										isColorEnabled() const;
	bool										isDepthEnabled() const;
	bool										isInfraredEnabled() const;
	bool										isInfraredLongExposureEnabled() const;
protected:
	bool										mEnabledAudio;
	bool										mEnabledBody;
	bool										mEnabledBodyIndex;
	bool										mEnabledColor;
	bool										mEnabledDepth;
	bool										mEnabledInfrared;
	bool										mEnabledInfraredLongExposure;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Device;

class Body
{
public:
	Body();

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Joint
	{
	public:
		Joint();
		
		const ci::Quatf&						getOrientation() const;
		const ci::Vec3f&						getPosition() const;
		TrackingState							getTrackingState() const;
	protected:
		Joint( const ci::Vec3f& position, const ci::Quatf& orientation, TrackingState trackingState );
		
		ci::Quatf								mOrientation;
		ci::Vec3f								mPosition;
		TrackingState							mTrackingState;

		friend class							Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	uint64_t									getId() const;
	uint8_t										getIndex() const;
	const std::map<JointType, Body::Joint>&		getJointMap() const;
	bool										isTracked() const;
private:
	Body( uint64_t id, uint8_t index, const std::map<JointType, Body::Joint>& jointMap );

	uint64_t									mId;
	uint8_t										mIndex;
	std::map<JointType, Body::Joint>			mJointMap;
	bool										mTracked;

	friend class								Device;
};

class Frame
{
public:
	enum : size_t
	{
		TIMESTAMP_DEFAULT, 
		TIMESTAMP_BODY, 
		TIMESTAMP_BODY_INDEX, 
		TIMESTAMP_COLOR, 
		TIMESTAMP_DEPTH, 
		TIMESTAMP_INFRARED, 
		TIMESTAMP_INFRARED_LONG_EXPOSURE
	} typedef TimeStamp;

	Frame();

	const std::vector<Body>&					getBodies() const;
	const ci::Channel8u&						getBodyIndex() const;
	const ci::Surface8u&						getColor() const;
	const ci::Channel16u&						getDepth() const;
	float										getFovDiagonal() const;
	float										getFovHorizontal() const;
	float										getFovVertical() const;
	const ci::Channel16u&						getInfrared() const;
	const ci::Channel16u&						getInfraredLongExposure() const;
	uint16_t									getMaxReliableDepthDistance() const;
	uint16_t									getMinReliableDepthDistance() const;
	long long									getTimeStamp( TimeStamp timeStamp = TimeStamp::TIMESTAMP_DEFAULT ) const;
protected:
	std::vector<Body>							mBodies;
	ci::Channel8u								mChannelBodyIndex;
	ci::Channel16u								mChannelDepth;
	ci::Channel16u								mChannelInfrared;
	ci::Channel16u								mChannelInfraredLongExposure;
	uint16_t									mDepthMaxReliableDistance;
	uint16_t									mDepthMinReliableDistance;
	float										mFovDiagonal;
	float										mFovHorizontal;
	float										mFovVertical;
	ci::Surface8u								mSurfaceColor;
	std::map<TimeStamp, long long>				mTimeStamp;

	friend class								Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

typedef std::shared_ptr<Device>	DeviceRef;

class Device
{
public:
	static DeviceRef							create();
	~Device();
	
	void										start( const DeviceOptions& deviceOptions = DeviceOptions() );
	void										stop();

	const DeviceOptions&						getDeviceOptions() const;
	const Frame&								getFrame() const;
	KinectStatus								getStatus() const;

	ci::Vec2i									mapCameraToColor( const ci::Vec3f& v ) const;
	std::vector<ci::Vec2i>						mapCameraToColor( const std::vector<ci::Vec3f>& v ) const;
	ci::Vec2i									mapCameraToDepth( const ci::Vec3f& v ) const;
	std::vector<ci::Vec2i>						mapCameraToDepth( const std::vector<ci::Vec3f>& v ) const;
	std::vector<ci::Vec3f>						mapColorToCamera( const ci::Surface8u& color ) const;
	std::vector<ci::Vec2i>						mapColorToDepth( const ci::Surface8u& color ) const;
	ci::Vec3f									mapDepthToCamera( const ci::Vec2i& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec3f>						mapDepthToCamera( const std::vector<ci::Vec2i>& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec3f>						mapDepthToCamera( const ci::Channel16u& depth ) const;
	ci::Vec2i									mapDepthToColor( const ci::Vec2i& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec2i>						mapDepthToColor( const std::vector<ci::Vec2i>& v, const ci::Channel16u& depth ) const;
	std::vector<ci::Vec2i>						mapDepthToColor( const ci::Channel16u& depth ) const;
protected:
	Device();

	virtual void								update();

	std::function<void ( Frame frame )>	mEventHandler;
	
	DeviceOptions								mDeviceOptions;
	Frame										mFrame;
	KCBHANDLE									mKinect;
	KinectStatus								mStatus;
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Exception : public ci::Exception
	{
	public:
		const char* what() const throw();
	protected:
		char									mMessage[ 2048 ];
		friend class							Device;
	};

	class ExcDeviceCloseFailed : public Exception 
	{
	public:
		ExcDeviceCloseFailed( long hr ) throw();
	};
	
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
