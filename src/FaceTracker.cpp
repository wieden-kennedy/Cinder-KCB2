/*
* 
* Copyright (c) 2013, Ban the Rewind, Wieden+Kennedy
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

#include "FaceTracker.h"
#include "cinder/app/App.h"
#include "cinder/ip/Resize.h"
#include "NuiApi.h"

namespace Kinect2
{
using namespace ci;
using namespace ci::app;
using namespace std;

Face::Face()
{
	mPoseMatrix.setToNull();
	mUserId = 0;
}

const AnimationUnitMap& Face::getAnimationUnits() const
{
	return mAnimationUnits;
}

const Rectf& Face::getBounds() const
{
	return mBounds;
}

const TriMeshRef& Face::getMesh() const
{
	return mMesh;
}

const TriMeshRef& Face::getMesh2d() const
{
	return mMesh2d;
}

const Matrix44f& Face::getPoseMatrix() const
{
	return mPoseMatrix;
}

size_t Face::getUserId() const
{
	return mUserId;
}

//////////////////////////////////////////////////////////////////////////////////////////////

FaceTrackerOptions::FaceTrackerOptions()
: mColorFovHorizontal( 0.0f ), mColorFovVertical( 0.0f ), mColorSize( 1920, 1080 ), 
mDepthFovHorizontal( 0.0f ), mDepthFovVertical( 0.0f ), mDepthSize( 512, 424 )
{
}

FaceTrackerOptions& FaceTrackerOptions::setColorFovHorizontal( float fov )
{
	mColorFovHorizontal = fov;
	return *this;
}

FaceTrackerOptions& FaceTrackerOptions::setColorFovVertical( float fov )
{
	mColorFovVertical = fov;
	return *this;
}

FaceTrackerOptions& FaceTrackerOptions::setColorSize( const Vec2i& sz )
{
	mColorSize = sz;
	return *this;
}

FaceTrackerOptions& FaceTrackerOptions::setDepthFovHorizontal( float fov )
{
	mDepthFovHorizontal = fov;
	return *this;
}

FaceTrackerOptions& FaceTrackerOptions::setDepthFovVertical( float fov )
{
	mDepthFovVertical = fov;
	return *this;
}

FaceTrackerOptions& FaceTrackerOptions::setDepthSize( const Vec2i& sz )
{
	mDepthSize = sz;
	return *this;
}

float FaceTrackerOptions::getColorFovHorizontal() const
{
	return mColorFovHorizontal;
}

float FaceTrackerOptions::getColorFovVertical() const
{
	return mColorFovVertical;
}

const Vec2i& FaceTrackerOptions::getColorSize() const
{
	return mColorSize;
}

float FaceTrackerOptions::getDepthFovHorizontal() const
{
	return mDepthFovHorizontal;
}

float FaceTrackerOptions::getDepthFovVertical() const
{
	return mDepthFovVertical;
}

const Vec2i& FaceTrackerOptions::getDepthSize() const
{
	return mDepthSize;
}

//////////////////////////////////////////////////////////////////////////////////////////////

FaceTrackerRef FaceTracker::create()
{
	return FaceTrackerRef( new FaceTracker() );
}

FaceTracker::FaceTracker()
	: mCalcMesh( true ), mCalcMesh2d( true ), mEventHandler( nullptr ), mFaceTracker( nullptr ), 
	mModel( nullptr ), mNewFace( false ), mResult( nullptr ), mRunning( false ), mUserId( 0 )
{
}

FaceTracker::~FaceTracker()
{
	stop();

	if ( mFaceTracker != nullptr ) {
		mFaceTracker->Release();
		mFaceTracker = nullptr;
	}
	if ( mSensorData.pVideoFrame != nullptr ) {
		mSensorData.pVideoFrame->Release();
		mSensorData.pVideoFrame = nullptr;
	}
	if ( mSensorData.pDepthFrame != nullptr ) {
		mSensorData.pDepthFrame->Release();
		mSensorData.pDepthFrame = nullptr;
	}
	if ( mModel != nullptr ) {
		mModel->Release();
		mModel = nullptr;
	}
	if ( mResult != nullptr ) {
		mResult->Release();
		mResult = nullptr;
	}
}

string FaceTracker::getErrorString( long hr )
{
	switch ( hr ) {
	case  FT_FACILITY:
		return "Face Tracking facility error.";
	case  FT_ERROR_INVALID_MODELS:
		return "Face tracking models loaded by the tracking engine have incorrect format.";
	case  FT_ERROR_INVALID_INPUT_IMAGE:
		return "Input image is invalid.";
	case  FT_ERROR_FACE_DETECTOR_FAILED:
		return "Face tracking failed due to face detection errors.";
	case  FT_ERROR_AAM_FAILED:
		return "Face tracking failed due to errors in tracking individual face parts.";
	case  FT_ERROR_NN_FAILED:
		return "Face tracking failed due to inability of the Neural Network to find nose, mouth corners and eyes.";
	case  FT_ERROR_UNINITIALIZED:
		return "Uninitialized face tracker is being used.";
	case  FT_ERROR_INVALID_MODEL_PATH:
		return "File path to the face model files is invalid or the model files could not be located.";
	case  FT_ERROR_EVAL_FAILED:
		return "Face tracking worked but later evaluation found that the quality of the results was poor.";
	case  FT_ERROR_INVALID_CAMERA_CONFIG:
		return "Camera configuration is invalid.";
	case  FT_ERROR_INVALID_3DHINT:
		return "3D hint vectors contain invalid values (for example out of range).";
	case  FT_ERROR_HEAD_SEARCH_FAILED:
		return "The system cannot find the head area in the passed data based on passed 3D hint vectors or region of interest rectangle.";
	case  FT_ERROR_USER_LOST:
		return "The user ID of the subject being tracked is switched or lost so we should call StartTracking on next call for tracking face.";
	case  FT_ERROR_KINECT_DLL_FAILED:
		return "Kinect DLL failed to load.";
	case  FT_ERROR_KINECT_NOT_CONNECTED:
		return "Kinect sensor was not detected in the system.";
	default:
		return "An unknown error occurred.";
	}
}

IFTFaceTracker* FaceTracker::getFaceTracker() const
{
	return mFaceTracker;
}

IFTModel* FaceTracker::getModel() const
{
	return mModel;
}

IFTResult* FaceTracker::getResult() const
{
	return mResult;
}

void FaceTracker::enableCalcMesh( bool enabled )
{
	mCalcMesh = enabled;
}

void FaceTracker::enableCalcMesh2d( bool enabled )
{
	mCalcMesh2d = enabled;
}

bool FaceTracker::isCalcMeshEnabled() const
{
	return mCalcMesh;
}

bool FaceTracker::isCalcMesh2dEnabled() const
{
	return mCalcMesh2d;
}

bool FaceTracker::isTracking() const
{
	return mRunning;
}

void FaceTracker::start( const FaceTrackerOptions& faceTrackerOptions )
{
	stop();

	long hr			= S_OK;
	
	hr = NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | 
						NUI_INITIALIZE_FLAG_USES_COLOR );
	if ( FAILED( hr ) ) {
		throw ExcFaceTrackerNuiInit( hr );
	}
	
	mFaceTracker	= FTCreateFaceTracker();
    if ( !mFaceTracker ) {
		throw ExcFaceTrackerCreate();
	}

	mConfigColor.Height			= faceTrackerOptions.getColorSize().y;
	mConfigColor.Width			= faceTrackerOptions.getColorSize().x;
	mConfigColor.FocalLength	= calcFocalLength( 
		faceTrackerOptions.getColorSize(), 
		faceTrackerOptions.getColorFovHorizontal(), 
		faceTrackerOptions.getColorFovVertical() 
		);

	mConfigDepth.Height			= faceTrackerOptions.getDepthSize().y;
	mConfigDepth.Width			= faceTrackerOptions.getDepthSize().x;
	mConfigDepth.FocalLength	= calcFocalLength( 
		faceTrackerOptions.getDepthSize(), 
		faceTrackerOptions.getDepthFovHorizontal(), 
		faceTrackerOptions.getDepthFovVertical() 
		);

	hr = mFaceTracker->Initialize( &mConfigColor, &mConfigDepth, 0, 0 );
	if ( FAILED( hr ) ) {
		throw ExcFaceTrackerInit( hr );
    }

	hr = mFaceTracker->CreateFTResult( &mResult );
	if ( FAILED( hr ) || mResult == 0 ) {
		throw ExcFaceTrackerCreateResult( hr );
	}

	tagPOINT offset;
	offset.x				= 0;
	offset.y				= 0;
	mSensorData.pDepthFrame	= FTCreateImage();
	mSensorData.pVideoFrame	= FTCreateImage();
	mSensorData.ViewOffset	= offset;
	mSensorData.ZoomFactor	= 1.0f;
	
	mRunning	= true;
	mThread		= ThreadRef( new thread( &FaceTracker::run, this ) );
}

void FaceTracker::stop()
{
	mRunning = false;
	if ( mThread ) {
		mThread->join();
		mThread.reset();
	}
	NuiShutdown();
}

void FaceTracker::update( const Surface8u& color, const Channel16u& depth, const Vec3f headPoints[ 2 ], size_t userId )
{
	if ( mNewFace && mEventHandler != nullptr ) {
		mEventHandler( mFace );
		if ( color && depth ) {
			mHeadPoints.clear();
			if ( headPoints != nullptr ) {
				mHeadPoints.push_back( headPoints[ 0 ] );
				mHeadPoints.push_back( headPoints[ 1 ] );
			}
			bool attachDepth = !mChannelDepth;
			bool attachVideo = !mSurfaceColor;

			mChannelDepth	= depth;
			mSurfaceColor	= color;
			mUserId			= userId;	

			if ( attachDepth ) {
				mSensorData.pDepthFrame->Attach( mChannelDepth.getWidth(), mChannelDepth.getHeight(), 
					(void*)mChannelDepth.getData(), FTIMAGEFORMAT_UINT16_D13P3, mChannelDepth.getRowBytes() );
			}
			if ( attachVideo ) {
				mSensorData.pVideoFrame->Attach( mSurfaceColor.getWidth(), mSurfaceColor.getHeight(), 
					(void*)mSurfaceColor.getData(), FTIMAGEFORMAT_UINT8_B8G8R8X8, mSurfaceColor.getRowBytes() );
			}
		}
		mNewFace = false;
	}
}

void FaceTracker::connectEventHander( const function<void( Face )>& eventHandler )
{
	mEventHandler = eventHandler;
}

void FaceTracker::run()
{
	while ( mRunning ) {
		if ( !mNewFace ) {
			long hr = S_OK;

			mFace.mAnimationUnits.clear();
			mFace.mBounds = Rectf( 0.0f, 0.0f, 0.0f, 0.0f );
			mFace.mMesh->clear();
			mFace.mMesh2d->clear();
			mFace.mPoseMatrix.setToIdentity();
			mFace.mUserId = mUserId;
			
			FT_VECTOR3D* hint = nullptr;
			if ( mHeadPoints.size() == 2 ) {
				hint = new FT_VECTOR3D[ 2 ];
				for ( size_t i = 0; i < 2; ++i ) {
					hint[ i ] = FT_VECTOR3D( mHeadPoints[ i ].x, mHeadPoints[ i ].y, mHeadPoints[ i ].z );
				}
			}

			if ( mSensorData.pDepthFrame != 0					&& 
				 mSensorData.pVideoFrame != 0					&& 
				 mSensorData.pDepthFrame->IsAttached()			&& 
				 mSensorData.pVideoFrame->IsAttached()			&& 
				 mSensorData.pDepthFrame->GetBufferSize() > 0	&& 
				 mSensorData.pVideoFrame->GetBufferSize() > 0 ) {
				if ( mSuccess ) {
					hr = mFaceTracker->ContinueTracking( &mSensorData, hint, mResult );
				} else {
					hr = mFaceTracker->StartTracking( &mSensorData, 0, hint, mResult );
				}
			}
		
			if ( hint != nullptr ) {
				delete [] hint;
			}

			mSuccess = SUCCEEDED( hr ) && SUCCEEDED( mResult->GetStatus() );
		
			if ( mSuccess ) {
				hr = mFaceTracker->GetFaceModel( &mModel );

				if ( SUCCEEDED( hr ) ) {
					float* shapeUnits	= 0;
					UINT numShapeUnits	= 0;
					int32_t haveConverged	= false;
					mFaceTracker->GetShapeUnits( 0, &shapeUnits, &numShapeUnits, &haveConverged );
							
					float* animationUnits;
					UINT numAnimationUnits;
					hr = mResult->GetAUCoefficients( &animationUnits, &numAnimationUnits );
					if ( SUCCEEDED( hr ) ) {
						for ( size_t i = 0; i < numAnimationUnits; ++i ) {
							mFace.mAnimationUnits[ (AnimationUnit)i ] = animationUnits[ i ];
						}
					}

					float scale;
					float rotation[ 3 ];
					float translation[ 3 ];
					hr = mResult->Get3DPose( &scale, rotation, translation );
					if ( SUCCEEDED( hr ) ) {
						Vec3f r( rotation[ 0 ], rotation[ 1 ], rotation[ 2 ] );
						Vec3f t( translation[ 0 ], translation[ 1 ], translation[ 2 ] );

						mFace.mPoseMatrix.translate( t );
						mFace.mPoseMatrix.rotate( r );
						mFace.mPoseMatrix.translate( -t );
						mFace.mPoseMatrix.translate( t );
						mFace.mPoseMatrix.scale( Vec3f::one() * scale );
					}

					UINT numVertices = mModel->GetVertexCount();
							
					if ( numAnimationUnits > 0 && numShapeUnits > 0 && numVertices > 0 ) {
						if ( mCalcMesh ) {
							FT_VECTOR3D* pts		= new FT_VECTOR3D[ numVertices ];
							pts[ numVertices - 1 ]	= 0;
							hr = mModel->Get3DShape( shapeUnits, numShapeUnits, animationUnits, numAnimationUnits, scale, rotation, translation, pts, numVertices );
							if ( SUCCEEDED( hr ) ) {
								for ( size_t i = 0; i < numVertices; ++i ) {
									Vec3f v( pts[ i ].x, pts[ i ].y, pts[ i ].z );
									mFace.mMesh->appendVertex( v );
								}

								FT_TRIANGLE* triangles;
								UINT triangleCount;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( size_t i = 0; i < triangleCount; ++i ) {
										mFace.mMesh->appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
									}
								}
							}
							delete [] pts;
						}

						if ( mCalcMesh2d ) {
							tagPOINT viewOffset	= { 0, 0 };
							FT_VECTOR2D* pts		= new FT_VECTOR2D[ numVertices ];
							pts[ numVertices - 1 ]	= 0;
							hr = mModel->GetProjectedShape( &mConfigColor, mSensorData.ZoomFactor, viewOffset, shapeUnits, numShapeUnits, animationUnits, 
								numAnimationUnits, scale, rotation, translation, pts, numVertices );
							if ( SUCCEEDED( hr ) ) {
								for ( size_t i = 0; i < numVertices; ++i ) {
									Vec2f v( pts[ i ].x + 0.5f, pts[ i ].y + 0.5f );
									mFace.mMesh2d->appendVertex( v );
								}

								FT_TRIANGLE* triangles;
								UINT triangleCount;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( UINT i = 0; i < triangleCount; ++i ) {
										mFace.mMesh2d->appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
									}
								}
							}
							delete [] pts;
						}
					}

					tagRECT rect;
					hr = mResult->GetFaceRect( &rect );
					if ( SUCCEEDED( hr ) ) {
						mFace.mBounds = Rectf( (float)rect.left, (float)rect.top, (float)rect.right, (float)rect.bottom );
					}
				}
			} else {
				mResult->Reset();
			}
		}
		mNewFace = true;
	}
}

float FaceTracker::calcFocalLength( const Vec2i& sz, float xFov, float yFov )
{
	float x		= (float)sz.x;
	float y		= (float)sz.y;
	float tx	= math<float>::tan( toRadians( xFov ) * 0.5f ) * 2.0f;
	float ty	= math<float>::tan( toRadians( yFov ) * 0.5f ) * 2.0f;
	float fx	= x / tx;
	float fy	= y / ty;
	float f		= ( fx + fy ) * 0.5f;
	return f;
}

//////////////////////////////////////////////////////////////////////////////////////////////

const char* FaceTracker::Exception::what() const throw() 
{ 
	return mMessage; 
}

FaceTracker::ExcFaceTrackerCreate::ExcFaceTrackerCreate() throw()
{
	sprintf( mMessage, "Unable to create face tracker." );
}

FaceTracker::ExcFaceTrackerCreateImage::ExcFaceTrackerCreateImage( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker image. Error: %i. %s", hr, FaceTracker::getErrorString( hr ).c_str() );
}

FaceTracker::ExcFaceTrackerCreateResult::ExcFaceTrackerCreateResult( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker result. Error: %i. %s", hr, FaceTracker::getErrorString( hr ).c_str() );
}

FaceTracker::ExcFaceTrackerInit::ExcFaceTrackerInit( long hr ) throw()
{
	sprintf( mMessage, "Unable to initialize face tracker. Error: %i. %s", hr, FaceTracker::getErrorString( hr ).c_str() );
}

FaceTracker::ExcFaceTrackerNuiInit::ExcFaceTrackerNuiInit( long hr ) throw()
{
	sprintf( mMessage, "Unable to initialize NUI. Error: %i.", hr );
}
}
 