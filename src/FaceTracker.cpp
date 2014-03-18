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

FaceTracker::Face::Face()
{
	mPoseMatrix.setToNull();
	mUserId = 0;
}

const FaceTracker::AnimationUnitMap& FaceTracker::Face::getAnimationUnits() const
{
	return mAnimationUnits;
}

const Rectf& FaceTracker::Face::getBounds() const
{
	return mBounds;
}

const TriMesh& FaceTracker::Face::getMesh() const
{
	return mMesh;
}

const TriMesh2d& FaceTracker::Face::getMesh2d() const
{
	return mMesh2d;
}

const Matrix44f& FaceTracker::Face::getPoseMatrix() const
{
	return mPoseMatrix;
}

size_t FaceTracker::Face::getUserId() const
{
	return mUserId;
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

void FaceTracker::start()
{
	stop();

	mConfigColor.Height			= 480;
	mConfigColor.Width			= 640;
	mConfigColor.FocalLength	= 531.15f;

	mConfigDepth.Height			= 240;
	mConfigDepth.Width			= 320;
	mConfigDepth.FocalLength	= 285.63f;

	long hr			= S_OK;
	mFaceTracker	= FTCreateFaceTracker();
    if ( !mFaceTracker ) {
		throw ExcFaceTrackerCreate();
	}

	hr = NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH | 
						NUI_INITIALIZE_FLAG_USES_COLOR );
	if ( FAILED( hr ) ) {
		throw ExcFaceTrackerNuiInit( hr );
	}

	//hr = mFaceTracker->Initialize( &mConfigColor, &mConfigDepth, FTRegisterDepthToColor( FaceTracker::depthToColor ), 0 );
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

void FaceTracker::update( const Channel16u& infrared, const Vec3f headPoints[ 2 ], size_t userId )
{
	if ( mNewFace && mEventHandler != nullptr ) {
		mEventHandler( mFace );
		if ( infrared ) {
			mHeadPoints.clear();
			if ( headPoints != nullptr ) {
				mHeadPoints.push_back( headPoints[ 0 ] );
				mHeadPoints.push_back( headPoints[ 1 ] );
			}
			mChannelInfraredOriginal	= infrared;
			mUserId						= userId;	
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
			if ( mChannelInfraredOriginal ) {
				bool attachDepth = !mChannelInfrared;
				bool attachVideo = !mSurfaceInfrared;

				if ( !mChannelColor ) {
					mChannelColor = Channel32f( 640, 480 );
				}
				if ( !mChannelDepth ) {
					mChannelDepth = Channel32f( 320, 240 );
				}
				ip::resize( Channel32f( mChannelInfraredOriginal ), &mChannelColor );
				ip::resize( Channel32f( mChannelInfraredOriginal ), &mChannelDepth );

				mChannelInfrared = Channel16u( mChannelDepth );
				mSurfaceInfrared = Surface8u( mChannelColor );

				if ( attachDepth ) {
					mSensorData.pDepthFrame->Attach( mChannelInfrared.getWidth(), mChannelInfrared.getHeight(), 
						(void*)mChannelInfrared.getData(), FTIMAGEFORMAT_UINT16_D13P3, mChannelInfrared.getRowBytes() );
				}
				if ( attachVideo ) {
					mSensorData.pVideoFrame->Attach( mSurfaceInfrared.getWidth(), mSurfaceInfrared.getHeight(), 
						(void*)mSurfaceInfrared.getData(), FTIMAGEFORMAT_UINT8_R8G8B8, mSurfaceInfrared.getRowBytes() );
				}
			}

			long hr = S_OK;

			mFace.mAnimationUnits.clear();
			mFace.mBounds = Rectf( 0.0f, 0.0f, 0.0f, 0.0f );
			mFace.mMesh.clear();
			mFace.mMesh2d.clear();
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
									mFace.mMesh.appendVertex( v );
								}

								FT_TRIANGLE* triangles;
								UINT triangleCount;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( size_t i = 0; i < triangleCount; ++i ) {
										mFace.mMesh.appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
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
									mFace.mMesh2d.appendVertex( v );
								}

								FT_TRIANGLE* triangles;
								UINT triangleCount;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( UINT i = 0; i < triangleCount; ++i ) {
										mFace.mMesh2d.appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
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

HRESULT FTAPI FaceTracker::depthToColor( UINT depthFrameWidth, UINT depthFrameHeight, 
										 UINT colorFrameWidth, UINT colorFrameHeight, 
										 FLOAT zoomFactor, POINT viewOffset, 
										 LONG depthX, LONG depthY, USHORT depthZ, 
										 LONG* pColorX, LONG* pColorY )
{
    *pColorX = depthX * 2;
    *pColorY = depthY * 2;
    return S_OK;
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
 