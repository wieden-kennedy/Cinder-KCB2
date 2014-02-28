#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN     // Exclude rarely-used stuff from Windows headers
#endif

// Windows Header Files:
#include <windows.h>
#include <objbase.h>

#include <Kinect.h>                // will need $(KINECTSDK20_DIR)inc; in the includes directory for project

#ifdef WIN32
    #ifdef DLL_EXPORTS
        #define KINECT_CB __declspec(dllexport)
    #else
        #define KINECT_CB __declspec(dllimport)
        #pragma comment (lib, "KCBV2.lib") // add $(KINECTSDK20_DIR)lib\x64 to the library directory for project
    #endif // DLL_EXPORTS
#endif //_WIN32

#ifndef __KCBHANDLE__
#define __KCBHANDLE__
typedef int KCBHANDLE;
#endif

#ifndef KCB_INVALID_HANDLE
#define KCB_INVALID_HANDLE    0xffffffff
#endif

extern "C"
{
    KINECT_CB KCBHANDLE APIENTRY KCBOpenDefaultSensor();
    KINECT_CB HRESULT APIENTRY KCBCloseSensor(_Inout_ KCBHANDLE* kcbHandle);
    KINECT_CB HRESULT APIENTRY KCBSensorStatus(_In_ KCBHANDLE kcbHandle, _Inout_ KinectStatus* pStatus);

    // get the native IFrameDescription, caller must release the object
    KINECT_CB HRESULT APIENTRY KCBGetBodyIndexFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetColorFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetDepthFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetInfraredFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetLongExposureInfraredFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);

    // get the native IxxxFrame, caller must release the object
    KINECT_CB HRESULT APIENTRY KCBGetBodyFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IBodyFrame **ppBodyFrame);
    KINECT_CB HRESULT APIENTRY KCBGetBodyIndexFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IBodyIndexFrame **ppBodyIndexFrame);
    KINECT_CB HRESULT APIENTRY KCBGetColorFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IColorFrame **ppColorFrame);
    KINECT_CB HRESULT APIENTRY KCBGetDepthFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IDepthFrame **ppDepthFrame);
    KINECT_CB HRESULT APIENTRY KCBGetInfraredFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IInfraredFrame **ppColorFrame);
    KINECT_CB HRESULT APIENTRY KCBGetLongExposureInfraredFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ ILongExposureInfraredFrame **ppColorFrame);

    // copy methods just to get the buffer data
    KINECT_CB HRESULT APIENTRY KCBGetBodies(_In_ KCBHANDLE kcbHandle, UINT capacity, _Inout_updates_all_(capacity) IBody **bodies, _Out_opt_ LONGLONG* liTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetBodyIndexFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cbBufferSize, _Inout_cap_(cbBufferSize) BYTE* pbBuffer, _Out_opt_ LONGLONG* liTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetColorFrameAsBGRA(_In_ KCBHANDLE kcbHandle, ULONG cbBufferSize, _Inout_cap_(cbBufferSize) BYTE* pbBuffer, _Out_opt_ LONGLONG* liTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetDepthFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cbBufferSize, _Inout_cap_(cbBufferSize) UINT16* pbBuffer, _Out_opt_ LONGLONG* liTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetInfraredFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cbBufferSize, _Inout_cap_(cbBufferSize) UINT16* pbBuffer, _Out_opt_ LONGLONG* liTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetLongExposureInfraredFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cbBufferSize, _Inout_cap_(cbBufferSize) UINT16* pbBuffer, _Out_opt_ LONGLONG* liTimeStamp);

    // Coordinate mapper, caller must release the object
    KINECT_CB HRESULT APIENTRY KCBGetCoordinateMapper(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ ICoordinateMapper* ppCoordinateMapper);

    // straight calls to the coordinate mapper
    KINECT_CB HRESULT APIENTRY KCBMapCameraPointToDepthSpace(_In_ KCBHANDLE kcbHandle, 
        CameraSpacePoint cameraPoint, 
        _Out_ DepthSpacePoint* depthPoint);
    KINECT_CB HRESULT APIENTRY KCBMapCameraPointToColorSpace(_In_ KCBHANDLE kcbHandle, 
        CameraSpacePoint cameraPoint, 
        _Out_ ColorSpacePoint *colorPoint);
    KINECT_CB HRESULT APIENTRY KCBMapDepthPointToCameraSpace(_In_ KCBHANDLE kcbHandle, 
        DepthSpacePoint depthPoint, UINT16 depth, 
        _Out_ CameraSpacePoint *cameraPoint);
    KINECT_CB HRESULT APIENTRY KCBMapDepthPointToColorSpace(_In_ KCBHANDLE kcbHandle, 
        DepthSpacePoint depthPoint, UINT16 depth, 
        _Out_ ColorSpacePoint *colorPoint);
    KINECT_CB HRESULT APIENTRY KCBMapCameraPointsToDepthSpace(_In_ KCBHANDLE kcbHandle, 
        UINT cameraPointCount, _In_reads_(cameraPointCount) const CameraSpacePoint *cameraPoints, 
        UINT depthPointCount, 
        _Out_writes_all_(depthPointCount) DepthSpacePoint *depthPoints);
    KINECT_CB HRESULT APIENTRY KCBMapCameraPointsToColorSpace(_In_ KCBHANDLE kcbHandle,
        UINT cameraPointCount, _In_reads_(cameraPointCount) const CameraSpacePoint *cameraPoints,
        UINT colorPointCount, 
        _Out_writes_all_(colorPointCount) ColorSpacePoint *colorPoints);
    KINECT_CB HRESULT APIENTRY KCBMapDepthPointsToCameraSpace(_In_ KCBHANDLE kcbHandle, 
        UINT depthPointCount, _In_reads_(depthPointCount) const DepthSpacePoint *depthPoints,
        UINT depthCount, _In_reads_(depthCount) const UINT16 *depths,
        UINT cameraPointCount, 
        _Out_writes_all_(cameraPointCount) CameraSpacePoint *cameraPoints);
    KINECT_CB HRESULT APIENTRY KCBMapDepthPointsToColorSpace(_In_ KCBHANDLE kcbHandle,
        UINT depthPointCount, _In_reads_(depthPointCount) const DepthSpacePoint *depthPoints,
        UINT depthCount, _In_reads_(depthCount) const UINT16 *depths,
        UINT colorPointCount, 
        _Out_writes_all_(colorPointCount) ColorSpacePoint *colorPoints);
    KINECT_CB HRESULT APIENTRY KCBMapDepthFrameToCameraSpace(_In_ KCBHANDLE kcbHandle, 
        UINT depthPointCount, _In_reads_(depthPointCount) const UINT16 *depthFrameData, 
        UINT cameraPointCount, 
        _Out_writes_all_(cameraPointCount) CameraSpacePoint *cameraSpacePoints);
    KINECT_CB HRESULT APIENTRY KCBMapDepthFrameToColorSpace(_In_ KCBHANDLE kcbHandle,
        UINT depthPointCount, _In_reads_(depthPointCount) const UINT16 *depthFrameData,
        UINT colorPointCount, 
        _Out_writes_all_(colorPointCount) ColorSpacePoint *colorSpacePoints);
    KINECT_CB HRESULT APIENTRY KCBMapColorFrameToDepthSpace(_In_ KCBHANDLE kcbHandle,
        UINT depthDataPointCount, _In_reads_(depthDataPointCount) const UINT16 *depthFrameData,
        UINT depthPointCount, 
        _Out_writes_all_(depthPointCount) DepthSpacePoint *depthSpacePoints);
    KINECT_CB HRESULT APIENTRY KCBMapColorFrameToCameraSpace(_In_ KCBHANDLE kcbHandle,
        UINT depthDataPointCount, _In_reads_(depthDataPointCount)  const UINT16 *depthFrameData,
        UINT cameraPointCount, 
        _Out_writes_all_(cameraPointCount) CameraSpacePoint *cameraSpacePoints);
    KINECT_CB HRESULT APIENTRY GetDepthFrameToCameraSpaceTable(_In_ KCBHANDLE kcbHandle,
        _Out_  UINT32 *tableEntryCount,
        _Outptr_result_bytebuffer_(*tableEntryCount) PointF **tableEntries);

}
