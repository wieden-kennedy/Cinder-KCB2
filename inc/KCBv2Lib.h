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

#ifndef __KCB_HANDLE__
#define __KCB_HANDLE__

typedef int KCBHANDLE;
static const int KCB_INVALID_HANDLE = 0xffffffff;

#endif

#ifndef __KCB_FRAME_TYPES__
#define __KCB_FRAME_TYPES__
typedef enum _KCBFrameTypes
{
    KCBFrameTypes_Body = 0,
    KCBFrameTypes_BodyIndex,
    KCBFrameTypes_Color,
    KCBFrameTypes_Depth,
    KCBFrameTypes_Infrared,
    KCBFrameTypes_LongExposureInfrared,
    KCBFrameTypes_Count,
    KCBFrameTypes_Unknown
} KCBFrameTypes;
#endif

struct KCBBodyFrame
{
    UINT count;
    IBody **ppBodies;
    LONGLONG* llTimeStamp;

    KCBBodyFrame()
        : count(0), ppBodies(0), llTimeStamp(nullptr)
    {
    }
};

struct KCBBodyIndexFrame
{
    ULONG cbBufferSize;
    BYTE* pbFrameBuffer;
    LONGLONG* llTimeStamp;

    KCBBodyIndexFrame()
        : cbBufferSize(0), pbFrameBuffer(nullptr), llTimeStamp(nullptr)
    {
    }
};

struct KCBColorFrame
{
    ColorImageFormat eColorFormat; 
    ULONG cbBufferSize; 
    BYTE* pbFrameBuffer;
    LONGLONG* llTimeStamp;

    KCBColorFrame()
        : cbBufferSize(0), pbFrameBuffer(nullptr), llTimeStamp(nullptr)
    {
    }
};

struct KCBDepthFrame
{
    ULONG cuiBufferSize;
    UINT16* puiFrameBuffer;
    LONGLONG* llTimeStamp;

    KCBDepthFrame()
        : cuiBufferSize(0), puiFrameBuffer(nullptr), llTimeStamp(nullptr)
    {
    }
};

struct KCBInfraredFrame
{
    ULONG cuiBufferSize;
    UINT16* puiFrameBuffer;
    LONGLONG* llTimeStamp;

    KCBInfraredFrame()
        : cuiBufferSize(0), puiFrameBuffer(nullptr), llTimeStamp(nullptr)
    {
    }
};

struct KCBLongExposureInfraredFrame
{
    ULONG cuiBufferSize;
    UINT16* puiFrameBuffer;
    LONGLONG* llTimeStamp;
    
    KCBLongExposureInfraredFrame()
        : cuiBufferSize(0), puiFrameBuffer(nullptr), llTimeStamp(nullptr)
    {
    }
};

extern "C"
{
    KINECT_CB KCBHANDLE APIENTRY KCBOpenDefaultSensor();
    KINECT_CB HRESULT APIENTRY KCBCloseSensor(_Inout_ KCBHANDLE* kcbHandle);
   // KINECT_CB HRESULT APIENTRY KCBSensorStatus(_In_ KCBHANDLE kcbHandle, _Inout_ KinectStatus* pStatus);

    // get the native IFrameDescription, caller must release the object
    KINECT_CB HRESULT APIENTRY KCBGetBodyIndexFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetColorFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetDepthFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetInfraredFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetLongExposureInfraredFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IFrameDescription** ppFrameDescription);

    // copy methods just to get the buffer data
    KINECT_CB HRESULT APIENTRY KCBGetBodies(_In_ KCBHANDLE kcbHandle, UINT capacity, _Inout_updates_all_(capacity) IBody **bodies, _Out_opt_ LONGLONG* llTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetBodyIndexFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cbBufferSize, _Inout_cap_(cbBufferSize) BYTE* pbBuffer, _Out_opt_ LONGLONG* llTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetColorFrameAsBGRA(_In_ KCBHANDLE kcbHandle, ULONG cbBufferSize, _Inout_cap_(cbBufferSize) BYTE* pbBuffer, _Out_opt_ LONGLONG* llTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetDepthFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cuiBufferSize, _Inout_cap_(cuiBufferSize) UINT16* puiBuffer, _Out_opt_ LONGLONG* llTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetInfraredFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cuiBufferSize, _Inout_cap_(cuiBufferSize) UINT16* puiBuffer, _Out_opt_ LONGLONG* llTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetLongExposureInfraredFrameBuffer(_In_ KCBHANDLE kcbHandle, ULONG cuiBufferSize, _Inout_cap_(cuiBufferSize) UINT16* puiBuffer, _Out_opt_ LONGLONG* llTimeStamp);
    KINECT_CB HRESULT APIENTRY KCBGetMultipleFrameBuffers(_In_ KCBHANDLE kcbHandle, 
        _Inout_opt_ KCBBodyFrame* pstBodyFrame,
        _Inout_opt_ KCBBodyIndexFrame* pstBodyIndexFrame,
        _Inout_opt_ KCBColorFrame* pstColorFrame,
        _Inout_opt_ KCBDepthFrame* pstDepthFrame,
        _Inout_opt_ KCBInfraredFrame* pstInfraredFrame,
        _Inout_opt_ KCBLongExposureInfraredFrame* pstLongExposureInfraredFrame);

    KINECT_CB bool APIENTRY KCBIsFrameReady(_In_ KCBHANDLE kcbHandle, KCBFrameTypes eFrameType);
    KINECT_CB bool APIENTRY KCBAnyFrameReady(_In_ KCBHANDLE kcbHandle);
    KINECT_CB bool APIENTRY KCBAllFramesReady(_In_ KCBHANDLE kcbHandle);
    KINECT_CB bool APIENTRY KCBMultiFrameReady(_In_ KCBHANDLE kcbHandle);

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

    // Coordinate mapper, caller must release the object
    KINECT_CB HRESULT APIENTRY KCBGetCoordinateMapper(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ ICoordinateMapper* ppCoordinateMapper);

    // get the native IxxxFrame, caller must release the object
    // if you just want the data, use the other frame functions.
    KINECT_CB HRESULT APIENTRY KCBGetBodyFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IBodyFrame **ppBodyFrame);
    KINECT_CB HRESULT APIENTRY KCBGetBodyIndexFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IBodyIndexFrame **ppBodyIndexFrame);
    KINECT_CB HRESULT APIENTRY KCBGetColorFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IColorFrame **ppColorFrame);
    KINECT_CB HRESULT APIENTRY KCBGetDepthFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IDepthFrame **ppDepthFrame);
    KINECT_CB HRESULT APIENTRY KCBGetInfraredFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ IInfraredFrame **ppInfraredFrame);
    KINECT_CB HRESULT APIENTRY KCBGetLongExposureInfraredFrame(_In_ KCBHANDLE kcbHandle, _COM_Outptr_result_maybenull_ ILongExposureInfraredFrame **ppLongExposureInfraredFrame);
    KINECT_CB HRESULT APIENTRY KCBGetMultiSourceFrame(_In_ KCBHANDLE kcbHandle, DWORD dwFrameSourceTypes, _COM_Outptr_result_maybenull_ IMultiSourceFrame **ppMultiSourceFrame);

}
