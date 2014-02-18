/***************************************************************
*                                                              *
*   Copyright(c) Microsoft Corp.All rights reserved.           *
*                                                              *
****************************************************************/
#pragma once


#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN     // Exclude rarely-used stuff from Windows headers
#endif

// Windows Header Files:
#include <windows.h>
#include <objbase.h>

#include <Kinect.h>             // will need $(KINECTSDK20_DIR)inc; in the includes directory for project

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

    // get the native IFrameDescription
    // TODO: wrap data in a struct
    KINECT_CB HRESULT APIENTRY KCBGetBodyIndexFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetColorFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetDepthFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetInfraredFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);
    KINECT_CB HRESULT APIENTRY KCBGetLongExposureInfraredFrameDescription(_In_ KCBHANDLE kcbHandle, _COM_Outptr_ IFrameDescription** ppFrameDescription);

    // get the native IxxxFrame
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
}
