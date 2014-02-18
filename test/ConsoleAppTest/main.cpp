// Win32Project1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "KCBv2Lib.h"

#ifndef __BRGA
#define __BRGA 
#endif

int _tmain(int argc, _TCHAR* argv[])
{
    KCBHANDLE kcbHandle = KCB_INVALID_HANDLE;
    kcbHandle = KCBOpenDefaultSensor();
    if (KCB_INVALID_HANDLE == kcbHandle)
    {
        return 1;
    }

    IFrameDescription* pFrameDesc;
    HRESULT hr = KCBGetColorFrameDescription(kcbHandle, &pFrameDesc);

    UINT length = 0;
    hr = pFrameDesc->get_LengthInPixels(&length);

    UINT bbp = 0;
#ifdef __BRGA
    bbp = sizeof(DWORD);
#else
    hr = pFrameDesc->get_BytesPerPixel(&bbp);
#endif

    ULONG bufferSize = bbp * length;
    BYTE* pbBuffer = new BYTE[bufferSize];

    BOOL fContinue = true;
    UINT uiCount = 0;
    LONGLONG timestamp = 0;
    while(fContinue)
    {
        hr = KCBGetColorFrameAsBGRA(kcbHandle, bufferSize, pbBuffer, &timestamp);
        if(SUCCEEDED(hr))
        {
            //ProcessColorFrame(bufferSize, pbBuffer);
            if(120 < uiCount++)
            {
                fContinue = false;
            }
        }
    }

    // clean-up
    delete [] pbBuffer;

    hr = KCBCloseSensor(&kcbHandle);

    return 0;
}

