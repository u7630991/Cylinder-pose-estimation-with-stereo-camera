#ifndef SBASLERCAMERACONTROL_H
#define SBASLERCAMERACONTROL_H

#include <pylon/PylonIncludes.h>
#include <iostream>
#include <string>

#include "ImageHandler.h"

using namespace std;
using namespace Pylon;
using namespace GenApi;

class SBaslerCameraControl
{

public:

    enum SBaslerCameraControl_Type{
        Type_Basler_Freerun,
        Type_Basler_Line1,
        Type_Basler_ExposureTimeAbs,
        Type_Basler_GainRaw,
        Type_Basler_AcquisitionFrameRateAbs,
        Type_Basler_LineDebouncerTime,
        Type_Basler_Width,
        Type_Basler_Height,
        Type_Basler_LineSource,
        Type_Basler_Software,

    };

    CSampleImageEventHandler *Handler1;
    CSampleImageEventHandler *Handler2;
    CInstantCameraArray m_basler[2];

    string CamSerial0, CamSerial1;

    void initSome();
    void deleteAll();

    int OpenCamera(string cameraSN);
    int CloseCamera();

    void setFeatureTriggerSourceType(string type,size_t number);
    void setFeatureTriggerModeType(bool on,size_t number);
    bool getFeatureTriggerModeType();
    void SetCamera(SBaslerCameraControl::SBaslerCameraControl_Type index, double tmpValue = 0.0,size_t number=1); // 设置各种参数
    double GetCamera(SBaslerCameraControl::SBaslerCameraControl_Type index);

    long StartAcquire();
    long StopAcquire();
    void SwitchCameraMode_Freerun();//LUO NAN
    void SwitchCameraMode_Line1();//LUO NAN
    void SwitchCameraMode_Software();

private:
//     //CInstantCamera m_basler;
    vector<string> m_cameralist;
    string m_currentMode;
    bool m_isOpenAcquire = false;
    bool m_isOpen = false;
};
#endif // SBASLERCAMERACONTROL_H
