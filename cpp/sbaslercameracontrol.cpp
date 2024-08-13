
#include "sbaslercameracontrol.h"


#define Real_Freerun
int width  = 2448;
int height = 2048;
int exposureTimeAbs = 5000; //microseconds;
int lineDebouncerTime = 50;//microseconds;

void SBaslerCameraControl::initSome()
{
    Pylon::PylonInitialize();
    CTlFactory &tlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    if (tlFactory.EnumerateDevices(devices) == 0)
    {
        cerr << "No camera !!!: " << endl;
    }
    m_basler[0].Initialize((min(devices.size(), (size_t)2)));
    m_basler[1].Initialize((min(devices.size(), (size_t)2)));

    for (size_t i = 0; i < m_basler[0].GetSize(); i = i + 1)
    {
        m_basler[0][i].Attach(tlFactory.CreateDevice(devices[i]));
        String_t sn = m_basler[0][i].GetDeviceInfo().GetSerialNumber();

        if (sn.compare(CamSerial0.c_str())==0)//origin
        {
            m_basler[1][0].Attach(tlFactory.CreateDevice(devices[i]), Pylon::Cleanup_Delete);
        }

        if (sn.compare(CamSerial1.c_str())==0)
        {
            m_basler[1][1].Attach(tlFactory.CreateDevice(devices[i]), Pylon::Cleanup_Delete);
        }
    }

    Handler1 = new CSampleImageEventHandler();
    Handler2 = new CSampleImageEventHandler();
    m_basler[1][0].RegisterImageEventHandler(Handler1, Pylon::RegistrationMode_ReplaceAll, Pylon::Ownership_ExternalOwnership);
    m_basler[1][1].RegisterImageEventHandler(Handler2, Pylon::RegistrationMode_ReplaceAll, Pylon::Ownership_ExternalOwnership);
    m_basler[1][0].Open();
    m_basler[1][1].Open();
    SetCamera(Type_Basler_ExposureTimeAbs, exposureTimeAbs,0);
    SetCamera(Type_Basler_ExposureTimeAbs, exposureTimeAbs,1);
    SetCamera(Type_Basler_LineDebouncerTime, lineDebouncerTime, 0);
    SetCamera(Type_Basler_LineDebouncerTime, lineDebouncerTime, 1);

    setFeatureTriggerSourceType("Software",0);
    setFeatureTriggerSourceType("Software",1);

    setFeatureTriggerModeType(0,0);
    setFeatureTriggerModeType(0,1);


    //------------------------------------------------
    INodeMap &cameraNodeMap0 = m_basler[1][0].GetNodeMap();
    //CBooleanPtr(cameraNodeMap0.GetNode("GammaEnable"))->SetValue(true);
    //CEnumerationPtr(cameraNodeMap0.GetNode("GammaSelector"))->FromString("user");
    CFloatPtr(cameraNodeMap0.GetNode("Gamma"))->SetValue(0.5);

    INodeMap &cameraNodeMap1 = m_basler[1][1].GetNodeMap();
    //CBooleanPtr(cameraNodeMap1.GetNode("GammaEnable"))->SetValue(true);
    //CEnumerationPtr(cameraNodeMap1.GetNode("GammaSelector"))->FromString("user");
    CFloatPtr(cameraNodeMap1.GetNode("Gamma"))->SetValue(0.5);

 /**/
}
void SBaslerCameraControl::deleteAll()
{

    if(m_isOpenAcquire)
    {
        StopAcquire();
    }
    CloseCamera();
    cout << "SBaslerCameraControl deleteAll: PylonTerminate" << endl;
    PylonTerminate();
    cout << "SBaslerCameraControl deleteAll: Close" << endl;
}

int SBaslerCameraControl::OpenCamera(string cameraSN)
{
    return 0;
}
int SBaslerCameraControl::CloseCamera()
{
    if(!m_isOpen)
    {
        return -1;
    }
    try {
            if(m_basler[1][0].IsOpen())
            {
                m_basler[1][0].DetachDevice();
                m_basler[1][0].Close();
            }
    } catch (GenICam::GenericException &e)
    {
         // OutputDebugString(LPCWSTR(e.GetDescription()));
          cerr << "CloseCamera Error:" << e.GetDescription() << endl;
          return -2;
    }
    return 0;
}
void SBaslerCameraControl::setFeatureTriggerSourceType(string type,size_t number)
{

    if(type == "Freerun")
    {
        m_currentMode = "Freerun";
        SetCamera(Type_Basler_Freerun,number);

    } else if(type == "Line1")
    {
        m_currentMode = "Line1";
        SetCamera(Type_Basler_Line1,number);

    }else if(type == "Software")
    {
        m_currentMode = "Software";
        SetCamera(Type_Basler_Software,number);
    }
}
void SBaslerCameraControl::setFeatureTriggerModeType(bool on,size_t number)
{
    INodeMap &cameraNodeMap = m_basler[1][number].GetNodeMap();
    CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
    ptrTriggerSel->FromString("FrameStart");
    CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
    ptrTrigger->SetIntValue(on?1:0);
}
bool SBaslerCameraControl::getFeatureTriggerModeType()
{
    INodeMap &cameraNodeMap = m_basler[1][0].GetNodeMap();
    CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
    ptrTriggerSel->FromString("FrameStart");
    CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
    return ptrTrigger->GetIntValue() == 1;
}
void SBaslerCameraControl::SetCamera(SBaslerCameraControl::SBaslerCameraControl_Type index, double tmpValue,size_t number)
{
    INodeMap &cameraNodeMap = m_basler[1][number].GetNodeMap();
    switch (index)
    {
        case Type_Basler_Freerun:
        {
            CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
            ptrTriggerSel->FromString("FrameStart");
            CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
#ifdef Real_Freerun
         ptrTrigger->SetIntValue(0);
#else //Software
            ptrTrigger->SetIntValue(1);
            CEnumerationPtr  ptrTriggerSource = cameraNodeMap.GetNode ("TriggerSource");
            ptrTriggerSource->FromString("Software");
#endif
        } break;
        case Type_Basler_Line1:
        {
            CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
            ptrTriggerSel->FromString("FrameStart");
            CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
            ptrTrigger->SetIntValue(1);
            CEnumerationPtr  ptrTriggerSource = cameraNodeMap.GetNode ("TriggerSource");
            ptrTriggerSource->FromString("Line1");
        } break;
        case Type_Basler_ExposureTimeAbs:
        {
            const CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTime");
            exposureTime->SetValue(tmpValue);
        } break;
        case Type_Basler_GainRaw:
        {
            const CIntegerPtr cameraGen = cameraNodeMap.GetNode("GainRaw");
            cameraGen->SetValue(tmpValue);
        } break;
        case Type_Basler_AcquisitionFrameRateAbs:
        {
            const CBooleanPtr frameRate = cameraNodeMap.GetNode("AcquisitionFrameRateEnable");
            frameRate->SetValue(true);
            const CFloatPtr frameRateABS = cameraNodeMap.GetNode("AcquisitionFrameRateAbs");
            frameRateABS->SetValue(tmpValue);
        } break;
        case Type_Basler_LineDebouncerTime:
        {
            const CFloatPtr debouncerTime = cameraNodeMap.GetNode("LineDebouncerTime");
            debouncerTime->SetValue(tmpValue);
        } break;
        case Type_Basler_Width:
        {
            const CIntegerPtr widthPic = cameraNodeMap.GetNode("Width");
            widthPic->SetValue(tmpValue);
        } break;
        case Type_Basler_Height:
        {
            const CIntegerPtr heightPic = cameraNodeMap.GetNode("Height");
            heightPic->SetValue(tmpValue);
        } break;
        case Type_Basler_LineSource:
        {
             CEnumerationPtr  ptrLineSource = cameraNodeMap.GetNode ("LineSource");
             ptrLineSource->SetIntValue(2);
        } break;

        default:
        break;
    }
}
double SBaslerCameraControl::GetCamera(SBaslerCameraControl::SBaslerCameraControl_Type index)
{
    INodeMap &cameraNodeMap = m_basler[1][1].GetNodeMap();
    switch (index)
    {
        case Type_Basler_ExposureTimeAbs:
        {
            const CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTime");
            return exposureTime->GetValue();
        } break;
        case Type_Basler_GainRaw:
        {
            const CIntegerPtr cameraGen = cameraNodeMap.GetNode("GainRaw");
            return cameraGen->GetValue();
        } break;
         case Type_Basler_AcquisitionFrameRateAbs:
        {
             const CBooleanPtr frameRate = cameraNodeMap.GetNode("AcquisitionFrameRateEnable");
             frameRate->SetValue(true);
             const CFloatPtr frameRateABS = cameraNodeMap.GetNode("AcquisitionFrameRateAbs");
             return frameRateABS->GetValue();
        } break;
        case Type_Basler_Width:
        {
            const CIntegerPtr widthPic = cameraNodeMap.GetNode("Width");
            return widthPic->GetValue();
        } break;
        case Type_Basler_Height:
        {
            const CIntegerPtr heightPic = cameraNodeMap.GetNode("Height");
            return heightPic->GetValue();
        } break;
        default:
            return -1;
            break;
    }
}
long SBaslerCameraControl::StartAcquire()
{
    cout << "SBaslerCameraControl StartAcquire: " << m_currentMode << endl;
    if(m_currentMode == "Freerun")
    {
        m_basler[1][0].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        m_basler[1][1].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        m_isOpenAcquire = true;//luo
        //cout << "Freerun" << endl;

    } else if(m_currentMode == "Software")
    {
        m_basler[1][0].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        m_basler[1][1].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        //cout << "Software" << endl;
    } else if(m_currentMode == "Line1")
    {
        //m_basler[1][0].StartGrabbing(GrabStrategy_OneByOne);
        //m_basler[1][1].StartGrabbing(GrabStrategy_OneByOne);
        m_basler[1][0].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        m_basler[1][1].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        m_isOpenAcquire = false;
        //cout << "Line1" << endl;
    } else if(m_currentMode == "Line2")
    {
        //m_basler[1][0].StartGrabbing(GrabStrategy_OneByOne);
        //m_basler[1][1].StartGrabbing(GrabStrategy_OneByOne);
        m_basler[1][0].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        m_basler[1][1].StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
        //cout << "Line2" << endl;
    }
    return 0;
}
long SBaslerCameraControl::StopAcquire()
{
    m_isOpenAcquire = false;
    cout << "SBaslerCameraControl StopAcquire" << endl;
    try
    {
        if (m_basler[1][0].IsGrabbing() && m_basler[1][1].IsGrabbing())
        {
            m_basler[1][0].StopGrabbing();
            m_basler[1][1].StopGrabbing();
        }

    } catch (GenICam::GenericException &e)
    {
        cerr << "StopAcquire Error:" << e.GetDescription() << endl;
    }
    return 0;
}

void SBaslerCameraControl::SwitchCameraMode_Freerun()
{
     setFeatureTriggerSourceType("Freerun",0);
     setFeatureTriggerSourceType("Freerun",1);
     setFeatureTriggerModeType(0,0);
     setFeatureTriggerModeType(0,1);
}

void SBaslerCameraControl::SwitchCameraMode_Software()
{
     setFeatureTriggerSourceType("Software",0);
     setFeatureTriggerSourceType("Software",1);
     setFeatureTriggerModeType(0,0);
     setFeatureTriggerModeType(0,1);
}

void SBaslerCameraControl::SwitchCameraMode_Line1()
{
     setFeatureTriggerSourceType("Line1",0);
     setFeatureTriggerSourceType("Line1",1);
     setFeatureTriggerModeType(1,0);
     setFeatureTriggerModeType(1,1);
}

