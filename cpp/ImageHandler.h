#include <pylon/PylonIncludes.h>

using namespace Pylon;
using namespace GenApi;
class CSampleImageEventHandler : public CImageEventHandler
{
public:
	CPylonImage targetImage;
	CImageFormatConverter converter;
	virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult);
    unsigned char targetImage_projector[26][2448*2048];
    int triggercount=0;
    int trigger=0;
};
