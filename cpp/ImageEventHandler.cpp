#include "ImageHandler.h"
#include <pylon/PylonIncludes.h>

void CSampleImageEventHandler::OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
{
    if (ptrGrabResult->GrabSucceeded())
	{
        converter.OutputPixelFormat = PixelType_Mono8;
		converter.Convert(targetImage, ptrGrabResult);

        if(trigger==1 && triggercount<26)
        {
            memcpy(targetImage_projector[triggercount], targetImage.GetBuffer(), 2448 * 2048 * sizeof(unsigned char));
            triggercount++;
        }
	}
}
