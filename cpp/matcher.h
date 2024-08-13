
#include <iostream>
#include <sstream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <vector>

using namespace std;
using namespace chrono;

#if ENABLE_COMPUTATION

#include <sys/stat.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

int ImageWidth;
int ImageHeight;

#define NUM_PICS 13
#define GP_Num 1

extern void Call_d_atan2(
	unsigned char *pPixel_tywhite, 
	unsigned char *pPixel_tyblack, 
	unsigned char *pPixel1, unsigned char *pPixel2, 
	unsigned char *pPixel3, unsigned char *pPixel4, 
	unsigned char *pPixel_ty1, unsigned char *pPixel_ty2, 
	unsigned char *pPixel_ty3, unsigned char *pPixel_ty4, 
	unsigned char *pPixel_ty5, unsigned char *pPixel_ty6, 
	unsigned char *pPixel_ty7, double *pOutputData, const int image_size);

extern void Call_StereoMatching_GAll(
	Mat Mask1_zero[GP_Num], Mat Mask2_zero[GP_Num],
    Mat w1_1[GP_Num], Mat w2_1[GP_Num],
    Mat correspondent_lines[GP_Num], Mat correspondent_lines2[GP_Num],
    Mat M[GP_Num], Mat M_2[GP_Num], Mat M2[GP_Num], Mat M2_Mir[GP_Num], Mat M1[GP_Num], Mat M1_2[GP_Num],
    double *m_distortion[GP_Num], double *m_distortion_2[GP_Num],
    double *pPhase1h[GP_Num], double *pPhase1v[GP_Num], double *pPhase2h[GP_Num], double *pPhase2v[GP_Num],
    double P, double XOriginCentral, double YOriginCentral,
    double MatchRange, double HeightRange, double AngleRange,
    double *OutPut3DData[GP_Num], double *OutPut3DData_norm[GP_Num], double *OutPut3DData_dis[GP_Num],
    int ImageHeight, int ImageWidth, char *CudaError);

class Pics
{
    public:
        
        Pics(int imgW, int imgH)
        {
            w = imgW;
            h = imgH;
            for (int i=0; i<NUM_PICS; i++)
                pixels[i] = new uchar[imgW*imgH];
        }

        void update(const vector<Mat> imgs)
        {
            // Write the images into the flattened array of pixels
            for (int i=0; i<NUM_PICS; i++)
            {
                for (int y=0; y <= h-1; y++)
                {
                    for (int x=0; x <= w-1; x++)
                        *(pixels[i] + y*w + x) = imgs[i].at<uchar>(y, x);
                }
            }

            // Take the bright part of the dark/light image diff as foreground
            foreground = imgs[0] - imgs[1];
            threshold(foreground, foreground, 48, 255, THRESH_BINARY);
        }

        uchar *p_tywhite() { return pixels[0]; }
        uchar *p_tyblack() { return pixels[1]; }
        uchar *p_1()       { return pixels[2]; }
        uchar *p_2()       { return pixels[3]; }
        uchar *p_3()       { return pixels[4]; }
        uchar *p_4()       { return pixels[5]; }
        uchar *p_ty1()     { return pixels[6]; }
        uchar *p_ty2()     { return pixels[7]; }
        uchar *p_ty3()     { return pixels[8]; }
        uchar *p_ty4()     { return pixels[9]; }
        uchar *p_ty5()     { return pixels[10];}
        uchar *p_ty6()     { return pixels[11];}
        uchar *p_ty7()     { return pixels[12];}
        const Mat getForeground() { return foreground; }

    private:
        int w, h;
        unsigned char *pixels[NUM_PICS];
        Mat foreground;
};

#endif