#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>
#include "math_constants.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#include <string>
#include <thrust/version.h>
#include <fstream>
#include <vector>
#include <cuda_fp16.h>
// #include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
//==stereo
#include <stdio.h>
#include <cuda.h>
#include "cuda_runtime.h"
#include <math.h>

//#include <helper_math.h>
//#include <helper_functions.h>
//#include <helper_cuda.h>

#include "math_constants.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string>
//#include <windows.h>
#include <thrust/version.h>
#include <fstream>
#include <vector>
#include <cuda_fp16.h>
#include "cublas_v2.h"
//==stereo



using namespace cv;
extern int ImageWidth;
extern int ImageHeight;

//===stereo===
namespace GPU = cv::cuda;
using namespace std;
extern double PI;
extern double P;
#define GP_Num 1
//===stereo===


//=====================================================stereo=========================================================
void OutputString(const char *InputString, char *OutputChar);
void Call_StereoMatching_GAll(Mat Mask1_zero[GP_Num], Mat Mask2_zero[GP_Num],
    Mat w1_1[GP_Num], Mat w2_1[GP_Num],
    Mat correspondent_lines[GP_Num], Mat correspondent_lines2[GP_Num],
    Mat M[GP_Num], Mat M_2[GP_Num], Mat M2[GP_Num], Mat M2_Mir[GP_Num], Mat M1[GP_Num], Mat M1_2[GP_Num],
    double *m_distortion[GP_Num], double *m_distortion_2[GP_Num],
    double *pPhase1h[GP_Num], double *pPhase1v[GP_Num], double *pPhase2h[GP_Num], double *pPhase2v[GP_Num],
    double P, double XOriginCentral, double YOriginCentral,
    double MatchRange, double HeightRange, double AngleRange,
    double *OutPut3DData[GP_Num], double *OutPut3DData_norm[GP_Num], double *OutPut3DData_dis[GP_Num],
    int ImageHeight, int ImageWidth, char *CudaError);
void Call_d_atan2(unsigned char *pPixel_tywhite, unsigned char *pPixel_tyblack, unsigned char *pPixel1, unsigned char *pPixel2, unsigned char *pPixel3, unsigned char *pPixel4, unsigned char *pPixel_ty1, unsigned char *pPixel_ty2, unsigned char *pPixel_ty3, unsigned char *pPixel_ty4, unsigned char *pPixel_ty5, unsigned char *pPixel_ty6, unsigned char *pPixel_ty7, double *pOutputData, const int image_size);


__global__ void StereoMatching_GAll(unsigned char *d_Mask1_zero,double *d_M, double *d_M_2,double *d_correspondent_lines,double *d_pPhase1v,double *d_pPhase2v,double *d_OutPut3DData,double MatchRange,int ImageHeight, int ImageWidth);
__global__ void d_atan2(unsigned char *d_pPixel_tywhite, unsigned char *d_pPixel_tyblack, unsigned char *d_pPixel1, unsigned char *d_pPixel2, unsigned char *d_pPixel3, unsigned char *d_pPixel4, unsigned char * d_pPixel_ty1, unsigned char *d_pPixel_ty2, unsigned char *d_pPixel_ty3, unsigned char *d_pPixel_ty4, unsigned char *d_pPixel_ty5, unsigned char *d_pPixel_ty6, unsigned char *d_pPixel_ty7, double *d_pOutputData, const int image_size);
__device__ void Direct(int n, double a[3][3], double b[3]);
__global__ void d_atan2(unsigned char *d_pPixel_tywhite, unsigned char *d_pPixel_tyblack, unsigned char *d_pPixel1, unsigned char *d_pPixel2, unsigned char *d_pPixel3, unsigned char *d_pPixel4, unsigned char * d_pPixel_ty1, unsigned char *d_pPixel_ty2, unsigned char *d_pPixel_ty3, unsigned char *d_pPixel_ty4, unsigned char *d_pPixel_ty5, unsigned char *d_pPixel_ty6, unsigned char *d_pPixel_ty7, double *d_pOutputData, const int image_size)
{
    int ii = blockDim.x * blockIdx.x + threadIdx.x;
    double Thresold[1], ThresoldPic1[1], ThresoldPic2[1], ThresoldPic3[1], ThresoldPic4[1], ThresoldPic5[1], ThresoldPic6[1], ThresoldPic7[1], d_pPhotoAtan[1];
    int SumPhoto[1];
    const double PI = 3.1415926535898;

    if (ii < image_size)
    {
        d_pPhotoAtan[0] = atan2f(d_pPixel4[ii] - d_pPixel2[ii], d_pPixel1[ii] - d_pPixel3[ii]);
        Thresold[0] = (d_pPixel_tywhite[ii] + d_pPixel_tyblack[ii]) / 2;

        ThresoldPic1[0] = (d_pPixel_ty1[ii] > Thresold[0]) ? 1 : 0;
        ThresoldPic2[0] = (d_pPixel_ty2[ii] > Thresold[0]) ? 1 : 0;
        ThresoldPic3[0] = (d_pPixel_ty3[ii] > Thresold[0]) ? 1 : 0;
        ThresoldPic4[0] = (d_pPixel_ty4[ii] > Thresold[0]) ? 1 : 0;
        ThresoldPic5[0] = (d_pPixel_ty5[ii] > Thresold[0]) ? 1 : 0;
        ThresoldPic6[0] = (d_pPixel_ty6[ii] > Thresold[0]) ? 1 : 0;
        ThresoldPic7[0] = (d_pPixel_ty7[ii] > Thresold[0]) ? 1 : 0;

        ThresoldPic2[0] = (ThresoldPic2[0] == ThresoldPic1[0]) ? 0 : 1;
        ThresoldPic3[0] = (ThresoldPic3[0] == ThresoldPic2[0]) ? 0 : 1;
        ThresoldPic4[0] = (ThresoldPic4[0] == ThresoldPic3[0]) ? 0 : 1;
        ThresoldPic5[0] = (ThresoldPic5[0] == ThresoldPic4[0]) ? 0 : 1;
        ThresoldPic6[0] = (ThresoldPic6[0] == ThresoldPic5[0]) ? 0 : 1;
        ThresoldPic7[0] = (ThresoldPic7[0] == ThresoldPic6[0]) ? 0 : 1;

        SumPhoto[0] = (int)(ThresoldPic7[0] * 1 + ThresoldPic6[0] * 2 + ThresoldPic5[0] * 4 + ThresoldPic4[0] * 8 + ThresoldPic3[0] * 16 + ThresoldPic2[0] * 32 + ThresoldPic1[0] * 64);

        if (SumPhoto[0] % 2 == 0)
        {
            if (d_pPhotoAtan[0] <= 3.15 && d_pPhotoAtan[0] >= -3.15 / 2)
            {
                d_pOutputData[ii] = SumPhoto[0] * PI + d_pPhotoAtan[0];

            }
            else
            {
                d_pOutputData[ii] = (SumPhoto[0] + 2)*PI + d_pPhotoAtan[0];
            }
        }
        if (SumPhoto[0] % 2 != 0)
        {
            if (d_pPhotoAtan[0] <= 3.15 && d_pPhotoAtan[0] >= 3.15 / 2)
            {
                d_pOutputData[ii] = (SumPhoto[0] - 1)*PI + d_pPhotoAtan[0];

            }
            else
            {
                d_pOutputData[ii] = (SumPhoto[0] + 1)*PI + d_pPhotoAtan[0];

            }
        }



    }

}
__device__ void Direct(int n, double a[3][3], double b[3])
{
    bool flag = true;
    double y[3];
    int r;
    int i;
    int k;
    double sum_u, sum_l;
    for (r = 1; r < n; r++)
        a[r][0] = a[r][0] / a[0][0];


    for (r = 1; r < n; r++)
    {
        for (i = r; i < n; i++)
        {
            sum_u = 0;
            for (k = 0; k < r; k++)
                sum_u += a[r][k] * a[k][i];

            a[r][i] = a[r][i] - sum_u;
        }
        for (i = r + 1; i < n && r != n - 1; i++)
        {
            sum_l = 0;
            for (k = 0; k < r; k++)
                sum_l += a[i][k] * a[k][r];

            a[i][r] = (a[i][r] - sum_l) / a[r][r];
        }
    }
    for (r = 0; r < n; r++)
    {
        if (a[r][r] == 0)
        {
            flag = false;
            return;
        }
    }
    y[0] = b[0];
    double sum;
    for (i = 1; i < n; i++)
    {
        sum = 0;
        for (k = 0; k < i; k++)
            sum += a[i][k] * y[k];
        y[i] = b[i] - sum;
    }

    b[n - 1] = y[n - 1] / a[n - 1][n - 1];
    for (i = n - 2; i >= 0; i--)
    {
        sum = 0;
        for (k = i + 1; k < n; k++)
            sum += a[i][k] * b[k];
        b[i] = (y[i] - sum) / a[i][i];
    }
    return;
}
__device__ double dot_double(double3 a, double3 b)
{

    return a.x * b.x + a.y * b.y + a.z * b.z;
}
__device__ double3 normalize_double(double3 v)
{
    double3 result;
    double invLen = 1.0 / sqrt(dot_double(v, v));
    result.x = v.x * invLen;
    result.y = v.y * invLen;
    result.z = v.z * invLen;
    return result;
}
__device__ double length_double(double3 v)
{
    return sqrt(dot_double(v, v));
}
void Call_d_atan2(unsigned char *pPixel_tywhite, unsigned char *pPixel_tyblack, unsigned char *pPixel1, unsigned char *pPixel2, unsigned char *pPixel3, unsigned char *pPixel4, unsigned char *pPixel_ty1, unsigned char *pPixel_ty2, unsigned char *pPixel_ty3, unsigned char *pPixel_ty4, unsigned char *pPixel_ty5, unsigned char *pPixel_ty6, unsigned char *pPixel_ty7, double *pOutputData, const int image_size)
{

    //DWORD dwStart; 	DWORD dwEnd; double t;
    unsigned char *d_pPixel_tywhite;
    unsigned char *d_pPixel_tyblack;
    unsigned char *d_pPixel1, *d_pPixel2, *d_pPixel3, *d_pPixel4;
    unsigned char *d_pPixel_ty1, *d_pPixel_ty2, *d_pPixel_ty3, *d_pPixel_ty4, *d_pPixel_ty5, *d_pPixel_ty6, *d_pPixel_ty7;
    double *d_pOutputData;

    cudaMalloc((void **)&d_pPixel1, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel2, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel3, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel4, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_ty1, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_ty2, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_ty3, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_ty4, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_ty5, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_ty6, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_ty7, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_tywhite, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pPixel_tyblack, sizeof(unsigned char) * ImageWidth * ImageHeight * 1);
    cudaMalloc((void **)&d_pOutputData, sizeof(double) * ImageWidth * ImageHeight * 1);

    cudaMemcpy(d_pPixel_tywhite, pPixel_tywhite, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_tyblack, pPixel_tyblack, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel1, pPixel1, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel2, pPixel2, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel3, pPixel3, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel4, pPixel4, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_ty1, pPixel_ty1, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_ty2, pPixel_ty2, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_ty3, pPixel_ty3, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_ty4, pPixel_ty4, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_ty5, pPixel_ty5, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_ty6, pPixel_ty6, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_pPixel_ty7, pPixel_ty7, sizeof(unsigned char) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);

    uint threadsPerBlock = 1024;
    uint blocksPerGrid;
    blocksPerGrid = (image_size + threadsPerBlock - 1) / threadsPerBlock;

    d_atan2 << < blocksPerGrid, threadsPerBlock >> > (d_pPixel_tywhite, d_pPixel_tyblack, d_pPixel1, d_pPixel2, d_pPixel3, d_pPixel4, d_pPixel_ty1, d_pPixel_ty2, d_pPixel_ty3, d_pPixel_ty4, d_pPixel_ty5, d_pPixel_ty6, d_pPixel_ty7, d_pOutputData, image_size);
    cudaThreadSynchronize();
    cudaMemcpy(pOutputData, d_pOutputData, sizeof(double) * ImageWidth * ImageHeight * 1, cudaMemcpyDeviceToHost);
    cudaFree(d_pPixel_tywhite);
    cudaFree(d_pPixel_tyblack);
    cudaFree(d_pPixel1);
    cudaFree(d_pPixel2);
    cudaFree(d_pPixel3);
    cudaFree(d_pPixel4);
    cudaFree(d_pPixel_ty1);
    cudaFree(d_pPixel_ty2);
    cudaFree(d_pPixel_ty3);
    cudaFree(d_pPixel_ty4);
    cudaFree(d_pPixel_ty5);
    cudaFree(d_pPixel_ty6);
    cudaFree(d_pPixel_ty7);
    cudaFree(d_pOutputData);
}
void Call_StereoMatching_GAll(Mat Mask1_zero[GP_Num], Mat Mask2_zero[GP_Num],
    Mat w1_1[GP_Num], Mat w2_1[GP_Num],
    Mat correspondent_lines[GP_Num], Mat correspondent_lines2[GP_Num],
    Mat M[GP_Num], Mat M_2[GP_Num], Mat M2[GP_Num], Mat M2_Mir[GP_Num], Mat M1[GP_Num], Mat M1_2[GP_Num],
    double *m_distortion[GP_Num], double *m_distortion_2[GP_Num],
    double *pPhase1h[GP_Num], double *pPhase1v[GP_Num], double *pPhase2h[GP_Num], double *pPhase2v[GP_Num],
    double P, double XOriginCentral, double YOriginCentral,
    double MatchRange, double HeightRange, double AngleRange,
    double *OutPut3DData[GP_Num], double *OutPut3DData_norm[GP_Num], double *OutPut3DData_dis[GP_Num],
    int ImageHeight, int ImageWidth, char *CudaError)
{

    unsigned char *d_Mask1_zero[GP_Num], *d_Mask2_zero[GP_Num];
    double *d_w1_1[GP_Num], *d_w2_1[GP_Num];
    double *d_M[GP_Num], *d_M_2[GP_Num], *d_M2[GP_Num], *d_M2_Mir[GP_Num], *d_M1[GP_Num], *d_M1_2[GP_Num];
    double *d_correspondent_lines[GP_Num], *d_correspondent_lines2[GP_Num];
    double *d_distortion[GP_Num], *d_distortion_2[GP_Num];
    double *d_pPhase1h[GP_Num], *d_pPhase1v[GP_Num], *d_pPhase2h[GP_Num], *d_pPhase2v[GP_Num];
    double *d_P;
    double *d_XOriginCentral;
    double *d_YOriginCentral;
    double *d_MatchRange;
    double *d_HeightRange;
    double *d_AngleRange;
    double *d_OutPut3DData[GP_Num];
    double *d_OutPut3DData_norm[GP_Num];
    double *d_OutPut3DData_dis[GP_Num];
    int uint = 16;
    dim3 block(uint, uint);
    dim3 grid((ImageWidth + block.x - 1) / block.x, (ImageHeight + block.y - 1) / block.y);

    for (int ii = 0; ii < GP_Num; ii++)
    {
        if (ii == 0)
        {

            cudaMalloc((void **)&d_Mask1_zero[ii], sizeof(unsigned char) * ImageWidth * ImageHeight);
            cudaMemcpy(d_Mask1_zero[ii], Mask1_zero[ii].data, sizeof(unsigned char) * ImageWidth * ImageHeight, cudaMemcpyHostToDevice);
            cudaMalloc((void **)&d_M[ii], sizeof(double) * 3 * 4);
            cudaMemcpy(d_M[ii], M[ii].data, sizeof(double) * 3 * 4, cudaMemcpyHostToDevice);
            cudaMalloc((void **)&d_M_2[ii], sizeof(double) * 3 * 4);
            cudaMemcpy(d_M_2[ii], M_2[ii].data, sizeof(double) * 3 * 4, cudaMemcpyHostToDevice);
            cudaMalloc((void **)&d_correspondent_lines[ii], sizeof(double) * ImageWidth * ImageHeight * 3);
            cudaMemcpy(d_correspondent_lines[ii], correspondent_lines[ii].data, sizeof(double) * ImageWidth * ImageHeight * 3, cudaMemcpyHostToDevice);
            cudaMalloc((void **)&d_pPhase1v[ii], sizeof(double) * ImageWidth * ImageHeight * 1);
            cudaMalloc((void **)&d_pPhase2v[ii], sizeof(double) * ImageWidth * ImageHeight * 1);
            cudaMemcpy(d_pPhase1v[ii], pPhase1v[ii], sizeof(double) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
            cudaMemcpy(d_pPhase2v[ii], pPhase2v[ii], sizeof(double) * ImageWidth * ImageHeight * 1, cudaMemcpyHostToDevice);
            cudaMalloc((void **)&d_OutPut3DData[ii], sizeof(double) * ImageWidth *ImageHeight * 3);
            cudaMemset(d_OutPut3DData[ii], 0, sizeof(double) * ImageWidth * ImageHeight * 3);

            StereoMatching_GAll << < grid, block >> >(d_Mask1_zero[ii],d_M[ii], d_M_2[ii], d_correspondent_lines[ii],d_pPhase1v[ii], d_pPhase2v[ii],d_OutPut3DData[ii],MatchRange,ImageHeight, ImageWidth);
            cudaError err = cudaGetLastError();	if (err != cudaSuccess) { const char *StrTemp; StrTemp = cudaGetErrorString(err); OutputString(StrTemp, CudaError); }
            else { char *StrTemp = "cudaSuccess"; OutputString(StrTemp, CudaError); }

            cudaMemcpy(OutPut3DData[ii], d_OutPut3DData[ii], sizeof(double)*ImageWidth * ImageHeight * 3, cudaMemcpyDeviceToHost);
            cudaFree(d_OutPut3DData[ii]);
            cudaFree(d_Mask1_zero[ii]);
            cudaFree(d_M[ii]);
            cudaFree(d_correspondent_lines[ii]);
            cudaFree(d_pPhase1v[ii]);
            cudaFree(d_pPhase2v[ii]);
            cudaDeviceSynchronize();
        }
    }
}
void OutputString(const char *InputString, char *OutputChar)
{
    int Snum = 0;
    Snum = strlen(InputString);
    for (int i = 0; i < Snum; i++)
    {
        OutputChar[i] = InputString[i];
    }
}
__global__ void StereoMatching_GAll(unsigned char *d_Mask1_zero,double *d_M, double *d_M_2,double *d_correspondent_lines,double *d_pPhase1v,double *d_pPhase2v,double *d_OutPut3DData,double MatchRange,int ImageHeight, int ImageWidth)
{
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int idy = threadIdx.y + blockIdx.y * blockDim.y;
    float MatchDis = 1;
    //================================Group0=================================================================================================
    if (idx < ImageWidth && idy < ImageHeight && idx % 1 == 0 && idy % 1 == 0)
    {
        int k = 0;
        double u1, v1, u1_2, v1_2;
        int MatchX = idx;
        int MatchY = idy;
        double A[3][3], B[3];
        double m11, m12, m13, m14,
               m21, m22, m23, m24,
               m31, m32, m33, m34;
        double m11_2, m12_2, m13_2, m14_2,
               m21_2, m22_2, m23_2, m24_2,
               m31_2, m32_2, m33_2, m34_2;

        double Pixel1_Mask;
        struct Temp
        {
            double SN;
            double angle;
            double x, y, z, nx, ny, nz, lcdx, lcdy, lcdz;
            double recNum;
            double sumX, sumY, sumZ;

        }	Temp;
        double3 X;


        //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        m11 = d_M[0 * 4 + 0];      m12 = d_M[0 * 4 + 1];       m13 = d_M[0 * 4 + 2];       m14 = d_M[0 * 4 + 3];
        m21 = d_M[1 * 4 + 0];      m22 = d_M[1 * 4 + 1];       m23 = d_M[1 * 4 + 2];       m24 = d_M[1 * 4 + 3];
        m31 = d_M[2 * 4 + 0];      m32 = d_M[2 * 4 + 1];       m33 = d_M[2 * 4 + 2];       m34 = d_M[2 * 4 + 3];

        m11_2 = d_M_2[0 * 4 + 0];   m12_2 = d_M_2[0 * 4 + 1];   m13_2 = d_M_2[0 * 4 + 2];   m14_2 = d_M_2[0 * 4 + 3];
        m21_2 = d_M_2[1 * 4 + 0];   m22_2 = d_M_2[1 * 4 + 1];   m23_2 = d_M_2[1 * 4 + 2];   m24_2 = d_M_2[1 * 4 + 3];
        m31_2 = d_M_2[2 * 4 + 0];   m32_2 = d_M_2[2 * 4 + 1];   m33_2 = d_M_2[2 * 4 + 2];   m34_2 = d_M_2[2 * 4 + 3];


        //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        //--------------------------------
        //Temp.SN = -99999;
        Temp.angle = 0;
        Temp.x = 0;
        Temp.y = 0;
        Temp.z = 0;
        Temp.nx = 0;
        Temp.ny = 0;
        Temp.nz = 0;
        Temp.lcdx = 0;
        Temp.lcdy = 0;
        Temp.lcdz = 0;
        Temp.sumX = 0;
        Temp.sumY = 0;
        Temp.sumZ = 0;
        Temp.recNum = 0;



        double temp = 0, PhaseTempL = 0, PhaseTempR = 0, PhaseTempR_1;
        struct TempPhase
        {
            double TempPhaseValue; double x, y; bool Flag;
        }TempPhase;

        TempPhase.TempPhaseValue = 999999;
        TempPhase.x = -1;
        TempPhase.y = -1;
        TempPhase.Flag = false;



        u1 = (double)MatchX;
        v1 = (double)MatchY;


        //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        Pixel1_Mask = (double)d_Mask1_zero[(int)v1 * ImageWidth + (int)u1];
        //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        if (Pixel1_Mask == 255)
        {
            k = MatchY * ImageWidth + MatchX;
            double EPL1, EPL2, EPL3;
            //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            EPL1 = (double)d_correspondent_lines[k * 3 + 2];
            EPL2 = (double)d_correspondent_lines[k * 3 + 0];
            EPL3 = (double)d_correspondent_lines[k * 3 + 1];
            //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


            for (double AnyX = u1 - MatchRange; AnyX < u1 + MatchRange; AnyX = AnyX + MatchDis)
            {

                u1_2 = AnyX;
                v1_2 = (-1 * EPL1 - EPL2 * AnyX) / EPL3;

                if (u1 >= 1 && u1 < ImageWidth - 1 && v1 >= 1 && v1 < ImageHeight - 1 && u1_2 >= 1 && u1_2 < ImageWidth - 1 && v1_2 > 1 && v1_2 < ImageHeight - 1)
                {

                            PhaseTempL = *(d_pPhase1v + (int)v1 * ImageWidth + (int)u1);
                            PhaseTempR = *(d_pPhase2v + (int)v1_2 * ImageWidth + (int)u1_2);
                            PhaseTempR_1 = *(d_pPhase2v + (int)v1_2 * ImageWidth + (int)u1_2 - 1);
                            temp = (PhaseTempL - PhaseTempR_1)*(PhaseTempL - PhaseTempR);
                            if (temp < 0 && (fabs(PhaseTempL - PhaseTempR_1) < (0.5))	&& (fabs(PhaseTempL - PhaseTempR) < (0.5)))//理论上3.14/2
                            {

                                //TempPhase.x = 1 / (PhaseTempR - PhaseTempR_1)*(PhaseTempL - PhaseTempR_1) + u1_2 - 1;
                                TempPhase.x = u1_2;
                                TempPhase.y = (int)((-1 * EPL1 - EPL2 *TempPhase.x) / EPL3);
                                TempPhase.Flag = true;
                                break;
                            }

                }

            }
            //===================matching end=====================
            if (TempPhase.Flag == true)
            {
                u1_2 = TempPhase.x;
                v1_2 = TempPhase.y;
                A[0][0] = (m11 - u1 * m31);			A[0][1] = (m12 - u1 * m32);					A[0][2] = (m13 - u1 * m33);
                A[1][0] = (m21 - v1 * m31);			A[1][1] = (m22 - v1 * m32);					A[1][2] = (m23 - v1 * m33);
                A[2][0] = (m11_2 - u1_2 * m31_2);	A[2][1] = (m12_2 - u1_2 * m32_2);			A[2][2] = (m13_2 - u1_2 * m33_2);

                B[0] = (u1*m34 - m14);
                B[1] = (v1*m34 - m24);
                B[2] = (u1_2*m34_2 - m14_2);

                Direct(3, A, B);
                X.x = B[0];
                X.y = B[1];
                X.z = B[2];
                Temp.x = X.x;
                Temp.y = X.y;
                Temp.z = X.z;

                *(d_OutPut3DData + idy * ImageWidth * 3 + idx * 3 + 0) = Temp.x;
                *(d_OutPut3DData + idy * ImageWidth * 3 + idx * 3 + 1) = Temp.y;
                *(d_OutPut3DData + idy * ImageWidth * 3 + idx * 3 + 2) = Temp.z;
            }

            //TempPhase.TempPhaseValue = 999999;
            //TempPhase.x = -1;
            //TempPhase.y = -1;
            TempPhase.Flag = false;
        }


    }

}




