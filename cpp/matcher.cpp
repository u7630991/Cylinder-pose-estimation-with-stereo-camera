#include "matcher.h"

class PhaseBasedStereoMatcher
{
    public:
        PhaseBasedStereoMatcher(int imgW, int imgH, const char* group);
        double* match(long long capToken, int index);
    
    private:
        int m_imgW = 0;
        int m_imgH = 0;
        int m_3DDataLen;
        int CodeAmp = 2;

        string groupName;

        map<string, vector<string>> staticModelPathsMap = {
            {"vial", { "static/vial_1213_calibL.xyz", "static/vial_1213_calibM.xyz", "static/vial_1213_calibR.xyz",
                        "static/vial_1213_sampling1.xyz", "static/vial_1213_sampling2.xyz" }},
            {"beaker", { "static/beaker_1214_set1_L.xyz", "static/beaker_1214_set1_R.xyz", 
                        "static/beaker_1214_set2_L.xyz", "static/beaker_1214_set2_R.xyz" }}
        };
        
        #if ENABLE_COMPUTATION
        Pics *pics1h, *pics1v, *pics2h, *pics2v;

        double *pPhase1h, *pPhase1v, *pPhase2h, *pPhase2v;
        double *pPhase1h_copy[6], *pPhase1v_copy[6], *pPhase2h_copy[6], *pPhase2v_copy[6];
        double *OutPut3DData_G[6], *OutPut3DData_norm_G[6], *OutPut3DData_dis_G[6];
        double *m_distortion, *m_distortion_2;

        double xBoundMin, xBoundMax, yBoundMin, yBoundMax, zBoundMin, zBoundMax;
        double XOriginCentral, YOriginCentral, MatchRange, HeightRange, AngleRange, P;

        Mat correspondent_lines, correspondent_lines2;
        Mat M, M1, M2, M_2, M1_2, M2_2, M2_Mir;
        Mat w1_0, w1_1, w2_0, w2_1;

        vector<vector<Point>> rois1, rois2;

        void computePicAllPointsCorrespondEpilines();
        void initMM1M2matrix();
        void initMatricesFromCalibResult(string calibResultPath, 
            Mat &M, Mat &M1, Mat &M2, double *distortion);
        void preComputeInit();
        void initROIs();

        void readPics(string dir, Pics *pics);
        void gpuComputePhase(double *pOutPutdata, Pics *pics);
        void computeNormalStereoMultiThread_Cuda_GAll(const Mat &maskCam1, const Mat &maskCam2, 
            double *result, int writeFileFlag);

        #endif
};

PhaseBasedStereoMatcher::PhaseBasedStereoMatcher(int imgW, int imgH, const char* group)
{
    m_imgW = imgW;
    m_imgH = imgH;
    m_3DDataLen = imgW * imgH * 3;

    groupName = string(group);
    
    #if ENABLE_COMPUTATION
    P = imgW * CodeAmp / 64 * 0.1845;
    ImageWidth = imgW;
    ImageHeight = imgH;

    pPhase1h = new double[imgW * imgH];
    pPhase1v = new double[imgW * imgH];
    pPhase2h = new double[imgW * imgH];
    pPhase2v = new double[imgW * imgH];

    pics1h = new Pics(imgW, imgH);
    pics1v = new Pics(imgW, imgH);
    pics2h = new Pics(imgW, imgH);
    pics2v = new Pics(imgW, imgH);

    for (int ii = 0; ii < 1; ii++)
    {
        OutPut3DData_G[ii] = new double[m_3DDataLen];
        OutPut3DData_norm_G[ii] = new double[m_3DDataLen];
        OutPut3DData_dis_G[ii] = new double[m_3DDataLen];
        memset(OutPut3DData_G[ii], 0, m_3DDataLen * sizeof(double));
        memset(OutPut3DData_norm_G[ii], 0, m_3DDataLen * sizeof(double));
        memset(OutPut3DData_dis_G[ii], 0, m_3DDataLen * sizeof(double));
        pPhase1h_copy[ii] = new double[imgW * imgH];
        pPhase1v_copy[ii] = new double[imgW * imgH];
        pPhase2h_copy[ii] = new double[imgW * imgH];
        pPhase2v_copy[ii] = new double[imgW * imgH];
    }

    initROIs();
    computePicAllPointsCorrespondEpilines();
    initMM1M2matrix();
    preComputeInit();

    #endif
}

double* PhaseBasedStereoMatcher::match(long long capToken, int index)
{
    double* result = new double[m_3DDataLen];

    #if ENABLE_COMPUTATION

        string parentDirPath = "dynamic/" + to_string(capToken) + "/";

        /* Fallback to the test pics if the folder tagged with capToken doesn't exist */
        struct stat info;
        string dirPrefix = (stat(parentDirPath.c_str(), &info) != 0) ? 
            "calibration/" + groupName + "/CalibPic/NoTriggerTestStriPic" : parentDirPath;

        cout << "Going to match image pairs in: " << dirPrefix << endl;

        MatchRange = m_imgW / 2;
        
        readPics(dirPrefix + "1/H/", pics1h);
        readPics(dirPrefix + "1/V/", pics1v);
        readPics(dirPrefix + "2/H/", pics2h);
        readPics(dirPrefix + "2/V/", pics2v);

        gpuComputePhase(pPhase1h, pics1h);
        gpuComputePhase(pPhase1v, pics1v);
        gpuComputePhase(pPhase2h, pics2h);
        gpuComputePhase(pPhase2v, pics2v);


        Mat maskCam1, maskCam2;

        if (index >= 0 && index < rois1.size() && index < rois2.size())
        {
            maskCam1 = Mat::zeros(m_imgH, m_imgW, CV_8UC1);
            maskCam2 = Mat::zeros(m_imgH, m_imgW, CV_8UC1);
            
            vector<vector<Point>> roiList1 = { rois1[index] };
            vector<vector<Point>> roiList2 = { rois2[index] };

            fillPoly(maskCam1, roiList1, Scalar(255));
            fillPoly(maskCam2, roiList2, Scalar(255));

            bitwise_and(pics1v->getForeground(), maskCam1, maskCam1);
            bitwise_and(pics2v->getForeground(), maskCam2, maskCam2);
        }
        else
        {
            maskCam1 = pics1v->getForeground();
            maskCam2 = pics2v->getForeground();
        }       

        computeNormalStereoMultiThread_Cuda_GAll(maskCam1, maskCam2, result, 0);
        
    #else
        //Return 3D points from either of static models
        srand(time(0));

        int j = 0;

        if (staticModelPathsMap.count(groupName) > 0)
        {

            vector<string> staticModelPaths = staticModelPathsMap[groupName];
            const char* staticModelPath = staticModelPaths[rand()%(staticModelPaths.size())].c_str();
            
            cout << "Load static model as the result: " << staticModelPath << endl;

            FILE * fStatic;
            
            fStatic = fopen(staticModelPath, "r");
            if (fStatic != NULL) 
            {
                while (fscanf(fStatic, "%lf %lf %lf\n", &result[j], &result[j+1], &result[j+2]) != EOF)
                {
                    j += 3;
                }
                fclose(fStatic);
            }
        }
        else
            cout << "No static model available for " << groupName << endl;

        for (int k=j; k<m_3DDataLen; k++)
        {
            result[k] = numeric_limits<double>::quiet_NaN();
        }

    #endif
    
    return result;
}

#if ENABLE_COMPUTATION
struct ContourSorter
{
    bool operator ()( const vector<Point> &a, const vector<Point> &b )
    {
        Rect rectA(boundingRect(a));
        Rect rectB(boundingRect(b));
        return (rectA.x < rectB.x);
    }
};

void PhaseBasedStereoMatcher::computePicAllPointsCorrespondEpilines()
{
    // Read the pair of fundamental matrices
	double *fund_matrix = new double[9];
    double *fund_matrix_2 = new double[9];

    string pathFM1 = "calibration/" + groupName + 
        "/fundamental_matrix1/fundamental_matrix1.txt";
    FILE* fpFM1;
    fpFM1 = fopen(pathFM1.c_str(), "r");
    fscanf(fpFM1, "%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n",
        &fund_matrix[0], &fund_matrix[1], &fund_matrix[2],
        &fund_matrix[3], &fund_matrix[4], &fund_matrix[5],
        &fund_matrix[6], &fund_matrix[7], &fund_matrix[8]);

    string pathFM2 = "calibration/" + groupName + 
        "/fundamental_matrix2/fundamental_matrix2.txt";
    FILE* fpFM2;
    fpFM2 = fopen(pathFM2.c_str(), "r");
    fscanf(fpFM2, "%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n",
        &fund_matrix_2[0], &fund_matrix_2[1], &fund_matrix_2[2],
        &fund_matrix_2[3], &fund_matrix_2[4], &fund_matrix_2[5],
        &fund_matrix_2[6], &fund_matrix_2[7], &fund_matrix_2[8]);
	
	fclose(fpFM1);
    fclose(fpFM2);

	Mat fundamental_matrix;
    Mat fundamental_matrix_2;
    fundamental_matrix = Mat(3, 3, CV_64FC1);
    fundamental_matrix_2 = Mat(3, 3, CV_64FC1);

	for (int i = 0; i < 9; i++)
    {
        ((double *)fundamental_matrix.data)[i] = fund_matrix[i];
        ((double *)fundamental_matrix_2.data)[i] = fund_matrix_2[i];
    }

	// Prepare repsective input points for each pixels on both cameras
	Mat points;
    Mat points_2;
    points = Mat(m_imgW * m_imgH, 1, CV_64FC2);
    points_2 = Mat(m_imgW * m_imgH, 1, CV_64FC2);
    for (int i = 0; i < m_imgW * m_imgH; i++)
    {
        points.at<Vec2d>(i, 0)[0] = (int)(i % m_imgW);
        points.at<Vec2d>(i, 0)[1] = (int)(i / m_imgW);
    }
    for (int i = 0; i < m_imgW * m_imgH; i++)
    {
        points_2.at<Vec2d>(i, 0)[0] = (int)(i % m_imgW);
        points_2.at<Vec2d>(i, 0)[1] = (int)(i / m_imgW);
    }

	// Compute the epilines
	correspondent_lines = Mat(m_imgW * m_imgH, 3, CV_64FC1);
    correspondent_lines2 = Mat(m_imgW * m_imgH, 3, CV_64FC1);
    computeCorrespondEpilines(points, 1, fundamental_matrix, correspondent_lines);
    computeCorrespondEpilines(points_2, 1, fundamental_matrix_2, correspondent_lines2);
   
}

void PhaseBasedStereoMatcher::initMM1M2matrix()
{
    M = Mat(3, 4, CV_64FC1);
    M1 = Mat(3, 4, CV_64FC1);
    M2 = Mat(4, 4, CV_64FC1);
    m_distortion = new double[4];
	initMatricesFromCalibResult("calibration/" + groupName + 
        "/CalibPic/camera1pic/calibration_result.txt",
		M, M1, M2, m_distortion);

	M_2 = Mat(3, 4, CV_64FC1); 
    M1_2 = Mat(3, 4, CV_64FC1);
    M2_2 = Mat(4, 4, CV_64FC1);
    m_distortion_2 = new double[4];
	initMatricesFromCalibResult("calibration/" + groupName + 
        "/CalibPic/camera2pic/calibration_result.txt",
		M_2, M1_2, M2_2, m_distortion_2);
}

void PhaseBasedStereoMatcher::preComputeInit()
{
	//--------------------------------------------------------------------------------------------------------------------------------
    Mat CCam1 = Mat(4, 1, CV_64FC1);
    Mat CCam2 = Mat(4, 1, CV_64FC1);
    //--------------Get the coordinates of the camera orgin in the world coordinates--------------------------------------
    CCam1.at<double>(0, 0) = 0;
    CCam1.at<double>(1, 0) = 0;
    CCam1.at<double>(2, 0) = 0;
    CCam1.at<double>(3, 0) = 1;
    CCam2.at<double>(0, 0) = 0;
    CCam2.at<double>(1, 0) = 0;
    CCam2.at<double>(2, 0) = 0;
    CCam2.at<double>(3, 0) = 1;
    Mat M_World_Camera1 = Mat(4, 1, CV_64FC1);
    Mat M_World_Camera2 = Mat(4, 1, CV_64FC1);
    M_World_Camera1 = (M2.inv(DECOMP_SVD)) * CCam1;
    M_World_Camera2 = (M2_2.inv(DECOMP_SVD)) * CCam2;
    w1_0 = Mat(3, 1, CV_64FC1);	w1_1 = Mat(3, 1, CV_64FC1);
    w2_0 = Mat(3, 1, CV_64FC1);	w2_1 = Mat(3, 1, CV_64FC1);
    w1_1.at<double>(0, 0) = M_World_Camera1.at<double>(0, 0);
    w1_1.at<double>(1, 0) = M_World_Camera1.at<double>(1, 0);
    w1_1.at<double>(2, 0) = M_World_Camera1.at<double>(2, 0);
    w2_1.at<double>(0, 0) = M_World_Camera2.at<double>(0, 0);
    w2_1.at<double>(1, 0) = M_World_Camera2.at<double>(1, 0);
    w2_1.at<double>(2, 0) = M_World_Camera2.at<double>(2, 0);
}

void PhaseBasedStereoMatcher::initMatricesFromCalibResult(
    string calibResultPath, Mat &M, Mat &M1, Mat &M2, double *distortion)
{
    // Read calibration results for construction of camera matices
	int i = 0;
    FILE * cps;
    cps = fopen(calibResultPath.c_str(), "r");
    char *aa = new char[100];
    for (i = 0; i < 3; i++) { fgets(aa, 100, cps); }

    double *m_cameraMatrix, *m_rotMatrs, *m_transVects;
    m_cameraMatrix = new double[9];
    m_rotMatrs = new double[9];
    m_transVects = new double[3];
    fscanf(cps, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
        &m_cameraMatrix[0], &m_cameraMatrix[1], &m_cameraMatrix[2],
        &m_cameraMatrix[3], &m_cameraMatrix[4], &m_cameraMatrix[5],
        &m_cameraMatrix[6], &m_cameraMatrix[7], &m_cameraMatrix[8]);

	for (i = 0; i < 12; i++)
    {
        fgets(aa, 100, cps);
    }

    for (i = 0; i < 1; i++)
    {
        fscanf(cps, "%lf %lf %lf %lf\n", 
            &distortion[0], &distortion[1], &distortion[2], &distortion[3]);
    }

    for (i = 0; i < 4; i++)
    {
        fgets(aa, 100, cps);
    }
    fscanf(cps, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
        &m_rotMatrs[0], &m_rotMatrs[1], &m_rotMatrs[2], &m_transVects[0],
        &m_rotMatrs[3], &m_rotMatrs[4], &m_rotMatrs[5], &m_transVects[1],
        &m_rotMatrs[6], &m_rotMatrs[7], &m_rotMatrs[8], &m_transVects[2]);
    fclose(cps);

	Mat M_cameraMatrix, M_rotMatrs, M_transVects;
    M_cameraMatrix = Mat(3, 3, CV_64FC1);
    M_rotMatrs = Mat(3, 3, CV_64FC1);
    M_transVects = Mat(3, 1, CV_64FC1);
    for (i = 0; i < 9; i++)
    {
        ((double *)M_cameraMatrix.data)[i] = m_cameraMatrix[i];
        ((double *)M_rotMatrs.data)[i] = m_rotMatrs[i];
    }
    for (i = 0; i < 3; i++)
    {
        ((double *)M_transVects.data)[i] = m_transVects[i];
    }
    //-------M1-------------------
    double v00 = 0, v01 = 0, v02 = 0, v10 = 0, v11 = 0, v12 = 0, v20 = 0, v21 = 0, v22 = 0;
    v00 = M_cameraMatrix.at<double>(0, 0); v01 = M_cameraMatrix.at<double>(0, 1); v02 = M_cameraMatrix.at<double>(0, 2);
    v10 = M_cameraMatrix.at<double>(1, 0); v11 = M_cameraMatrix.at<double>(1, 1); v12 = M_cameraMatrix.at<double>(1, 2);
    v20 = M_cameraMatrix.at<double>(2, 0); v21 = M_cameraMatrix.at<double>(2, 1); v22 = M_cameraMatrix.at<double>(2, 2);
    M1.at<double>(0, 0) = v00; M1.at<double>(0, 1) = v01; M1.at<double>(0, 2) = v02; M1.at<double>(0, 3) = 0;
    M1.at<double>(1, 0) = v10; M1.at<double>(1, 1) = v11; M1.at<double>(1, 2) = v12; M1.at<double>(1, 3) = 0;
    M1.at<double>(2, 0) = v20; M1.at<double>(2, 1) = v21; M1.at<double>(2, 2) = v22; M1.at<double>(2, 3) = 0;

    //-------M2-------------------
    double w00 = 0, w01 = 0, w02 = 0, w03 = 0, w10 = 0, w11 = 0, w12 = 0, w13 = 0, w20 = 0, w21 = 0, w22 = 0, w23 = 0;
    w00 = M_rotMatrs.at<double>(0, 0); w01 = M_rotMatrs.at<double>(0, 1); w02 = M_rotMatrs.at<double>(0, 2); w03 = M_transVects.at<double>(0, 0);
    w10 = M_rotMatrs.at<double>(1, 0); w11 = M_rotMatrs.at<double>(1, 1); w12 = M_rotMatrs.at<double>(1, 2); w13 = M_transVects.at<double>(1, 0);
    w20 = M_rotMatrs.at<double>(2, 0); w21 = M_rotMatrs.at<double>(2, 1); w22 = M_rotMatrs.at<double>(2, 2); w23 = M_transVects.at<double>(2, 0);
    M2.at<double>(0, 0) = w00; M2.at<double>(0, 1) = w01; M2.at<double>(0, 2) = w02; M2.at<double>(0, 3) = w03;
    M2.at<double>(1, 0) = w10; M2.at<double>(1, 1) = w11; M2.at<double>(1, 2) = w12; M2.at<double>(1, 3) = w13;
    M2.at<double>(2, 0) = w20; M2.at<double>(2, 1) = w21; M2.at<double>(2, 2) = w22; M2.at<double>(2, 3) = w23;
    M2.at<double>(3, 0) = 0;   M2.at<double>(3, 1) = 0;   M2.at<double>(3, 2) = 0;   M2.at<double>(3, 3) = 1;
    M = M1 * M2;

	delete[]m_cameraMatrix;
    delete[]m_rotMatrs;
    delete[]m_transVects;
    delete[]aa;
}

void PhaseBasedStereoMatcher::initROIs()
{
    // Read 2D ROIs of slot positions from binary image pair
    Mat imgRois1 = imread("calibration/" + groupName + "/CalibPic/roi/camera1.png", 0);
    Mat imgRois2 = imread("calibration/" + groupName + "/CalibPic/roi/camera2.png", 0);
    vector<vector<Point> > contours1, contours2;
    
    findContours( imgRois1, contours1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours( imgRois2, contours2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for( size_t i = 0; i< contours1.size(); i++ )
    {
        vector<Point> contApprox;
        approxPolyDP(contours1[i], contApprox, 3, true);
        rois1.push_back(contApprox);
    }
    for( size_t i = 0; i< contours2.size(); i++ )
    {
        vector<Point> contApprox;
        approxPolyDP(contours2[i], contApprox, 3, true);
        rois2.push_back(contApprox);
    }

    sort(rois1.begin(), rois1.end(), ContourSorter());
    sort(rois2.begin(), rois2.end(), ContourSorter());

    // for( size_t i = 0; i< rois1.size(); i++ )
    // {
    //     cout << "rois1[" << i << "]:" << rois1[i] << endl;
    // }
    // for( size_t i = 0; i< rois2.size(); i++ )
    // {
    //     cout << "rois2[" << i << "]:" << rois2[i] << endl;
    // }

    // Read 3D bounds 
    string boundsPath = "calibration/" + groupName + "/CalibPic/roi/bounds.txt";
    FILE * cps;
    cps = fopen(boundsPath.c_str(), "r");
    char *aa = new char[10];
    fgets(aa, 10, cps);
    fscanf(cps, "%lf %lf\n", &xBoundMin, &xBoundMax);
    fgets(aa, 10, cps);
    fscanf(cps, "%lf %lf\n", &yBoundMin, &yBoundMax);
    fgets(aa, 10, cps);
    fscanf(cps, "%lf %lf\n", &zBoundMin, &zBoundMax);
    fclose(cps);
    delete[]aa;
}

void PhaseBasedStereoMatcher::readPics(string dir, Pics *pics)
{
    vector<Mat> imgs;
    for (int i=0; i<NUM_PICS; i++)
    {
        string path = dir + to_string(i) + ".bmp";
        // cout << path << endl;
        Mat m = imread(path, 0);
        imgs.push_back(m);
    }
    pics->update(imgs);
}

void PhaseBasedStereoMatcher::gpuComputePhase(double *pOutPutdata, Pics *pics)
{
    Call_d_atan2(pics->p_tywhite(), pics->p_tyblack(), 
		pics->p_1(), pics->p_2(), pics->p_3(), pics->p_4(),
		pics->p_ty1(), pics->p_ty2(), pics->p_ty3(), pics->p_ty4(),
		pics->p_ty5(), pics->p_ty6(), pics->p_ty7(), pOutPutdata, m_imgW*m_imgH);
}

void PhaseBasedStereoMatcher::computeNormalStereoMultiThread_Cuda_GAll(
    const Mat &maskCam1, const Mat &maskCam2, double *result, int writeFileFlag)
{
    Mat Mask1_zero[6];
    Mat Mask2_zero[6];
    Mask1_zero[0] = maskCam1;
    Mask2_zero[0] = maskCam2;
    
    //====================================================================================================
    char CudaError[100] = { ' ' };
    Mat w1_1_copy[GP_Num], w2_1_copy[GP_Num];
    Mat correspondent_lines_copy[GP_Num], correspondent_lines2_copy[GP_Num];
    Mat M_copy[GP_Num], M_2_copy[GP_Num], M2_copy[GP_Num], M2_Mir_copy[GP_Num];
    Mat M1_copy[GP_Num], M1_2_copy[GP_Num];
    double *m_distortion[GP_Num];
    double *m_distortion_2[GP_Num];
    for (int ii = 0; ii < GP_Num; ii++)
    {
        if (ii == 0)
        {
            m_distortion[ii] = new double[4];
            m_distortion_2[ii] = new double[4];

            w1_1.copyTo(w1_1_copy[ii]);
            w2_1.copyTo(w2_1_copy[ii]);
            correspondent_lines.copyTo(correspondent_lines_copy[ii]);
            correspondent_lines2.copyTo(correspondent_lines2_copy[ii]);
            M.copyTo(M_copy[ii]);
            M_2.copyTo(M_2_copy[ii]);
            M2.copyTo(M2_copy[ii]);
            M2_Mir.copyTo(M2_Mir_copy[ii]);
            M1.copyTo(M1_copy[ii]);
            M1_2.copyTo(M1_2_copy[ii]);

            memcpy(m_distortion[ii], m_distortion, sizeof(double) * 4);
            memcpy(m_distortion_2[ii],m_distortion_2, sizeof(double) * 4);

            memcpy(pPhase1h_copy[ii], pPhase1h, sizeof(double)*m_imgW*m_imgH);
            memcpy(pPhase1v_copy[ii], pPhase1v, sizeof(double)*m_imgW*m_imgH);
            memcpy(pPhase2h_copy[ii], pPhase2h, sizeof(double)*m_imgW*m_imgH);
            memcpy(pPhase2v_copy[ii], pPhase2v, sizeof(double)*m_imgW*m_imgH);
        }
    }

    const auto t0 = chrono::system_clock::now();

	Call_StereoMatching_GAll(
        Mask1_zero, Mask2_zero,
        w1_1_copy, w2_1_copy,
        correspondent_lines_copy, correspondent_lines2_copy,
        M_copy, M_2_copy, M2_copy, M2_Mir_copy, M1_copy, M1_2_copy,
        m_distortion, m_distortion_2,
        pPhase1h_copy, pPhase1v_copy, pPhase2h_copy, pPhase2v_copy,
        P, XOriginCentral, YOriginCentral,
        MatchRange, HeightRange, AngleRange,
        OutPut3DData_G, OutPut3DData_norm_G, OutPut3DData_dis_G,
        m_imgH, m_imgW, CudaError);
    
    const auto t = chrono::system_clock::now();

    //cout << "time_diff=" << setprecision(4) << chrono::duration<double>(t-t0).count() << "s" << endl;
	
	FILE *fp = NULL;
    if (writeFileFlag)
    {
        time_t now_tt = chrono::system_clock::to_time_t(t);
        tm tm = *localtime(&now_tt);
        ostringstream oss;
        oss << put_time(&tm, "%Y%m%d_%H%M%S");
        const char *testdata_path;
        string testdata = "dynamic/result3d_" + oss.str() + ".xyz";
        testdata_path = testdata.c_str();
        if (!(fp = fopen(testdata_path, "w"))) 
            cout << "cannot open" << endl; 
        else
            cout << "Point cloud: " << testdata << endl;
    }

	double xVal, yVal, zVal;
    int index;
	for (int j = 0; j < m_imgH; j = j + 1)
	{
		for (int i = 0; i < m_imgW; i = i + 1)
		{
			index = j * m_imgW * 3 + i * 3;
            xVal = OutPut3DData_G[0][index + 0];
            yVal = OutPut3DData_G[0][index + 1];
            zVal = OutPut3DData_G[0][index + 2];
            if (xVal != 0 && yVal != 0 && zVal != 0 /*&& 
                xVal >= xBoundMin && xVal <= xBoundMax &&
                yVal >= yBoundMin && yVal <= yBoundMax &&
                zVal >= zBoundMin && zVal <= zBoundMax*/)
			{
				*(result + index + 0) = xVal;
                *(result + index + 1) = yVal;
                *(result + index + 2) = zVal;
                if (fp != NULL)
                {
                    fprintf(fp, "%lf %lf %lf\n", xVal, yVal, zVal);
                }
			}
            else
            {
                for (int k=0; k<3; k++)
                {
                    *(result + index + k) = numeric_limits<double>::quiet_NaN();
                }
            }
		}
	}

    
    if (fp != NULL)
	{
        fclose(fp);
    }
}

#endif

extern "C" {
    PhaseBasedStereoMatcher* StereoMatcher_new(int imgW, int imgH, char* group) 
        { return new PhaseBasedStereoMatcher(imgW, imgH, group); }
    double* StereoMatcher_match(PhaseBasedStereoMatcher* matcher, 
                                long long capToken, int index) 
        { return matcher->match(capToken, index); }
}