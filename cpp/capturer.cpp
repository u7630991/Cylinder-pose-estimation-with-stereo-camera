#include "capturer.h"


class ImageCapturer
{
    public:
        ImageCapturer();
        long long capture();

    private:
        #if ENABLE_TRIGGER
            SBaslerCameraControl *m_control;
            TriggerClient *m_client;

        #endif
        
};

ImageCapturer::ImageCapturer()
{
    #if ENABLE_TRIGGER
    /* Prepare cameras */

    FILE *fp;
    const char *settingPath;
    settingPath ="cpp/capturer_setting.txt";

    if (!(fp = fopen(settingPath, "r")))
    {
        cerr << "Cannot open setting file" << endl;
        exit(1);
    }

    char camSerial0[20], camSerial1[20], serverAddr[20], serverPort[20];
    fscanf(fp, "%s %s", camSerial0, camSerial1);
    fscanf(fp, "%s %s", serverAddr, serverPort);
    
    fclose(fp);

    cout << camSerial0 << " " << camSerial1 << " " << serverAddr << " " << serverPort << endl;

    m_client = new TriggerClient(serverAddr, serverPort);

    m_control = new SBaslerCameraControl();
    m_control->CamSerial0 = camSerial0;
    m_control->CamSerial1 = camSerial1;

    m_control->initSome();

    #endif
}

long long ImageCapturer::capture()
{
    int debug = 0;
    int grabbedCount1, grabbedCount2;
    long long timestamp = 0;

    #if ENABLE_TRIGGER
    /* Try until 26 pairs of photos are acquired within one request */
    do
    {
        grabbedCount1 = 0;
        grabbedCount2 = 0;

        /* Start camera grabbing */
        cout << "Prepare to acquire..." << endl;
        m_control->StartAcquire();

        m_control->Handler1->triggercount=0;
        m_control->Handler2->triggercount=0;
        m_control->Handler1->trigger=0;
        m_control->Handler2->trigger=0;

        m_control->SwitchCameraMode_Line1();

        waitKey(150);
        m_control->Handler1->triggercount=0;
        m_control->Handler2->triggercount=0;
        m_control->Handler1->trigger=1;
        m_control->Handler2->trigger=1;

        /* Make triggering request */
        int ret = m_client->request();
        cout << "Handler1=" << m_control->Handler1->triggercount 
            << " Handler2=" << m_control->Handler2->triggercount << endl;

        grabbedCount1 = m_control->Handler1->triggercount;
        grabbedCount2 = m_control->Handler2->triggercount;

        /* Stop camera grabbing */
        m_control->StopAcquire();
        m_control->SwitchCameraMode_Software();

        if (ret != 0)
        {
            break;
        }

        /* Prepare folders and save the images */
        timestamp = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
        string dirName = "dynamic/" + to_string(timestamp);
        mkdir(dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        vector<string> camDirs = {"1", "2"};
        vector<string> subDirs = {"H", "V"};
        for (int i=0; i<camDirs.size(); i++)
        {
            string camDirName = dirName + "/" + camDirs[i];
            mkdir(camDirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            for (int j=0; j<subDirs.size(); j++)
            {
                string subDirName = camDirName + "/" + subDirs[j];
                mkdir(subDirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            }
        }

        int ImageWidth=2448;
        int ImageHeight=2048;
        unsigned char *pImg1[26], *pImg2[26];

        const int plotScaleDown = 16;
        const int plotColNum = 9;
        const int plotRowNum = 3;
        const int plotW = ImageWidth/plotScaleDown;
        const int plotH = ImageWidth/plotScaleDown;
        Mat debugPlot = Mat::zeros(plotH*plotRowNum*2, plotW*plotColNum, CV_8UC1);
        Mat img1 = Mat(ImageHeight, ImageWidth, CV_8UC1);
        Mat img2 = Mat(ImageHeight, ImageWidth, CV_8UC1);
        Mat imgDown = Mat(plotH, plotW, CV_8UC1);

        for (int k=0; k<m_control->Handler1->triggercount; k++)
        {
            pImg1[k] = new unsigned char[ImageWidth * ImageHeight * 1];
            memcpy(pImg1[k], m_control->Handler1->targetImage_projector[k], ImageWidth * ImageHeight * 1 * sizeof(char));
            memcpy(img1.data, pImg1[k], ImageWidth * ImageHeight * 1);
            if (debug)
            {
                resize(img1, imgDown, Size(plotW, plotH), INTER_LINEAR);
                int px = (k%plotColNum)*plotW;
                int py = (k/plotColNum)*plotH;
                imgDown.copyTo(debugPlot(Rect(px, py, plotW, plotH)));
            }

            imwrite(dirName + "/" + camDirs[0] + "/" + subDirs[k/13] + "/" + to_string(k%13) + ".bmp", img1);

        }
        for (int k=0; k<m_control->Handler2->triggercount; k++)
        {
            pImg2[k] = new unsigned char[ImageWidth * ImageHeight * 1];
            memcpy(pImg2[k], m_control->Handler2->targetImage_projector[k], ImageWidth * ImageHeight * 1 * sizeof(char));   
            memcpy(img2.data, pImg2[k], ImageWidth * ImageHeight * 1);
            if (debug)
            {
                resize(img2, imgDown, Size(plotW, plotH), INTER_LINEAR);
                int px = (k%plotColNum)*plotW;
                int py = (k/plotColNum+plotRowNum)*plotH;
                imgDown.copyTo(debugPlot(Rect(px, py, plotW, plotH)));
            }

            imwrite(dirName + "/" + camDirs[1] + "/" + subDirs[k/13] + "/" + to_string(k%13) + ".bmp", img2);
        }

        cout << "Images saved: " << dirName << "/" << endl;

        if (debug)
        {
            imshow("Grabbed images", debugPlot);
            waitKey(0);
            destroyAllWindows();
        }
    } while(grabbedCount1 != 26 && grabbedCount1 != 26);

    #endif
    return timestamp;
}

#if ENABLE_TRIGGER
TriggerClient::TriggerClient(string addr, string port)
{
    m_addr = addr;
    m_port = port;
}

int TriggerClient::request()
{
    /* Set up socket */
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    char buffer[128] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        cerr << "\n Socket creation error" << endl;
        return -1;
    }
   
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(atoi(m_port.c_str()));
       
    if(inet_pton(AF_INET, m_addr.c_str(), &serv_addr.sin_addr)<=0) 
    {
        cerr << "Invalid address/ Address not supported" << endl;
        return -1;
    }
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        cerr << "Connection Failed" << endl;
        return -1;
    }

    /* Send triggering request to the server */
    send(sock, MSG_RO, 1, 0);
    cout << "MSG_RO sent" << endl;
    
    while(1)
    {
        valread = read( sock, buffer, 1);
        if ( strcmp( buffer, MSG_STARTED) == 0) 
        {
            cout << "Projection started" << endl;
        }
        else if ( strcmp( buffer, MSG_SUCCESS) == 0) 
        {
            cout << "Projection completed: Success" << endl;
            break;
        }
        else if ( strcmp( buffer, MSG_FAILED) == 0) 
        {
            cout << "Projection completed: Failed" << endl;
            return 1;
        }
    }

    return 0;
}
#endif

extern "C" {
    ImageCapturer* ImageCapturer_new() { return new ImageCapturer(); }
    long long ImageCapturer_capture(ImageCapturer* cap) { return cap->capture(); }

}