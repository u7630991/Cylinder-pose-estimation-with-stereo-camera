#include <chrono>

using namespace std;
using namespace chrono;

#if ENABLE_TRIGGER

#include "sbaslercameracontrol.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sys/stat.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>


#define MSG_SINGLE  "s"
#define MSG_RO      "r"
#define MSG_STARTED "0"
#define MSG_SUCCESS "1"
#define MSG_FAILED  "2"

using namespace cv;

class TriggerClient
{
    public:
        TriggerClient(string addr, string port);
        int request();

    private:
        string m_addr, m_port;

};
#endif