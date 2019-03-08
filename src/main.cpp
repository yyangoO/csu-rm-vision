#include <string>
#include "inc/includes.h"
#include "inc/RMVideoCapture.hpp"
#include "inc/parameters.h"
#include "inc/armor_monocular.h"
#include "inc/device.h"


using namespace cv;


int main(void)
{
    RMVideoCapture cap0("/dev/video0", 3);
    cvNamedWindow("img0", CV_WINDOW_AUTOSIZE);
    Mat img0;
    params.param_init();
    cap0.setVideoFormat(1280, 720, 1);
    cap0.setVideoFPS(60);
    cap0.setExposureTime(false, 20);
    cap0.info();
    cap0.startStream();
    rin_serial.serrial_cmd();
    while(1)
    {
        params.param_init();
        cap0 >> img0;
        imshow("img0", img0);
        armor_mono.armor_mono_proc(img0, params);
        rin_serial.msg_send();
        if(waitKey(1) != -1)
        {
            break;
        }
    }
    cvDestroyAllWindows();

    return 0;
}
