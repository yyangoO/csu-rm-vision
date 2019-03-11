#include <string>

#include "inc/includes.h"
#include "inc/parameters.h"
#include "inc/armor_monocular.h"
#include "inc/device.h"

#include "inc/RMVideoCapture.hpp"


using namespace cv;


int main(void)
{
    RMVideoCapture mono_cap("/dev/video0", 3);
    Mat mono_img;
    params.param_init();
    mono_cap.setVideoFormat(MONO_IMAGE_X_SIZE, MONO_IMAGE_Y_SIZE, 1);
    mono_cap.setVideoFPS(FPS);
    mono_cap.setExposureTime(false, 15);
    mono_cap.info();
    mono_cap.startStream();
//    rin_serial.serrial_cmd();
    while(1)
    {
        mono_cap >> mono_img;
        armor_mono.armor_mono_proc(mono_img, params);
//        rin_serial.msg_send();
        if(waitKey(1) != -1)
        {
            break;
        }
    }
    cvDestroyAllWindows();
    return 0;
}
