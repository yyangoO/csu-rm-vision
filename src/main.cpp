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
//    RMVideoCapture cap1("/dev/video1", 3);
    cvNamedWindow("img0", CV_WINDOW_AUTOSIZE);
//    cvNamedWindow("img1", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("debug", CV_WINDOW_AUTOSIZE);


    Mat img0, img1, img2;

    cap0.setVideoFormat(1280, 720, 1);
    cap0.info();
    cap0.startStream();
//    cap1.setVideoFormat(2560, 720, 1);
//    cap1.info();
//    cap1.startStream();
    int idx = 0;
    while(1)
    {
//        Mat x = (Mat_<double>(2, 3) << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        params.param_init();
//        FileStorage fs(params.cam_params_xml_dir, FileStorage::WRITE);
//        fs << "monn_cam_matrix" << x;
//        fs.release();
        cap0 >> img0;
//        cap1 >> img1;
        imshow("img0", img0);
//        imshow("img1", img1);
        dev_cfg.mono_undistort(img0, img2, params);
        armor_mono.armor_mono_proc(img2, params);
        imshow("debug", img2);
        if(waitKey(1) != -1)
        {
//            idx++;
//            std::string a = std::to_string(idx);
//            a = "/home/ubuntu/csu_rm_vision/vision_by_rinck/2.0.0/csu_rm_vision/data/" + a;
//            a = a + ".jpg";
//            imwrite(a, img0);
            break;
        }
    }
    cvDestroyAllWindows();

    return 0;
}
