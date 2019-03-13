/*
 * @ File name: rin_proc_lib.cpp
 * @ Effect: My persional ways to process images.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include "math.h"

#include "inc/includes.h"
#include "inc/parameters.h"

#include "inc/rin_videocapture.h"
#include "inc/rin_proc_lib.h"


using namespace std;
using namespace cv;


ImgProcCon img_porc_con;


ImgProcCon::ImgProcCon(void)
{
//    _cam_params_path = "../csu_rm_vision_v2.1/data/proc_params.xml";
}


// Arm to fill the lightbar's hole.
// Input image: Bin; Output image: Bin.
void fill_hole(Mat &in_img, Mat &out_img)
{
    Mat org_img = in_img;
    Mat cut_img;
    Size size = org_img.size();
    Mat temp = Mat::zeros(size.height + 2, size.width + 2, org_img.type()); // Make image bigger.

    org_img.copyTo(temp(Range(1, size.height + 1), Range(1, size.width + 1)));
    floodFill(temp, Point(0, 0), Scalar(255));
    temp(Range(1, size.height + 1), Range(1, size.width + 1)).copyTo(cut_img);
    out_img = org_img | (~cut_img);
}

// Calibration
void calibration_img_get(void)
{
    int idx = 0;
    RinVideoCapture mono_cap("/dev/video0", 3);
    Mat img;
    string path;
    mono_cap.set_format(MONO_IMAGE_X_SIZE, MONO_IMAGE_Y_SIZE, 1);
    mono_cap.set_FPS(FPS);
    mono_cap.set_exposure_time(false, 8);
    mono_cap.start_stream();
    while(1)
    {
        mono_cap >> img;
        cvNamedWindow("calibration_image", CV_WINDOW_AUTOSIZE);
        imshow("calibration_image", img);
        if(waitKey(1) != -1)
        {
            idx++;
            path = "/home/csu-rm-infantry-1/csu_rm_vision/vision_by_rinck/csu_rm_vision_v2.1/data/";
            path = path + to_string(idx);
            path = path + ".jpg";
            imwrite(path, img);
        }
    }
}
