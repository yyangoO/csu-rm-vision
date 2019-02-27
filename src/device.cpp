#include "inc/device.h"


using namespace cv;


DeviceCfg dev_cfg;


void DeviceCfg::mono_cam_init(void)
{

}

void DeviceCfg::mono_cam_cfg(void)
{

}

void DeviceCfg::bino_cam_init(void)
{

}

void DeviceCfg::bino_cam_cfg(void)
{

}

void DeviceCfg::serrial_init(void)
{

}

void DeviceCfg::serrial_i(void)
{

}

void DeviceCfg::serrial_o(void)
{

}

void DeviceCfg::mono_undistort(Mat &in_img, Mat &out_img, Params params)
{
    Mat org_img = in_img;
    params.mono_cam_matrix = Mat::eye(3, 3, CV_64F);
    params.mono_cam_matrix.at<double>(0, 0) = 2.009376763781166e+03;
    params.mono_cam_matrix.at<double>(0, 1) = 0.0;
    params.mono_cam_matrix.at<double>(0, 2) = 6.217296897304576e+02;
    params.mono_cam_matrix.at<double>(1, 0) = 0.0;
    params.mono_cam_matrix.at<double>(1, 1) = 2.009757978153955e+03;
    params.mono_cam_matrix.at<double>(1, 2) = 3.460564154128788e+02;
    params.mono_cam_matrix.at<double>(2, 0) = 0;
    params.mono_cam_matrix.at<double>(2, 1) = 0;
    params.mono_cam_matrix.at<double>(2, 2) = 1;
    params.mono_cam_distcoeffs = Mat::zeros(5, 1, CV_64F);
    params.mono_cam_distcoeffs.at<double>(0, 0) = -0.264034079363052;
    params.mono_cam_distcoeffs.at<double>(1, 0) = -0.812466874650191;
    params.mono_cam_distcoeffs.at<double>(2, 0) = 0;
    params.mono_cam_distcoeffs.at<double>(3, 0) = 0;
    params.mono_cam_distcoeffs.at<double>(4, 0) = 0;
    undistort(org_img, out_img, params.mono_cam_matrix, params.mono_cam_distcoeffs);
//            FileStorage fs(params.cam_params_xml_dir, FileStorage::WRITE);
//            fs << "mono_cam_matrix" << params.mono_cam_matrix;
//            fs << "mono_cam_distcoeffs" << params.mono_cam_distcoeffs;
//            fs.release();
}
