# CSU-RM-Vision（中南大学RoboMaster视觉2019）
* Author：杨洋

核心算法
-
主要思路就是对存在孔洞的形状进行填充
基本上能解决80%处理时遇到的问题，比如
* 由于摄像头成本导致的灯柱过爆，红蓝色中间泛白
* 大符激活后的扇叶是一个巨大的红蓝封闭轮廓，直接填充后是一个非常明显的特征

`void fill_hole(Mat &in_img, Mat &out_img)`
`{`
`    Mat org_img = in_img;`
`    Mat cut_img;`
`    Size size = org_img.size();`
`    Mat temp = Mat::zeros(size.height + 2, size.width + 2, org_img.type());`
`    org_img.copyTo(temp(Range(1, size.height + 1), Range(1, size.width + 1)));`
`    floodFill(temp, Point(0, 0), Scalar(255));`
`    temp(Range(1, size.height + 1), Range(1, size.width + 1)).copyTo(cut_img);`
`    out_img = org_img | (~cut_img);`
`}`
