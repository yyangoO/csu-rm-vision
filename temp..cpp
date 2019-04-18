int main(void)
{
	Mat img, org_img, roi;
	Mat oc_element, oc_elemet_2;
	RotatedRect rotated_rect, target;
	Rect rectx;
	vector<vector<Point>> contours, target_contours;
	vector<Vec4i> hierachy, target_hierachy;
	Point2f rect_p[4], target_p[4], final_mid_p[2];
	float retated_area, contours_area, target_area;

	VideoCapture cap("F:\\project\\CV\\vision_run\\风车神符模拟击打短版.mp4");
	cvNamedWindow("org", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("out", CV_WINDOW_AUTOSIZE);
	while (1)
	{
		cap >> img;
		org_img = img;
		if (img.empty())
		{
			break;
		}
		hsv_proc(img, img);
		fill_hole(img, img);
		GaussianBlur(img, img, Size(3, 3), 0);
		oc_element = getStructuringElement(MORPH_RECT, Size(3, 3));
		oc_elemet_2 = getStructuringElement(MORPH_RECT, Size(50, 50));
		erode(img, img, oc_element);
		dilate(img, img, oc_element);
		findContours(img, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		for (size_t contours_idx = 0; contours_idx < contours.size(); contours_idx++)
		{
			rotated_rect = minAreaRect(contours[contours_idx]);
			rectx = boundingRect(contours[contours_idx]);
			rotated_rect.points(rect_p);
			retated_area = (float)(rotated_rect.size.height * rotated_rect.size.width);
			contours_area = (float)(contourArea(contours[contours_idx]));
			if (retated_area > 20000.0f)
			{
				//cout << "Rotated size: " << retated_area << endl;
				//cout << "Contours size: " << contours_area << endl;
				if ((contours_area / retated_area) < 0.6)
				{
					rectangle(org_img, rectx, Scalar(0, 255, 0), 1);


					erode(img, img, oc_elemet_2);
					dilate(img, img, oc_elemet_2);
					findContours(img, target_contours, target_hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
					for (size_t contours_idx_2 = 0; contours_idx_2 < target_contours.size(); contours_idx_2++)
					{
						target = minAreaRect(target_contours[contours_idx_2]);
						target_area = (float)(target.size.height * target.size.width);
						target.points(target_p);
						cout << "target size: " << target_area << endl;

						if ((target_area > 5000.0f) && (target_area < 20000.0f))
						{
							//for (size_t i = 0; i < 4; i++)
							//{
							//	line(org_img, target_p[i], target_p[(i + 1) % 4], Scalar(0, 255, 255), 1);
							//}
							circle(org_img, target.center, 8, Scalar(255, 0, 0), -1);
						}
					}
				}
			}
		}

		cout << endl;
		imshow("org", org_img);
		imshow("out", img);
		waitKey(1);
	}
	return 0;
}
