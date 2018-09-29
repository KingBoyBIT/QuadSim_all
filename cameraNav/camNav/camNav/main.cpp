#include <stdio.h>
#include <stdlib.h>
#include <opencv2\core\core.hpp>
#include <opencv2\opencv.hpp>
#include <opencv\highgui.h>

using namespace cv;
unsigned int ret = 0;
int main()
{

	VideoCapture cap(0);
	//cap.open(0);
	cap.set(cv::CAP_PROP_POS_MSEC, 0.05);
	cap.set(cv::CAP_PROP_FPS, 30);
	cap.set(cv::CAP_PROP_FOCUS, 25);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
	//if (cap.isOpened() != true)
	//{
	//	return -1;
	//}
	while (1)
	{
		//Mat img = imread("Lenna_(test_image).png");
		Mat frame;
		cap >> frame;

		if (frame.data == NULL)
		{
			return -1;
		}
#if 1
		printf("size of img:%d,%d\n", frame.rows, frame.cols);
		imshow("lena", frame);
#if 1
		ret = waitKey(20);//会崩溃，没事不要用
#endif
		if (ret != -1)
		{
			return ret;
		}
#endif // DEBUG_FLAG
	}
	return ret;
}