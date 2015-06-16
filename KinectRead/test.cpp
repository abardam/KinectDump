#include "Kinect2Manager.h"
#include <opencv2\opencv.hpp>

int main(){
	Kinect2Manager kinect_manager;
	kinect_manager.InitializeDefaultSensor();

	while (true){
		kinect_manager.UpdateColor();
		kinect_manager.UpdateDepth();
		kinect_manager.UpdateBody();
		kinect_manager.UpdateBodyFrameIndex();
		
		int width = kinect_manager.getColorWidth();
		int height = kinect_manager.getColorHeight();
		RGBQUAD* rgb = kinect_manager.GetBodyColorRGBX();

		if (height > 0 && width > 0){
			//cv::Mat rgb_mat(height, width, CV_8UC3);
			//for (int h = 0; h < height; ++h){
			//	for (int w = 0; w < width; ++w){
			//		char blue = rgb[h*width + w].rgbBlue;
			//		char green = rgb[h*width + w].rgbGreen;
			//		char red = rgb[h*width + w].rgbRed;
			//		rgb_mat.ptr<cv::Vec3b>(h)[w] = cv::Vec3b(blue, green, red);
			//	}
			//}
			cv::Mat rgba_mat(height, width, CV_8UC4, rgb);

			cv::imshow("img", rgba_mat);
		}

		cv::waitKey(5);
	}
}