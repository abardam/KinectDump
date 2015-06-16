#include "Kinect2Manager.h"
#include <opencv2\opencv.hpp>
#include <cv_pointmat_common.h>

struct TestCameraIntrinsics{
	float alpha, beta, gamma, u, v;
};

float row_mult_sum(const cv::Mat& a, unsigned int a_row, const cv::Mat& b, unsigned int b_row){
	cv::Mat pair_prod;
	cv::multiply(a(cv::Range(a_row, a_row + 1), cv::Range(0, a.cols)),
		b(cv::Range(b_row, b_row + 1), cv::Range(0, b.cols)),
		pair_prod);
	return cv::sum(pair_prod)(0);
}

TestCameraIntrinsics test_calculate_camera_matrix(cv::Mat X, cv::Mat B){
	float X1X1 = row_mult_sum(X, 0, X, 0);
	float X1X2 = row_mult_sum(X, 0, X, 1);
	float X1X3 = row_mult_sum(X, 0, X, 2);
	float X2X2 = row_mult_sum(X, 1, X, 1);
	float X2X3 = row_mult_sum(X, 1, X, 2);
	float X3X3 = row_mult_sum(X, 2, X, 2);
	float X1B1 = row_mult_sum(X, 0, B, 0);
	float X2B1 = row_mult_sum(X, 1, B, 0);
	float X3B1 = row_mult_sum(X, 2, B, 0);
	//float X1B2 = row_mult_sum(X, 0, B, 1);
	float X2B2 = row_mult_sum(X, 1, B, 1);
	float X3B2 = row_mult_sum(X, 2, B, 1);

	cv::Mat A = cv::Mat::zeros(5, 5, CV_32F);
	cv::Mat b = cv::Mat::zeros(5, 1, CV_32F);
	cv::Mat x;

	A.ptr<float>(0)[0] = X1X1;
	A.ptr<float>(1)[0] = A.ptr<float>(0)[1] = X1X2;
	A.ptr<float>(2)[0] = A.ptr<float>(0)[2] = X1X3;
	A.ptr<float>(1)[1] = X2X2;
	A.ptr<float>(2)[1] = A.ptr<float>(1)[2] = X2X3;
	A.ptr<float>(2)[2] = X3X3;

	A.ptr<float>(3)[3] = X2X2;
	A.ptr<float>(4)[3] = A.ptr<float>(3)[4] = X2X3;
	A.ptr<float>(4)[4] = X3X3;

	b.ptr<float>(0)[0] = X1B1;
	b.ptr<float>(1)[0] = X2B1;
	b.ptr<float>(2)[0] = X3B1;
	b.ptr<float>(3)[0] = X2B2;
	b.ptr<float>(4)[0] = X3B2;

	cv::solve(A, b, x, cv::DECOMP_CHOLESKY);

	TestCameraIntrinsics intrinsics;
	intrinsics.alpha = x.ptr<float>(0)[0];
	intrinsics.gamma = x.ptr<float>(1)[0];
	intrinsics.u = x.ptr<float>(2)[0];
	intrinsics.beta = x.ptr<float>(3)[0];
	intrinsics.v = x.ptr<float>(4)[0];

	return intrinsics;
}

int main(int argc, char ** argv){

	Kinect2Manager kinect_manager;
	kinect_manager.InitializeDefaultSensor();

	cv::FileStorage fs;
	std::stringstream filename_ss;
	int counter = 0;
	cv::namedWindow("img");

	while (true){
		kinect_manager.Update(Update::Color | Update::Depth);

		int width = kinect_manager.getColorWidth();
		int height = kinect_manager.getColorHeight();
		RGBQUAD* rgbx = kinect_manager.GetColorRGBX();
		USHORT* depth = kinect_manager.GetDepth();
		USHORT* depth_mapped = kinect_manager.GetDepthMappedToColor();

		int num_joints = JointType_Count;
		Joint* joints = kinect_manager.GetJoints();
		JointOrientation* joint_orientations = kinect_manager.GetJointOrientations();

		if (height > 0 && width > 0){
			cv::Mat rgba_mat(height, width, CV_8UC4, rgbx);
			cv::imshow("img", rgba_mat);

			//calculate camera space points

			int npoints = kinect_manager.getDepthWidth()*kinect_manager.getDepthHeight();
			std::vector<CameraSpacePoint> camera_space_points(npoints);
			//std::vector<ColorSpacePoint> color_space_points(npoints);

			kinect_manager.getCoordinateMapper()->MapDepthFrameToCameraSpace(
				npoints, depth, npoints, camera_space_points.data());

			//kinect_manager.getCoordinateMapper()->MapCameraPointsToColorSpace(
			//	npoints, camera_space_points.data(), npoints, color_space_points.data());

			//now calculate
			//std::vector<cv::Vec4f> color_pt_vec;
			//color_pt_vec.reserve(npoints);

			std::vector<cv::Vec4f> depth_pt_vec;
			depth_pt_vec.reserve(npoints);

			std::vector<cv::Vec4f> camera_pt_vec;

			int depthwidth = kinect_manager.getDepthWidth();

			for (int i = 0; i < npoints; ++i){
				float z = depth[i]/1000.f;
				if (z == 0) continue;
				//cv::Vec4f color_pt(color_space_points[i].X * z,
				//	color_space_points[i].Y * z,
				//	z,
				//	1);
				//color_pt_vec.push_back(color_pt);
				cv::Vec4f depth_pt((i%depthwidth)*z,
					(i / depthwidth)*z,
					z,
					1);
				depth_pt_vec.push_back(depth_pt);
				cv::Vec4f camera_pt(camera_space_points[i].X,
					camera_space_points[i].Y,
					camera_space_points[i].Z,
					1);
				camera_pt_vec.push_back(camera_pt);
			}

			//cv::Mat color_pt_mat = pointvec_to_pointmat(color_pt_vec);
			cv::Mat depth_pt_mat = pointvec_to_pointmat(depth_pt_vec);
			cv::Mat camera_pt_mat = pointvec_to_pointmat(camera_pt_vec);

			cv::Mat calculated_camera_matrix = 
				//color_pt_mat * 
				depth_pt_mat *
				camera_pt_mat.t() * (camera_pt_mat * camera_pt_mat.t()).inv();

			//fs.open("out_cameramatrix.yml", cv::FileStorage::WRITE);
			fs.open("out_cameramatrix_depth.yml", cv::FileStorage::WRITE);
			fs << "cameramatrix" << calculated_camera_matrix;
			fs.release();

			TestCameraIntrinsics test_int = test_calculate_camera_matrix(camera_pt_mat, depth_pt_mat);

			fs.open("out_cameramatrix_test.yml", cv::FileStorage::WRITE);
			fs << "alpha" << test_int.alpha << "beta" << test_int.beta
				<< "gamma" << test_int.gamma << "u" << test_int.u
				<< "v" << test_int.v;
			fs.release();

			CameraIntrinsics sdk_int;
			HRESULT hr = kinect_manager.getCoordinateMapper()->GetDepthCameraIntrinsics(&sdk_int);

			if (SUCCEEDED(hr)){
				fs.open("out_cameramatrix_sdk.yml", cv::FileStorage::WRITE);
				fs << "FocalLengthX" << sdk_int.FocalLengthX << "FocalLengthY" << sdk_int.FocalLengthY
					<< "PrincipalPointX" << sdk_int.PrincipalPointX << "PrincipalPointY" << sdk_int.PrincipalPointY
					<< "RadialDistortionSecondOrder" << sdk_int.RadialDistortionSecondOrder
					<< "RadialDistortionFourthOrder" << sdk_int.RadialDistortionFourthOrder
					<< "RadialDistortionSixthOrder" << sdk_int.RadialDistortionSixthOrder;

				fs.release();

				return 0;
			}
			++counter;
		}

		char q = cv::waitKey(1);
	}
}