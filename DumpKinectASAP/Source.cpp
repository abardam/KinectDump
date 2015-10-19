#include "Kinect2Manager.h"
#include <opencv2\opencv.hpp>

#define DUMP_COLOR
#define DUMP_DEPTH


struct KinectFrame{
	cv::Mat img_rgba;
	cv::Mat img_rgba_mapped_to_depth;
	cv::Mat img_depth;
	cv::Mat img_rgba_body;
	cv::Mat img_rgba_mapped_to_depth_body;
	std::vector<Joint> joints;
	std::vector<JointOrientation> joint_orientations;
	cv::Mat map_color_to_depth;
	cv::Mat map_depth_to_color;
	int lefthand_confidence, righthand_confidence;
	INT64 time;
};


void save(std::string dir, std::vector<KinectFrame>& frames){

	std::stringstream filename_ss;
	cv::FileStorage fs;

	for (int i = 0; i < frames.size(); ++i){

		if (!frames[i].img_rgba.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba" << frames[i].time << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba);
		}

		if (!frames[i].img_rgba_body.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba_body" << frames[i].time << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba_body);
		}

		if (!frames[i].img_rgba_mapped_to_depth.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba_depthmapped" << frames[i].time << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba_mapped_to_depth);
		}

		if (!frames[i].img_rgba_mapped_to_depth_body.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba_depthmapped_body" << frames[i].time << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba_mapped_to_depth_body);
		}

		if (!frames[i].img_depth.empty()){
			filename_ss.str("");
			filename_ss << dir << "depth" << frames[i].time << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);
			fs << "depth" << frames[i].img_depth;
			fs.release();
		}


		if (!frames[i].joints.empty()){
			filename_ss.str("");
			filename_ss << dir << "joints" << frames[i].time << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);

			fs << "frame" << i
				<< "lefthand" << frames[i].lefthand_confidence
				<< "righthand" << frames[i].righthand_confidence
				<< "joints" << "[";

			for (int j = 0; j < frames[i].joints.size(); ++j){
				cv::Vec4f pos(frames[i].joints[j].Position.X,
					frames[i].joints[j].Position.Y,
					frames[i].joints[j].Position.Z,
					1);
				cv::Vec4f orientation(frames[i].joint_orientations[j].Orientation.x,
					frames[i].joint_orientations[j].Orientation.y,
					frames[i].joint_orientations[j].Orientation.z,
					frames[i].joint_orientations[j].Orientation.w);
				fs << "{" << "joint_type" << (int)frames[i].joints[j].JointType
					<< "position" << pos
					<< "tracking_state" << (int)frames[i].joints[j].TrackingState
					<< "orientation" << orientation
					<< "}";
			}

			fs << "]";
			fs.release();
		}

		if (!frames[i].map_color_to_depth.empty()){
			filename_ss.str("");
			filename_ss << dir << "map_color_to_depth" << frames[i].time << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);
			fs << "map_color_to_depth" << frames[i].map_color_to_depth;
			fs.release();
		}

		if (!frames[i].map_depth_to_color.empty()){
			filename_ss.str("");
			filename_ss << dir << "map_depth_to_color" << frames[i].time << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);
			fs << "map_depth_to_color" << frames[i].map_depth_to_color;
			fs.release();
		}

		std::cout << "Frame " << frames[i].time << " saved.\n";
	}

	frames.clear();
}

int main(int argc, char ** argv){

	std::string dir;
	if (argc >= 2){
		dir = std::string(argv[1]);
	}
	else{
		dir = "vid/";
	}
	int startframe = 0;
	if (argc >= 3){
		startframe = atoi(argv[2]);
	}
	else{
		startframe = 0;
	}
	

	CreateDirectory(dir.c_str(), NULL);

	Kinect2Manager kinect_manager;
	kinect_manager.InitializeDefaultSensorSeparateReaders();

	cv::FileStorage fs;
	std::stringstream filename_ss;
	int counter = 0;

	std::vector<KinectFrame> color_frames;
	std::vector<KinectFrame> depth_frames;
	int offset = 0;

	bool b_record = false;



	cv::namedWindow("img");

	cv::Mat depth_prev;

	while (true){

#ifdef DUMP_COLOR
		kinect_manager.Update(Update::Color);

		int color_width = kinect_manager.getColorWidth();
		int color_height = kinect_manager.getColorHeight();
		INT64 color_time = kinect_manager.GetColorTime();

		bool color_valid = color_height > 0 && color_width > 0;
#endif

#ifdef DUMP_DEPTH
		kinect_manager.Update(Update::Depth |Update::Body | Update::MapColorToDepth );

		int depth_width = kinect_manager.getDepthWidth();
		int depth_height = kinect_manager.getDepthHeight();
		INT64 depth_time = kinect_manager.GetDepthTime();

		bool depth_valid = depth_height > 0 && depth_width > 0;
#endif

#ifdef DUMP_COLOR
		if (color_valid){
			RGBQUAD * color_rgb = kinect_manager.GetColorRGBX();
			cv::Mat color_mat(color_height, color_width, CV_8UC4, color_rgb);

			if (b_record){
				if (color_frames.empty() || color_frames.back().time != color_time){
					color_frames.push_back(KinectFrame());
					color_frames.back().img_rgba = color_mat.clone();
					color_frames.back().time = color_time;
				}
			}
		}
#endif

#ifdef DUMP_DEPTH
		if (depth_valid){
			USHORT * depth = kinect_manager.GetDepth();
			cv::Mat depth_mat(depth_height, depth_width, cv::DataType<USHORT>::type, depth);

			if (b_record){
				if (depth_frames.empty() || depth_frames.back().time != depth_time){
					depth_frames.push_back(KinectFrame());

					depth_frames.back().img_depth = depth_mat.clone();

					//color map to depth

					int * depth_space_X = kinect_manager.GetColorXMappedToDepth();
					int * depth_space_Y = kinect_manager.GetColorYMappedToDepth();
					//depth-sized mat specifying which color coordinates
					cv::Mat depthspace_pts(depth_height, depth_width, cv::DataType<cv::Vec2s>::type, cv::Scalar(-1, -1));
					
					for (int y = 0; y < depth_height; ++y){
						for (int x = 0; x < depth_width; ++x){
							depthspace_pts.ptr<cv::Vec2s>(y)[x] = cv::Vec2s(*depth_space_X, *depth_space_Y);
							++depth_space_X;
							++depth_space_Y;
						}
					}
					
					depth_frames.back().map_color_to_depth = (depthspace_pts);


					//joints
					if (kinect_manager.getSkeletonIsGood()){
						int num_joints = JointType_Count;
						Joint* joints = kinect_manager.GetJoints();
						JointOrientation* joint_orientations = kinect_manager.GetJointOrientations();


						depth_frames.back().joints = (std::vector<Joint>(num_joints));
						depth_frames.back().joint_orientations = (std::vector<JointOrientation>(num_joints));

						for (int j = 0; j < JointType_Count; ++j){
							depth_frames.back().joints[j] = joints[j];
							depth_frames.back().joint_orientations[j] = joint_orientations[j];
						}

						depth_frames.back().lefthand_confidence = (kinect_manager.getHandLeftConfidence());
						depth_frames.back().righthand_confidence = (kinect_manager.getHandRightConfidence());
					}

					depth_frames.back().time = depth_time + startframe;
				}
			}
		}
#endif

		char q = cv::waitKey(15);
		if (q == 'r'){
			b_record = !b_record;
			if (b_record){
				std::cout << "start recording\n";
			}
			else{
				std::cout << "stop recording\n";
			}
		}
		else if (q == 'q' || q == 's'){
			save(dir, depth_frames);
			save(dir, color_frames);
			b_record = false;

			if (q == 'q')
				return 0;
		}
	}
}