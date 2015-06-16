#include "Kinect2Manager.h"
#include <opencv2\opencv.hpp>

void save(std::string dir, std::vector<cv::Mat>& rgba_mats,
	std::vector<cv::Mat>& body_rgba_mats,
	std::vector<cv::Mat>& depth_mats,
	std::vector<std::vector<Joint>>& joint_frames,
	std::vector<std::vector<JointOrientation>>& joint_orientation_frames,
	std::vector<cv::Mat>& depthspace_pts_mats,
	std::vector<int>& lefthand_confidence_frames,
	std::vector<int>& righthand_confidence_frames,
	int& offset_rgba,
	int& offset_body_rgba,
	int& offset_depth,
	int& offset_joint,
	int& offset_joint_orientation,
	int& offset_depthspace,
	int& offset_lefthand,
	int& offset_righthand){

	std::stringstream filename_ss;
	cv::FileStorage fs;

	for (int i = 0; i < rgba_mats.size(); ++i){
		filename_ss.str("");
		filename_ss << dir << "/rgbx" << offset_rgba + i << ".png";
		cv::imwrite(filename_ss.str(), rgba_mats[i]);
	}
	for (int i = 0; i < body_rgba_mats.size(); ++i){
		filename_ss.str("");
		filename_ss << dir << "/body_rgbx" << offset_body_rgba + i << ".png";
		cv::imwrite(filename_ss.str(), body_rgba_mats[i]);
	}
	for (int i = 0; i < depth_mats.size(); ++i){
		filename_ss.str("");
		filename_ss << dir << "depth" << offset_depth + i << ".yml";
		fs.open(filename_ss.str(), cv::FileStorage::WRITE);
		fs << "depth" << depth_mats[i];
		fs.release();
	}
	for (int i = 0; i < joint_frames.size(); ++i){
		filename_ss.str("");
		filename_ss << dir << "joints" << offset_joint + i << ".yml";
		fs.open(filename_ss.str(), cv::FileStorage::WRITE);

		fs << "frame" << i
			<< "lefthand" << lefthand_confidence_frames[i]
			<< "righthand" << righthand_confidence_frames[i]
			<< "joints" << "[";

		for (int j = 0; j < joint_frames[i].size(); ++j){
			cv::Vec4f pos(joint_frames[i][j].Position.X,
				joint_frames[i][j].Position.Y,
				joint_frames[i][j].Position.Z,
				1);
			cv::Vec4f orientation(joint_orientation_frames[i][j].Orientation.x,
				joint_orientation_frames[i][j].Orientation.y,
				joint_orientation_frames[i][j].Orientation.z,
				joint_orientation_frames[i][j].Orientation.w);
			fs << "{" << "joint_type" << (int)joint_frames[i][j].JointType
				<< "position" << pos
				<< "tracking_state" << (int)joint_frames[i][j].TrackingState
				<< "orientation" << orientation
				<< "}";
		}

		fs << "]";
		fs.release();
	}
	for (int i = 0; i < depthspace_pts_mats.size(); ++i){
		filename_ss.str("");
		filename_ss << dir << "depthspace" << offset_depthspace + i << ".yml";
		fs.open(filename_ss.str(), cv::FileStorage::WRITE);
		fs << "depthspace" << depthspace_pts_mats[i];
		fs.release();
	}

	offset_rgba += rgba_mats.size();
	offset_body_rgba += body_rgba_mats.size();
	offset_depth += depth_mats.size();
	offset_joint += joint_frames.size();
	offset_joint_orientation += joint_orientation_frames.size();
	offset_depthspace += depthspace_pts_mats.size();
	offset_lefthand += lefthand_confidence_frames.size();
	offset_righthand += righthand_confidence_frames.size();

	rgba_mats.clear();
	body_rgba_mats.clear();
	depth_mats.clear();
	joint_frames.clear();
	joint_orientation_frames.clear();
	depthspace_pts_mats.clear();
	lefthand_confidence_frames.clear();
	righthand_confidence_frames.clear();
}

int main(int argc, char ** argv){

	std::string dir;
	if (argc == 2){
		dir = std::string(argv[1]);
	}
	else{
		dir = "vid/";
	}

	CreateDirectory(dir.c_str(), NULL);

	Kinect2Manager kinect_manager;
	kinect_manager.InitializeDefaultSensor();

	cv::FileStorage fs;
	std::stringstream filename_ss;
	int counter = 0;
	
	int offset_rgba = 0;
	int offset_body_rgba = 0;
	int offset_depth = 0;
	int offset_joint = 0;
	int offset_joint_orientation = 0;
	int offset_depthspace = 0;
	int offset_lefthand = 0;
	int offset_righthand = 0;

	std::vector<cv::Mat> rgba_mats;
	std::vector<cv::Mat> body_rgba_mats;
	std::vector<cv::Mat> depth_mats;

	std::vector<cv::Mat> depthspace_pts_mats;

	std::vector<std::vector<Joint>> joints_frames;
	std::vector<std::vector<JointOrientation>> joint_orientations_frames;
	std::vector<int> lefthand_confidence_frames;
	std::vector<int> righthand_confidence_frames;

	bool b_record = false;

	cv::namedWindow("img");

	cv::Mat depth_prev;

	while (true){
		kinect_manager.Update(Update::Color | Update::Depth | Update::DepthRGBX | Update::Body | Update::BodyIndex | Update::MapColorToDepth);
		
		int width = kinect_manager.getDepthWidth();
		int height = kinect_manager.getDepthHeight();
		RGBQUAD* rgbx = kinect_manager.GetColorMappedToDepth();
		USHORT* depth = kinect_manager.GetDepth();
		RGBQUAD* body_rgbx = kinect_manager.GetBodyDepthRGBX();
		RGBQUAD * depth_rgbx = kinect_manager.GetDepthRGBX();

		//kinect_manager.Update(Update::Color | Update::Depth | Update::Body | Update::BodyIndex | Update::MapDepthToColor);
		//
		//int width = kinect_manager.getColorWidth();
		//int height = kinect_manager.getColorHeight();
		//RGBQUAD* rgbx = kinect_manager.GetColorRGBX();
		//USHORT* depth = kinect_manager.GetDepthMappedToColor();
		//RGBQUAD* body_rgbx = kinect_manager.GetBodyColorRGBX();


		int num_joints = JointType_Count;
		Joint* joints = kinect_manager.GetJoints();
		JointOrientation* joint_orientations = kinect_manager.GetJointOrientations();

		if (height > 0 && width > 0){
			cv::Mat rgba_mat(height, width, CV_8UC4, rgbx);
			cv::Mat body_rgba_mat(height, width, CV_8UC4, body_rgbx);
			cv::Mat depth_rgba(height, width, CV_8UC4, depth_rgbx);
			cv::imshow("img", rgba_mat);
			cv::imshow("body img", body_rgba_mat);
			cv::imshow("depth img", depth_rgba);
			cv::Mat depth_mat(height, width, cv::DataType<USHORT>::type, depth);

			if (depth_prev.empty()){
				depth_prev.create(depth_mat.size(), depth_mat.type());
			}

			bool different = false;
			for (int i = depth_mat.rows*depth_mat.cols / 2 + depth_mat.cols / 2; i < depth_mat.rows*depth_mat.cols; ++i){
				if (depth_mat.ptr<USHORT>()[i] != depth_prev.ptr<USHORT>()[i]){
					different = true;
					break;
				}
			}
			if (different){

				try{
					if (b_record){
						cv::Mat body_rgba_mat(height, width, CV_8UC4, body_rgbx);

						cv::Mat new_rgba_mat = rgba_mat.clone();
						cv::Mat new_body_rgba_mat = body_rgba_mat.clone();
						cv::Mat new_depth_mat = depth_mat.clone();

						joints_frames.push_back(std::vector<Joint>(num_joints));
						joint_orientations_frames.push_back(std::vector<JointOrientation>(num_joints));

						for (int j = 0; j < JointType_Count; ++j){
							joints_frames.back()[j] = joints[j];
							joint_orientations_frames.back()[j] = joint_orientations[j];
						}

						lefthand_confidence_frames.push_back(kinect_manager.getHandLeftConfidence());
						righthand_confidence_frames.push_back(kinect_manager.getHandRightConfidence());

						rgba_mats.push_back(new_rgba_mat);
						body_rgba_mats.push_back(new_body_rgba_mat);
						depth_mats.push_back(new_depth_mat);

						int * depth_space_X = kinect_manager.GetDepthXMappedToColor();
						int * depth_space_Y = kinect_manager.GetDepthYMappedToColor();

						cv::Mat depthspace_pts(height, width, cv::DataType<cv::Vec2s>::type, cv::Scalar(-1, -1));

						for (int y = 0; y < height; ++y){
							for (int x = 0; x < width; ++x){
								depthspace_pts.ptr<cv::Vec2s>(y)[x] = cv::Vec2s(*depth_space_X, *depth_space_Y);
								++depth_space_X;
								++depth_space_Y;
							}
						}

						depthspace_pts_mats.push_back(depthspace_pts);
						std::cout << "Frame " << rgba_mats.size() + offset_rgba << " recorded\n";
					}

					depth_prev = depth_mat.clone();
				}
				catch (std::exception e){
					std::cout << "Memory full! Saving\n";
					save(dir, rgba_mats, body_rgba_mats, depth_mats, joints_frames, joint_orientations_frames, depthspace_pts_mats,
						lefthand_confidence_frames, righthand_confidence_frames,
						offset_rgba, offset_body_rgba, offset_depth, offset_joint, offset_joint_orientation, offset_depthspace, offset_lefthand,offset_righthand);

					rgba_mats.clear();
					body_rgba_mats.clear();
					depth_mats.clear();
					joints_frames.clear();
					joint_orientations_frames.clear();
					lefthand_confidence_frames.clear();
					righthand_confidence_frames.clear();

					b_record = false;
				}
			}

			//
			//filename_ss.str("");
			//filename_ss << dir << "/rgbx" << counter << ".png";
			//cv::imwrite(filename_ss.str(), rgba_mat);
			//
			//filename_ss.str("");
			//filename_ss << dir << "/body_rgbx" << counter << ".png";
			//cv::imwrite(filename_ss.str(), body_rgba_mat);
			//
			//filename_ss.str("");
			//filename_ss << dir << "depth" << counter << ".yml";
			//fs.open(filename_ss.str(), cv::FileStorage::WRITE);
			//fs << "depth" << depth_mat;
			//fs.release();
			++counter;
		}

		char q = cv::waitKey(15);
		if (q == 'r'){
			b_record = !b_record;
		}
		else if (q == 'q' || q=='s'){
			save(dir, rgba_mats, body_rgba_mats, depth_mats, joints_frames, joint_orientations_frames, depthspace_pts_mats,
				lefthand_confidence_frames, righthand_confidence_frames,
				offset_rgba, offset_body_rgba, offset_depth, offset_joint, offset_joint_orientation, offset_depthspace, offset_lefthand,offset_righthand);
			b_record = false;

			if (q == 'q')
				return 0;
		}
	}
}