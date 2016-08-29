#include "Kinect2Manager.h"
#include <opencv2\opencv.hpp>

struct KinectFrame{
	cv::Mat img_rgba;
	cv::Mat img_rgba_mapped_to_depth;
	cv::Mat img_depth;
	cv::Mat img_infrared;
	cv::Mat img_rgba_body;
	cv::Mat img_rgba_mapped_to_depth_body;
	std::vector<Joint> joints;
	std::vector<JointOrientation> joint_orientations;
	cv::Mat map_color_to_depth;
	cv::Mat map_depth_to_color;
	int lefthand_confidence, righthand_confidence;
	INT64 depth_time;
	INT64 color_time;
};


void save(std::string dir, std::vector<KinectFrame>& frames,
	int& offset){

	std::stringstream filename_ss;
	cv::FileStorage fs;

	for (int i = 0; i < frames.size(); ++i){

		if (!frames[i].img_rgba.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba" << offset + i << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba);
		}

		if (!frames[i].img_rgba_body.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba_body" << offset + i << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba_body);
		}

		if (!frames[i].img_rgba_mapped_to_depth.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba_depthmapped" << offset + i << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba_mapped_to_depth);
		}

		if (!frames[i].img_rgba_mapped_to_depth_body.empty()){
			filename_ss.str("");
			filename_ss << dir << "/rgba_depthmapped_body" << offset + i << ".png";
			cv::imwrite(filename_ss.str(), frames[i].img_rgba_mapped_to_depth_body);
		}

		if (!frames[i].img_depth.empty()){
			filename_ss.str("");
			filename_ss << dir << "depth" << offset + i << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);
			fs << "depth" << frames[i].img_depth;
			fs.release();
		}


		if (!frames[i].joints.empty()){
			filename_ss.str("");
			filename_ss << dir << "joints" << offset + i << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);

			fs << "frame" << i
				<< "lefthand" << frames[i].lefthand_confidence
				<< "righthand" << frames[i].righthand_confidence
				<< "depth_time" << std::to_string(frames[i].depth_time)
				<< "color_time" << std::to_string(frames[i].color_time)
				<< "joints" << "[";

			for (int j = 0; j < frames[i].joints.size(); ++j){
				cv::Vec4f pos(frames[i].joints[j].Position.X,
					frames[i].joints[j].Position.Y,
					frames[i].joints[j].Position.Z,
					1);
				cv::Vec4f orientation(	frames[i].joint_orientations[j].Orientation.x,
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
			filename_ss << dir << "map_color_to_depth" << offset + i << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);
			fs << "map_color_to_depth" << frames[i].map_color_to_depth;
			fs.release();
		}

		if (!frames[i].map_depth_to_color.empty()){
			filename_ss.str("");
			filename_ss << dir << "map_depth_to_color" << offset + i << ".yml";
			fs.open(filename_ss.str(), cv::FileStorage::WRITE);
			fs << "map_depth_to_color" << frames[i].map_depth_to_color;
			fs.release();
		}

		std::cout << "Frame " << offset + i << " saved.\n";
	}



	offset += frames.size();
	frames.clear();
}

void save(std::string dir, std::vector<cv::Mat>& rgba_mats,
	std::vector<cv::Mat>& body_rgba_mats,
	std::vector<cv::Mat>& depth_mats,
	std::vector<std::vector<Joint>>& joint_frames,
	std::vector<std::vector<JointOrientation>>& joint_orientation_frames,
	std::vector<cv::Mat>& depthspace_pts_mats,
	std::vector<int>& lefthand_confidence_frames,
	std::vector<int>& righthand_confidence_frames,
	std::vector<INT64>& depth_times,
	std::vector<INT64>& color_times,
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
			<< "depth_time" << (int)(depth_times[i])
			<< "color_time" << (int)(color_times[i])
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
	depth_times.clear();
	color_times.clear();
}

RGBQUAD * buffer;
/**
	args:
	KinectRead [directory] [startframe]

	controls:
	r = start/stop recording
	s = stop recording and save
	q = save and quit
	d = dump buffers (what is this?)
*/

int main(int argc, char ** argv){

	buffer = new RGBQUAD[CAPTURE_SIZE_X_DEPTH * CAPTURE_SIZE_Y_DEPTH];

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
	kinect_manager.InitializeDefaultSensor();

	cv::FileStorage fs;
	std::stringstream filename_ss;
	int counter = 0;

	std::vector<KinectFrame> frames;
	int offset = 0;

	bool b_record = false;

	cv::namedWindow("img");

	cv::Mat depth_prev;

	while (true){
		kinect_manager.Update(Update::Color | Update::Depth | Update::DepthRGBX | Update::Body | Update::BodyIndex | Update::MapColorToDepth | Update::Infrared);
		
		int width = kinect_manager.getDepthWidth();
		int height = kinect_manager.getDepthHeight();

		int color_width = kinect_manager.getColorWidth();
		int color_height = kinect_manager.getColorHeight();
		RGBQUAD* rgbx = kinect_manager.GetColorRGBX();
		RGBQUAD* rgbx_todepth = kinect_manager.GetColorMappedToDepth();

		USHORT* depth = kinect_manager.GetDepth();
		RGBQUAD* body_rgbx_todepth = kinect_manager.GetBodyDepthRGBX();
		RGBQUAD * depth_rgbx = kinect_manager.GetDepthRGBX();

		USHORT * infrared = kinect_manager.GetInfrared();

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
			cv::Mat depth_rgba(height, width, CV_8UC4, depth_rgbx);
			cv::imshow("depth img", depth_rgba);
			cv::Mat depth_mat(height, width, cv::DataType<USHORT>::type, depth);
			cv::Mat infrared_mat(height, width, cv::DataType<USHORT>::type, infrared);

			convert_ushort_to_color(infrared, buffer, width * height);
			cv::Mat infrared_mat_rgb(height, width, CV_8UC4, buffer);
			cv::imshow("infrared img", infrared_mat_rgb);

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

			cv::Mat rgba_todepth_mat(height, width, CV_8UC4, rgbx_todepth);
			cv::Mat body_rgba_mat(height, width, CV_8UC4, body_rgbx_todepth);

			if (color_height > 0 && color_width > 0){

				cv::imshow("img", rgba_todepth_mat);
				cv::imshow("body img", body_rgba_mat);
				
			}

			if (different){

				try{
					if (b_record){
						KinectFrame frame;
						frame.img_depth = depth_mat.clone();
						frame.img_infrared = infrared_mat.clone();

						if (color_height > 0 && color_width > 0){
							cv::Mat rgba_mat(color_height, color_width, CV_8UC4, rgbx);
							frame.img_rgba = rgba_mat.clone();

							frame.img_rgba_mapped_to_depth = rgba_todepth_mat.clone();
							frame.img_rgba_body = body_rgba_mat.clone();
						}

						INT64 depth_time = kinect_manager.GetDepthTime();
						INT64 color_time = kinect_manager.GetColorTime();

						frame.depth_time = depth_time;
						frame.depth_time = color_time;

						cv::Mat body_rgba_mat(height, width, CV_8UC4, body_rgbx_todepth);

						frame.joints = (std::vector<Joint>(num_joints));
						frame.joint_orientations = (std::vector<JointOrientation>(num_joints));

						for (int j = 0; j < JointType_Count; ++j){
							frame.joints[j] = joints[j];
							frame.joint_orientations[j] = joint_orientations[j];
						}

						frame.lefthand_confidence = (kinect_manager.getHandLeftConfidence());
						frame.righthand_confidence = (kinect_manager.getHandRightConfidence());


						int * depth_space_X = kinect_manager.GetColorXMappedToDepth();
						int * depth_space_Y = kinect_manager.GetColorYMappedToDepth();

						//static int num = 0;
						//char name[256];
						//FILE *fp;
						//sprintf_s(name, "%s\\dtocx%03d.pgm", dir.c_str(), num);
						//fopen_s(&fp, name, "wb");
						//fprintf( fp, "P5\n%d %d\n65536\n", width*2, height);
						//fwrite(depth_space_X, 1, width*height*4, fp);
						//fclose(fp);
						//sprintf_s(name, "%s\\bodyindex%03d.pgm", dir.c_str(), num);
						//fopen_s(&fp, name, "wb");
						//fprintf(fp, "P5\n%d %d\n65536\n", width * 2, height);
						//fwrite(kinect_manager.m_pBodyIndex, 1, width*height * 4, fp);
						//fclose(fp);
						//num++;
						//exit(1);

						//cv::Mat depthspace_pts(height, width, cv::DataType<cv::Vec2s>::type, cv::Scalar(-1, -1));
						//
						//for (int y = 0; y < height; ++y){
						//	for (int x = 0; x < width; ++x){
						//		depthspace_pts.ptr<cv::Vec2s>(y)[x] = cv::Vec2s(*depth_space_X, *depth_space_Y);
						//		++depth_space_X;
						//		++depth_space_Y;
						//	}
						//}
						//
						//frame.map_color_to_depth = (depthspace_pts);
						std::cout << "Frame " << frames.size() + offset << " recorded\n";

						frames.push_back(frame);
					}

					depth_prev = depth_mat.clone();
				}
				catch (std::exception e){
					std::cout << "Memory full! Saving\n";
					save(dir, frames, offset);

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
			save(dir, frames, offset);
			b_record = false;

			if (q == 'q')
				return 0;
		}
		else if (q == 'd'){
			kinect_manager.DumpBuffers();
		}
	}

	delete buffer;
}