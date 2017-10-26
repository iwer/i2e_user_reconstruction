#ifndef _PCLUTIL_H_
#define _PCLUTIL_H_

#include "../gen/KinectFrameMessage.pb.h"
#include "recon/typedefs.h"

class PCLUtil{
public:
	static void convertToXYZPointCloud(KinectFrameMessage& message, uint16_t* depth, int height, int width){
		int step = 3;
		float center_x = (float) (width >> 1);
		float center_y = (float) (height >> 1);
		float f_inv = 1.0f / 517.4f; // Inverse from 640 / (2 * tan(58.5 / 2)) = 517.4
		//float f_inv = 1.0f / 525; //found here http://www.pcl-users.org/Getting-strange-results-when-moving-from-depth-map-to-point-cloud-td4025104.html#a4025138

		int depth_idx = 0; //index for depth data array
		int cloud_idx = 0; //index for pointcloud

		message.clear_cloud();
		for (int h = 0; h < height; h++){
			for (int w = 0; w < width; w++, cloud_idx += step){
				float tmp = depth[depth_idx++] * 0.001f;
				message.add_cloud(tmp);

				tmp = tmp * f_inv;
				message.add_cloud((w - center_x) * tmp);
				message.add_cloud((h - center_y) * tmp);
			}
		}
	};

	static void convertToXYZPointCloud(float* cloud, const char* depth, int height, int width){
		int step = 3;
		float center_x = (float) (width >> 1);
		float center_y = (float) (height >> 1);
		float f_inv = 1.0f / 517.4f; // Inverse from 640 / (2 * tan(58.5 / 2)) = 517.4
		//float f_inv = 1.0f / 525; //found here http://www.pcl-users.org/Getting-strange-results-when-moving-from-depth-map-to-point-cloud-td4025104.html#a4025138

		int depth_idx = 0; //index for depth data array
		int cloud_idx = 0; //index for pointcloud

		for (int h = 0; h < height; h++){
			for (int w = 0; w < width; w++, cloud_idx += step, depth_idx += 2){
				uint16_t depth_value = depth[depth_idx] | (depth[depth_idx + 1] << 8);
				float tmp = depth_value * 0.001f;
				cloud[cloud_idx] = tmp;

				tmp = tmp * f_inv;
				cloud[cloud_idx + 1] = (w - center_x) * tmp;
				cloud[cloud_idx + 2] = (h - center_y) * tmp;
			}
		}
	};

	static void convertToXYZPointCloud(recon::CloudPtr pc, const char* depth, int height, int width){
		int step = 3;
		float center_x = (float) (width >> 1);
		float center_y = (float) (height >> 1);
		float f_inv = 1.0f / 517.4f; // Inverse from 640 / (2 * tan(58.5 / 2)) = 517.4
		//float f_inv = 1.0f / 525; //found here http://www.pcl-users.org/Getting-strange-results-when-moving-from-depth-map-to-point-cloud-td4025104.html#a4025138

		int depth_idx = 0; //index for depth data array
		int cloud_idx = 0; //index for pointcloud


		recon::PointType p;

		for (int h = 0; h < height; h++){
			for (int w = 0; w < width; w++, cloud_idx += 1, depth_idx += 2){
				uint16_t val1 = depth[depth_idx] < 0 ? (uint16_t) (((int) depth[depth_idx]) + 255) : (uint16_t) depth[depth_idx];
				uint16_t val2 = depth[depth_idx + 1] < 0 ? (uint16_t) (((int) depth[depth_idx + 1]) + 255) : (uint16_t) depth[depth_idx + 1];


				//float depth_value = ((int) depth[depth_idx]) + (((int) depth[depth_idx + 1]) << 8);
				float depth_value = val1 + (val2 << 8);
				float tmp = (float) depth_value; // * 0.001f;
				p.z = tmp * 0.001f;

				tmp = tmp * f_inv * 0.001f;
				p.x = (w - center_x) * tmp;
				p.y = (h - center_y) * -tmp;

				//pc->push_back(p);
				pc->at(cloud_idx) = p;
			}
		}
	};

	~PCLUtil();

private:
	PCLUtil();
};

#endif
