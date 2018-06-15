// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


#include <pcl/io/openni2_grabber.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/compression/octree_pointcloud_compression.h>

#include <cmath>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include "lepcc_c_api.h"
#include "lepcc_types.h"


SYSTEMTIME operator-(const SYSTEMTIME& pSr, const SYSTEMTIME& pSl);

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; 
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pcl_rgb_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);


void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);
void draw_pointcloud_rgb(window& app, state& app_state, const std::vector<pcl_rgb_ptr>& points);
void draw_pointcloud_rgb2(window& app, state& app_state, std::vector<pcl::PointXYZRGB> points);

void DoPCLCompression(window &app, rs2::frameset &frames, rs2::pipeline &pipe, rs2::depth_frame &depth, rs2::points &points, rs2::pointcloud &pc, rs2::video_frame &color, state app_state);
void DoLEPCCCompression(window &app, rs2::frameset &frames, rs2::pipeline &pipe, rs2::depth_frame &depth, rs2::points &points, rs2::pointcloud &pc, rs2::video_frame &color, state app_state);


pcl_rgb_ptr points_to_pcl(const rs2::points& points, rs2::video_frame frame)
{
    pcl_rgb_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());


	auto vertices = points.get_vertices();              // get vertices
	auto tex_coords = points.get_texture_coordinates(); // and texture coordinates


	//auto color = (unsigned char*)frame.get_data();
	auto color = (uint8_t *)frame.get_data();

	auto width = frame.get_width();
	auto height = frame.get_height();
	//std::cout << "width:" << width << std::endl;
	//std::cout << "height:" << height << std::endl;

	for (int i = 0; i < points.size(); i++)
	{
//		if (vertices[i].z)
		{
			// upload the point and texture coordinates only for points we have depth data for
			cloud->points[i].x = vertices[i].x;
			cloud->points[i].y = vertices[i].y;

			cloud->points[i].z = vertices[i].z;

			auto coords = tex_coords[i];
			auto color_x = (int)((coords.u ) * frame.get_width());
			//auto color_y = (int)((1.0 + coords.v)  * frame.get_height());
			auto color_y = (int)((coords.v)  * frame.get_height());
			//auto color_x = (int)((coords.u) * frame.get_width());
			//auto color_y = (int)((1.0 + coords.v) * frame.get_height());


			if (color_x < 0 || color_x > width) {
				//std::cout << "errorx" << color_x << std::endl;
				//std::cout << coords.u << "," << coords.v << std::endl;
				continue;

			}

			if (color_y < 0 || color_y > height) {
				//std::cout << "errory" << color_y << std::endl;
				//std::cout << coords.u << "," << coords.v << std::endl;
				continue;
			}


			cloud->points[i].r = *(color + color_x * 3 + color_y * width * 3 + 0);
			cloud->points[i].g = *(color + color_x * 3 + color_y * width * 3 + 1);
			cloud->points[i].b = *(color + color_x * 3 + color_y * width * 3 + 2);


			// for debug
			/*
			if (0 < i && i < 100) {
				std::cout << coords.u << "," << coords.v << std::endl;

				std::cout << color_x << "," << color_y << std::endl;

				std::cout << (int)cloud->points[i].r << "," << cloud->points[i].g << std::endl;
			}
			*/
				
			//auto r = frame.get();
				//cloud->points[i].r =
//			vertices[i];
//			tex_coords[i]);
		}
	}

	/*
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
	*/

    return cloud;
}

pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}



float3 colors[] { { 0.8f, 0.1f, 0.3f }, 
                  { 0.1f, 0.9f, 0.5f },
                };

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense PCL Pointcloud Example");
	// Construct an object to manage view state
	state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();


	rs2::frameset frames;
	rs2::video_frame color = nullptr;
	rs2::depth_frame depth = nullptr;
	
	
	//DoPCLCompression(app, frames, pipe, depth, points, pc, color, app_state);
	DoLEPCCCompression(app, frames, pipe, depth, points, pc, color, app_state);


    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void DoLEPCCCompression(window &app, rs2::frameset &frames, rs2::pipeline &pipe, rs2::depth_frame &depth, rs2::points &points, rs2::pointcloud &pc, rs2::video_frame &color, state app_state)
{

	lepcc_ContextHdl ctx = lepcc_createContext();
	lepcc_ContextHdl ctxDec = lepcc_createContext();
	lepcc_status hr;

	double maxXErr = 9e-8, maxYErr = maxXErr;
	double maxZErr = 0.01;

	

	vector<lepcc::Point3D> ptVec, decPtVec;
	vector<lepcc::RGB_t> rgbVec, sortedRgbVec, decRgbVec;
	vector<uint32_t> orderVec;
	vector<lepcc::Byte> byteVec;


	while (app) // Application still alive?
	{

		double totalNumPoints = 0, totalNumTiles = 0;
		double minPts = 1e16, maxPts = 0;
		double maxDecErrX = 0, maxDecErrY = 0, maxDecErrZ = 0;
		double maxDecErrRGB = 0, totalDecErrRGB = 0;
		double maxDecErrIntensity = 0, totalDecErrIntensity = 0;
		int64_t totalNumBytesCompressed = 0;

		// Wait for the next set of frames from the camera
		frames = pipe.wait_for_frames();
		depth = frames.get_depth_frame();
		color = frames.get_color_frame();
		auto colorData = (uint8_t *)frames.get_data();
		
		// Tell pointcloud object to map to this color frame
		pc.map_to(color);

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);
			
		points.get_profile().as<rs2::video_stream_profile>();


		int pointCount = points.size();
		ptVec.resize(pointCount);
		rgbVec.resize(pointCount);
		auto vertices = points.get_vertices();
		auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
		int width = color.get_width();
		int height = color.get_height();
		auto pclPoints2 = points_to_pcl(points, color);
		for (int i = 0; i < pointCount; i++)
		{
			ptVec[i].x = pclPoints2->points[i].x;
			ptVec[i].y = pclPoints2->points[i].y;
			ptVec[i].z = pclPoints2->points[i].z;
			rgbVec[i].r = pclPoints2->points[i].r;
			rgbVec[i].g = pclPoints2->points[i].g;
			rgbVec[i].b = pclPoints2->points[i].b;
			/*ptVec[i].x = vertices[i].x;
			ptVec[i].y = vertices[i].y;
			ptVec[i].z = vertices[i].z;

			
			auto coords = tex_coords[i];
			auto color_x = (int)((coords.u) * width);
			auto color_y = (int)((coords.v)  * height);
			
			if (color_x < 0 || color_x > width) {
				continue;

			}

			if (color_y < 0 || color_y > height) {
				continue;
			}

			rgbVec[i].r = *(colorData + color_x * 3 + color_y * width * 3 + 0);
			rgbVec[i].g = *(colorData + color_x * 3 + color_y * width * 3 + 1);
			rgbVec[i].b = *(colorData + color_x * 3 + color_y * width * 3 + 2);*/
		}
		orderVec.resize(pointCount);
		
		uint32_t nBytesXYZ = 0;

		hr = lepcc_computeCompressedSizeXYZ(ctx, pointCount, (const double*)(&ptVec[0]), maxXErr, maxYErr, maxZErr, &nBytesXYZ, (uint32_t*)(&orderVec[0]));
		if (hr)
		{
			printf("Error in main(): lepcc_computeCompressedSizeXYZ(...) failed.\n");  return;
		}

		uint32_t nBytesRGB = 0;
		if (rgbVec.size() > 0)
		{
			if (orderVec.size() > 0)    // resort the RGB values to point order
			{
				sortedRgbVec.resize(pointCount);
				for (int i = 0; i < pointCount; i++)
					sortedRgbVec[i] = rgbVec[orderVec[i]];

				hr = lepcc_computeCompressedSizeRGB(ctx, pointCount, (const lepcc::Byte*)(&sortedRgbVec[0]), &nBytesRGB);
			}
			else
				hr = lepcc_computeCompressedSizeRGB(ctx, pointCount, (const lepcc::Byte*)(&rgbVec[0]), &nBytesRGB);

			if (hr)
			{
				printf("Error in main(): lepcc_computeCompressedSizeRGB(...) failed.\n");  return;
			}
		}

		int64_t nBytes = nBytesXYZ + nBytesRGB;

		// stats
		totalNumBytesCompressed += nBytes;
		totalNumPoints += pointCount;
		totalNumTiles++;

		//encode
		byteVec.resize((size_t)nBytes);
		lepcc::Byte* buffer = &byteVec[0];
		lepcc::Byte* pByte = buffer;

		// encode
		if (ptVec.size() > 0)
			if (hr = lepcc_encodeXYZ(ctx, &pByte, nBytesXYZ))
			{
				printf("Error in main(): lepcc_encodeXYZ(...) failed.\n");  return;
			}

		if (rgbVec.size() > 0)
			if (hr = lepcc_encodeRGB(ctx, &pByte, nBytesRGB))
			{
				printf("Error in main(): lepcc_encodeRGB(...) failed.\n");  return;
			}

		cout << "number of bytes for frame: " << nBytes / 1024 << "KB\n";

		//decode
		const lepcc::Byte* pByte2 = buffer;    // same buffer

		uint32_t nPts2 = 0;

		if (!lepcc_getPointCount(ctxDec, pByte2, nBytesXYZ, &nPts2) && nPts2 > 0)
		{
			decPtVec.resize(nPts2);
			if (hr = lepcc_decodeXYZ(ctxDec, &pByte2, nBytesXYZ, &nPts2, (double*)(&decPtVec[0])))
			{
				printf("Error in main(): lepcc_decodeXYZ(...) failed.\n");  return;
			}

			// compare
			for (int i = 0; i < (int)nPts2; i++)
			{
				int k = orderVec[i];
				const lepcc::Point3D* p = &ptVec[k];
				const lepcc::Point3D* q = &decPtVec[i];

				double dx = abs(q->x - p->x);
				double dy = abs(q->y - p->y);
				double dz = abs(q->z - p->z);

				maxDecErrX = max(dx, maxDecErrX);
				maxDecErrY = max(dy, maxDecErrY);
				maxDecErrZ = max(dz, maxDecErrZ);
			}
		}

		if (!lepcc_getRGBCount(ctxDec, pByte2, nBytesRGB, &nPts2) && nPts2 > 0)
		{
			decRgbVec.resize(nPts2);
			if (hr = lepcc_decodeRGB(ctxDec, &pByte2, nBytesRGB, &nPts2, (lepcc::Byte*)(&decRgbVec[0])))
			{
				printf("Error in main(): lepcc_decodeRGB(...) failed.\n");  return;
			}

			// compare
			bool resortedColors = !orderVec.empty();

			for (int i = 0; i < (int)nPts2; i++)
			{
				int k = resortedColors ? orderVec[i] : i;
				lepcc::RGB_t rgbEnc = rgbVec[k];
				lepcc::RGB_t rgbDec = decRgbVec[i];

				double dx = rgbDec.r - rgbEnc.r;
				double dy = rgbDec.g - rgbEnc.g;
				double dz = rgbDec.b - rgbEnc.b;
				double delta = sqrt(dx * dx + dy * dy + dz * dz);

				maxDecErrRGB = max(delta, maxDecErrRGB);
				totalDecErrRGB += delta;
			}
		}


		std::vector<pcl::PointXYZRGB> pclPoints(nPts2);

		for (int i = 0; i < (int)nPts2; i++)
		{
			pclPoints[i].r = rgbVec[i].r;
			pclPoints[i].g = rgbVec[i].g;
			pclPoints[i].b = rgbVec[i].b;
			pclPoints[i].x = ptVec[i].x;
			pclPoints[i].y = ptVec[i].y;
			pclPoints[i].z = ptVec[i].z;

		}

		draw_pointcloud_rgb2(app, app_state, pclPoints);
		

		double nBytesPerPt = (double)totalNumBytesCompressed / (double)totalNumPoints;

		std::cout << "tiles  =  " << totalNumTiles << endl;
		std::cout << "points =  " << totalNumPoints << endl;
		std::cout << "max decoding error in [x, y, z] = [ " << maxDecErrX << ", " << maxDecErrY << ", " << maxDecErrZ << " ]" << endl;
		std::cout << "max decoding error in RGB = " << maxDecErrRGB << endl;
		std::cout << "mean decoding error in RGB = " << totalDecErrRGB / totalNumPoints << endl;
		std::cout << "max decoding error in intensity = " << maxDecErrIntensity << endl;
		std::cout << "mean decoding error in intensity = " << totalDecErrIntensity / totalNumPoints << endl;
		

	}

	lepcc_deleteContext(&ctx);
}

void DoPCLCompression(window &app, rs2::frameset &frames, rs2::pipeline &pipe, rs2::depth_frame &depth, rs2::points &points, rs2::pointcloud &pc, rs2::video_frame &color, state app_state)
{
	pcl_ptr pcl_points;
	//compression
	bool showStatistics = true;
	//	bool showStatistics = false;

	// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
	//pcl::io::compression_Profiles_e compressionProfileRGB = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	pcl::io::compression_Profiles_e compressionProfileRGB = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

	auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, showStatistics, 0.0002, 0.003, true, 0U);

	auto PointCloudDecoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
	
	int filtered_size = 0;
	int cloudSize = 0;
	int cumulativeDataLength = 0;


	int calc_compression_index = 0;
	int calc_compression_rate = 30;
	int compression_data_size = 0;


	int switch_buffer_counter = 0;
	int switch_buffer_rate = 5;

	int frameCounter = 0;

	SYSTEMTIME startTime;
	SYSTEMTIME endTime;
	GetLocalTime(&startTime);
	GetLocalTime(&endTime);
	//	int cumulativeDataLength = 0;

	while (app) // Application still alive?
	{
		// Wait for the next set of frames from the camera
		frames = pipe.wait_for_frames();

		depth = frames.get_depth_frame();


		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		color = frames.get_color_frame();


		// Tell pointcloud object to map to this color frame
		pc.map_to(color);

		//app_state.tex.upload(color);


		/*
		{
		auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
		//for (int i = 0; i < points.size(); i++)
		for (int i = 0; i < 100; i++)
		{
		auto coords = tex_coords[i];
		std::cout << coords.u << "," << coords.v << std::endl;
		}


		}
		*/
		//continue;


		auto pcl_rgb_points = points_to_pcl(points, color);

		//pcl_points = points_to_pcl(points);

		/*
		std::cout << "point size" << std::endl;
		std::cout << pcl_rgb_points->points.size() << std::endl;
		*/



		//pcl_rgb_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl_rgb_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(pcl_rgb_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.2, 1.0);

		pass.filter(*cloud_filtered);

		/*
		std::cout << cloud_filtered->points.size() << std::endl;
		std::cout << cloud_filtered->is_dense << std::endl;
		std::cout << cloud_filtered->size() << std::endl;
		*/
		/*
		if (filtered_size == 0) {
		filtered_size = cloud_filtered->size();
		}

		*/

		int filteredSize = cloud_filtered->size();
		if (cloudSize == 0)
			cloudSize = filteredSize;
		else
		{
			float differenceFactor = std::abs((float)(filteredSize - cloudSize) / cloudSize);
			//cout << "differenceFactor: " << differenceFactor << "\n";
			if (differenceFactor > 0.1 || (frameCounter++ % 5 == 0))
			{
				cloudSize = filteredSize;
				PointCloudEncoderRGB->switchBuffers();
				cout << "switched cloudsize, pointcloud size difference factor is " << differenceFactor << "\n";

			}
			else
			{
				cloud_filtered->resize(cloudSize);
			}
		}

		/*


		auto original_size = cloud_filtered->size();
		if (original_size > filtered_size) {
		cloud_filtered->resize(filtered_size);
		std::cout << "reduce" << std::endl;
		}
		else {
		std::cout << "add dummy data" << std::endl;

		cloud_filtered->resize(filtered_size);

		for (int i = original_size; i < filtered_size; i++)
		{
		// upload the point and texture coordinates only for points we have depth data for
		cloud_filtered->points[i].x = 0;
		cloud_filtered->points[i].y = 0;
		cloud_filtered->points[i].z = 0;
		cloud_filtered->points[i].r = 0;
		cloud_filtered->points[i].g = 0;
		cloud_filtered->points[i].b = 0;

		}
		}

		*/

		// compress 
		std::stringstream compressedData;
		// output pointcloud
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

		// compress point cloud
		//PointCloudEncoder->encodePointCloud(cloud_filteredp, compressedData);
		//PointCloudEncoder->encodePointCloud(pcl_rgb_points, compressedData);
		//PointCloudEncoderRGB->encodePointCloud(pcl_rgb_points, compressedData);

		//switchBuffer reduces noise
		/*
		if (++switch_buffer_counter >= switch_buffer_rate) {
		PointCloudEncoderRGB->switchBuffers();
		switch_buffer_counter = 0;
		}
		*/
		//PointCloudEncoderRGB->switchBuffers();
		
		PointCloudEncoderRGB->encodePointCloud(pcl_rgb_points, compressedData);
		//PointCloudEncoderRGB->encodePointCloud(cloud_filtered, compressedData);


		// calculate data size
		std::string compressedDataString = compressedData.str();

		//std::stringstream().swap(compressedData);
		int frameDataLength = compressedDataString.length() / 1024; //get the datalength in kilobytes
																	//cout << "compresseddata offset: " << frameDataLength << "\n";
		cumulativeDataLength += frameDataLength;
		GetLocalTime(&endTime);

		if ((endTime - startTime).wSecond > 0)
		{
			startTime = endTime;
			cout << "Total data transfer over last second was " << cumulativeDataLength << "KB\n";
			cumulativeDataLength = 0;
		}

		pcl_rgb_ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		PointCloudDecoderRGB->decodePointCloud(compressedData, decompressed_cloud);




		/*
		compressedData.seekp(0, ios::end);
		stringstream::pos_type offset = compressedData.tellp();
		std::cout << offset << std::endl;



		calc_compression_index++;
		if (calc_compression_index == calc_compression_rate) {
		compression_data_size += (int)offset;
		std::cout << compression_data_size / calc_compression_rate << "Bytes" << std::endl;
		compression_data_size = 0;
		calc_compression_index = 0;
		}
		*/




		//try {

		/*
		if (cloud_filtered != nullptr) {
		PointCloudEncoderRGB->encodePointCloud(cloud_filtered, compressedData);
		}
		*/



		/**/
		// draw point cloud
		std::vector<pcl_rgb_ptr> layers_rgb;
		//layers_rgb.push_back(pcl_rgb_points);
		//layers_rgb.push_back(cloud_filtered);
		layers_rgb.push_back(decompressed_cloud);

		draw_pointcloud_rgb(app, app_state, layers_rgb);



		continue;
		/*
		std::vector<pcl_rgb_ptr> layers_rgb;
		//layers_rgb.push_back(pcl_rgb_points);
		layers_rgb.push_back(cloud_filtered);

		draw_pointcloud_rgb(app, app_state, layers_rgb);

		}
		catch (exception e) {
		std::cerr << e.what() << std::endl;
		}


		continue;


		/*
		pcl_points = points_to_pcl(points);

		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(pcl_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);

		std::cout << "foo" << std::endl;

		pass.filter(*cloud_filtered);

		// compress
		std::stringstream compressedData;
		// output pointcloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

		// compress point cloud
		//PointCloudEncoder->encodePointCloud(cloud_filtered, compressedData);
		PointCloudEncoder->encodePointCloud(pcl_points, compressedData);


		//continue;

		std::vector<pcl_ptr> layers;
		layers.push_back(pcl_points);
		layers.push_back(cloud_filtered);


		draw_pointcloud(app, app_state, layers);
		*/

		continue;
	}
}


// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        app_state.offset_x += static_cast<float>(xoffset);
        app_state.offset_y += static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y)
    {
        if (app_state.ml)
        {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)
            {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}


void draw_pointcloud_rgb(window& app, state& app_state, const std::vector<pcl_rgb_ptr>& points)
{
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& pc : points)
	{
		//auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

		glBegin(GL_POINTS);
		//glColor3f(c.x, c.y, c.z);

		/* this segment actually prints the pointcloud */
		for (int i = 0; i < pc->points.size(); i++)
		{
			auto&& p = pc->points[i];
			if (p.z)
			{
				// upload the point and texture coordinates only for points we have depth data for
				glVertex3f(p.x, p.y, p.z);

				glColor3f(p.r / 255.0, p.g/ 255.0, p.b/255.0);
			}
		}

		glEnd();
	}

	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

void draw_pointcloud_rgb2(window& app, state& app_state, std::vector<pcl::PointXYZRGB> points)
{
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (int i = 0; i < points.size(); i++)
	{
		auto&& p = points[i];
		if (p.z)
		{
			// upload the point and texture coordinates only for points we have depth data for
			glVertex3f(p.x, p.y, p.z);

			glColor3f((float)p.r / 255.0, (float)p.g / 255.0, (float)p.b / 255.0);
		}
	}

	glEnd();

	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}


SYSTEMTIME operator-(const SYSTEMTIME& pSr, const SYSTEMTIME& pSl)
{
	SYSTEMTIME t_res;
	FILETIME v_ftime;
	ULARGE_INTEGER v_ui;
	__int64 v_right, v_left, v_res;
	SystemTimeToFileTime(&pSr, &v_ftime);
	v_ui.LowPart = v_ftime.dwLowDateTime;
	v_ui.HighPart = v_ftime.dwHighDateTime;
	v_right = v_ui.QuadPart;

	SystemTimeToFileTime(&pSl, &v_ftime);
	v_ui.LowPart = v_ftime.dwLowDateTime;
	v_ui.HighPart = v_ftime.dwHighDateTime;
	v_left = v_ui.QuadPart;

	v_res = v_right - v_left;

	v_ui.QuadPart = v_res;
	v_ftime.dwLowDateTime = v_ui.LowPart;
	v_ftime.dwHighDateTime = v_ui.HighPart;
	FileTimeToSystemTime(&v_ftime, &t_res);
	return t_res;
}


//d_intrinsics = depth_stream.asrs2::video_stream_profile().get_intrinsics();
//c_intrinsics = color_stream.asrs2::video_stream_profile().get_intrinsics();
//d_to_c_extrinsics = depth_stream.get_extrinsics_to(color_stream);