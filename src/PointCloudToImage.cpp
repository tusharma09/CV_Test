#include "pch.h"
//
//
//#include <iostream>
//#include <string>
//#include <chrono>
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
////#include <pcl/console/time.h>   // TicToc
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree_flann.h>
////#include <pcl/filters/statistical_outlier_removal.h>
////#include <pcl/filters/passthrough.h>
//
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//
//
//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//typedef boost::shared_ptr<PointCloudT> PointCloudPtr;
//
//bool next_iteration = false;
//
//using namespace std;
//using namespace pcl;
//using namespace cv;
//
//
//#define INRANGE(x, x1, x2) (x >= x1 && x <= x2)
//
//
//pcl::visualization::PCLVisualizer::Ptr mainViewer(new pcl::visualization::PCLVisualizer("3D Viewer"));;
//
//
//
//
///// <summary>
///// Loads the xyz and mdl file to the global variable _points; the scan of the Lorry
///// </summary>
///// <param name="fileName">Name of the file</param>
///// <returns>Length of one frame length; 0 means loading failed</returns>
//template <typename PointT>
//int LoadFile(string fileName, boost::shared_ptr<pcl::PointCloud<PointT> > return_cloud)
//{
//	///PCD file
//	if (fileName.substr(fileName.length() - 3, 3).compare("xyz"))
//	{
//		if (pcl::io::loadPCDFile<PointT>(fileName, *return_cloud) == -1) //* load the file
//		{
//			PCL_ERROR("Couldn't read file %s \n", fileName);
//			return (-1);
//		}
//	}
//	else ///XYZ file
//	{
//		std::ifstream infile(fileName);
//
//		if (infile.good())
//		{
//			string line;
//			std::getline(infile, line);
//			while (!infile.eof())
//			{
//				std::getline(infile, line);
//				std::istringstream iss(line);
//
//				PointT point;
//				int r, g, b;
//				iss >> point.x >> point.y >> point.z >> r >> g >> b;///Rest is ignored
//				point.r = r; point.g = g; point.b = b;
//				return_cloud->push_back(point);
//			}
//		}
//
//
//		infile.close();
//		return_cloud->width = return_cloud->points.size();
//		return_cloud->height = 1;
//	}
//	return return_cloud->width * return_cloud->height;
//}
//
//
/////brief	Retruns whether the given file name exists
/////param	[IN] name: Name of the file to check.. can be full path with file name
/////author	Tushar Sharma
//inline bool DoesFileExist(const std::string& name)
//{
//	struct stat buffer;
//	return (stat(name.c_str(), &buffer) == 0);
//}
//
//
/////brief	Saves the point cloud points in the given filename
/////param	[IN] fileName: input source cloud points
/////param	[IN] cloudPoints: points in the given template format
/////author	Tushar Sharma
//template <typename PointLocalT>
//int SaveXYZPointCloud(string fileName, const std::vector<PointLocalT, Eigen::aligned_allocator<PointLocalT>> &cloudPoints, int r = 255, int g = 255, int b = 255)
//{
//	///Remove file if exists
//	if (DoesFileExist(fileName))
//	{
//		remove(fileName.c_str());
//	}
//
//
//	ofstream file;
//	file.open(fileName.c_str());
//
//	file << "x\t y\t z\t r\t g\t b \n";
//
//	if (!file.is_open())
//		return -1;
//
//	for (PointLocalT point : cloudPoints)
//	{
//		string line = to_string(point.x) + "\t" + to_string(point.y) + "\t" + to_string(point.z) + "\t" + to_string(point.r) + "\t" + to_string(point.g) + "\t" + to_string(point.b)
//
//			//+ HasRGB<PointLocalT> ? "\t" + to_string(point.r) + "\t" + to_string(point.g) + "\t" + to_string(point.b) : ""
//
//			+"\n";
//
//		file << line;
//	}
//	file.close();
//
//	return 0;
//}
//
//
//
//void print4x4Matrix(const Eigen::Matrix4d & matrix)
//{
//	printf("Rotation matrix :\n");
//	printf("    | %6.5f %6.5f %6.5f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
//	printf("R = | %6.5f %6.5f %6.5f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
//	printf("    | %6.5f %6.5f %6.5f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
//	printf("Translation vector :\n");
//	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
//}
//
//
//
//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
//{
//	//if (event.getKeySym() == "space" && event.keyDown())
//	//	next_iteration = true;
//	//else if (event.getKeySym() == "t" && event.keyDown())
//	//	icpDone = true;
//}
//
//
//
//void ShowPointCloud(PointCloudPtr cloud, string name = "")
//{
//	mainViewer->addPointCloud<PointT>(cloud, name);
//	mainViewer->spin();
//	mainViewer->removeAllPointClouds();
//	mainViewer->spinOnce();
//}
//
//
//
//inline float DotProduct(vector<float> v1, vector<float> v2)
//{
//	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
//}
//
//
//
//inline vector<float> VectorSubtract3D(vector<float> &v1, vector<float> &v2)
//{
//	vector<float> v3(3);
//	v3[0] = v1[0] - v2[0];
//	v3[1] = v1[1] - v2[1];
//	v3[2] = v1[2] - v2[2];
//	return v3;
//}
//
//
////ORIGINAL
//cv::Mat ToImage1(PointCloudPtr cloud, uint zPlane = 300, uint imgWidth = 1920, uint imgHeight = 1080)
//{
//	cv::Mat image(imgHeight, imgWidth, CV_8UC3, cv::Scalar(0, 0, 0));
//
//	PointCloudT::Ptr imgCloud(new PointCloudT);///tgt
//
//	imgCloud->resize(cloud->size());
//	vector<float> Pp(3, 0), Pn(3, 0);
//	Pp[2] = zPlane; Pn[2] = 1;
//	vector<float> Rp(3, 0), Rv(3);
//
//	//cloud1->points[0].x = 150;
//	//cloud1->points[0].y = 200;
//	//cloud1->points[0].z = 400;
//
//
//	for (size_t i = 0; i < cloud->size(); i++)
//	{
//		Rp[0] = cloud->points[i].x;
//		Rp[1] = cloud->points[i].y;
//		Rp[2] = cloud->points[i].z;
//
//		float mag = sqrt(pow(Rp[0], 2) + pow(Rp[1], 2) + pow(Rp[2], 2));
//
//		Rv[0] = Rp[0] / mag;
//		Rv[1] = Rp[1] / mag;
//		Rv[2] = Rp[2] / mag;
//
//		///http://www.ambrsoft.com/TrigoCalc/Plan3D/PlaneLineIntersection_.htm
//		float t = (Pp[2]) / Rv[2];
//
//		imgCloud->points[i].x = Rv[0] * t;
//		imgCloud->points[i].y = Rv[1] * t;
//		imgCloud->points[i].z = Rv[2] * t;
//		imgCloud->points[i].r = cloud->points[i].r;
//		imgCloud->points[i].g = cloud->points[i].g;
//		imgCloud->points[i].b = cloud->points[i].b;
//	}
//
//	ShowPointCloud(imgCloud, "sdfcsdf");
//
//	//SaveXYZPointCloud("imgCloud.xyz", cloud2->points);
//
//
//	int cWidth = 100 * imgWidth / imgHeight, cHeight = 100;
//
//	int minX = -cWidth / 2, maxX = cWidth / 2;
//	int minY = -cHeight / 2, maxY = cHeight / 2;
//
//
//	for_each(imgCloud->points.begin(), imgCloud->points.end(), [minX, minY, cWidth, cHeight, imgWidth, imgHeight](PointT &point)
//	{
//		point.x = imgWidth - (int)((((float)(point.x - minX)) / (float)cWidth) * imgWidth);
//		point.y = imgHeight - (int)((((float)(point.y - minY)) / (float)cHeight) * imgHeight);
//	});
//
//
//	for (size_t i = 0; i < imgCloud->size(); i++)
//	{
//		if (INRANGE(imgCloud->points[i].x, 0, imgWidth - 1) && INRANGE(imgCloud->points[i].y, 0, imgHeight - 1))
//		{
//			image.at<Vec3b>(Point(imgCloud->points[i].x, imgCloud->points[i].y)) = Vec3b(imgCloud->points[i].b, imgCloud->points[i].g, imgCloud->points[i].r);
//		}
//	}
//	return image;
//}
//
//using namespace Eigen;
//struct Plane
//{
//	Vector3f point;
//	Vector3f normal;
//	Plane(Vector3f p, Vector3f n) : point(p), normal(n)
//	{
//
//	}
//	Plane()
//	{
//	}
//};
//Vector3f GetIntersectionPointVectorAndPlane(Vector3f rayV, Vector3f rayP, Vector3f planeN, Vector3f planeP)
//{
//	float d = planeP.dot(-planeN);
//	float t = -(d + rayP.z() * planeN.z() + rayP.y() * planeN.y() + rayP.x() * planeN.x()) / (rayV.z() * planeN.z() + rayV.y() * planeN.y() + rayV.x() * planeN.x());
//	return rayP + t * rayV;
//}
//Matrix3f AlignToVector(Vector3f vecFrom, Vector3f vecTo)
//{
//	vecFrom.normalize();
//	vecTo.normalize();
//	Vector3f cross = vecFrom.cross(vecTo);
//	cross.normalize();
//	float angle = acos(vecFrom.dot(vecTo));
//	AngleAxisf aa(angle, cross);
//	return aa.toRotationMatrix();
//}
//cv::Mat ToImage(PointCloudPtr cloud, Vector3f camPosition, Plane camPlane, Matrix4f toWorld, uint imgWidth = 640, uint imgHeight = 480)
//{
//	cv::Mat image(imgHeight, imgWidth, CV_8UC3, cv::Scalar(0, 0, 0));
//
//	PointCloudT::Ptr imgCloud(new PointCloudT);///tgt
//	imgCloud->resize(cloud->size());
//
//	Vector3f rayPoint, rayVector;
//
//	//cloud1->points[0].x = 150;
//	//cloud1->points[0].y = 200;
//	//cloud1->points[0].z = 400;
//
//
//	for (size_t i = 0; i < cloud->size(); i++)
//	{
//		rayPoint[0] = cloud->points[i].x;
//		rayPoint[1] = cloud->points[i].y;
//		rayPoint[2] = cloud->points[i].z;
//		rayVector = rayPoint - camPosition;
//
//		rayVector.normalize();
//
//		Vector3f temp = GetIntersectionPointVectorAndPlane(rayVector, rayPoint, camPlane.normal, camPlane.point);
//
//		imgCloud->points[i].x = temp.x();
//		imgCloud->points[i].y = temp.y();
//		imgCloud->points[i].z = temp.z();
//		imgCloud->points[i].r = cloud->points[i].r;
//		imgCloud->points[i].g = cloud->points[i].g;
//		imgCloud->points[i].b = cloud->points[i].b;
//	}
//
//	cout << "imgCloud" << endl;
//	ShowPointCloud(imgCloud, "sdfcsdf");
//
//	//SaveXYZPointCloud("imgCloud.xyz", cloud2->points);
//
//	Matrix4f trans = Matrix4f::Identity();
//	//trans(0, 3) = -camPlane.point.x();
//	//trans(1, 3) = -camPlane.point.y();
//	//trans(2, 3) = -camPlane.point.z();
//	//pcl::transformPointCloud(*imgCloud, *imgCloud, trans);
//	//ShowPointCloud(imgCloud, "sdfcsdf");
//
//	//trans = Matrix4f::Identity();
//	//if (camPlane.normal.z() < 0)
//	//{
//	//	trans.block(0, 0, 3, 3) = AlignToVector(camPlane.normal, Vector3f(0, 0, -1));
//	//}
//	//else
//	//{
//	//	trans.block(0, 0, 3, 3) = AlignToVector(camPlane.normal, Vector3f(0, 0, 1));
//	//}
//	////trans(0, 3) = -camPosition.x();
//	////trans(1, 3) = -camPosition.y();
//	////trans(2, 3) = -camPosition.z();
//	//pcl::transformPointCloud(*imgCloud, *imgCloud, trans);
//	////camPosition = trans.block(0, 0, 3, 3) * camPosition;
//	////trans = Matrix4f::Identity();
//	////trans(0, 3) = -camPosition.x();
//	////trans(1, 3) = -camPosition.y();
//	////trans(2, 3) = -camPosition.z();
//	////pcl::transformPointCloud(*imgCloud, *imgCloud, trans);
//	//ShowPointCloud(imgCloud, "sdfcsdf");
//
//
//	pcl::transformPointCloud(*imgCloud, *imgCloud, toWorld.inverse());
//
//	ShowPointCloud(imgCloud, "sdfcsdf");
//	int cWidth = 100 * imgWidth / imgHeight, cHeight = 100;
//
//	int minX = -cWidth / 2, maxX = cWidth / 2;
//	int minY = -cHeight / 2, maxY = cHeight / 2;
//
//
//	for_each(imgCloud->points.begin(), imgCloud->points.end(), [minX, minY, cWidth, cHeight, imgWidth, imgHeight](PointT &point)
//	{
//		point.x = imgWidth - (int)((((float)(point.x - minX)) / (float)cWidth) * imgWidth);
//		point.y = imgHeight - (int)((((float)(point.y - minY)) / (float)cHeight) * imgHeight);
//	});
//
//
//	for (size_t i = 0; i < imgCloud->size(); i++)
//	{
//		if (INRANGE(imgCloud->points[i].x, 0, imgWidth - 1) && INRANGE(imgCloud->points[i].y, 0, imgHeight - 1))
//		{
//			image.at<Vec3b>(Point(imgCloud->points[i].x, imgCloud->points[i].y)) = Vec3b(imgCloud->points[i].b, imgCloud->points[i].g, imgCloud->points[i].r);
//		}
//	}
//	return image;
//}
//
//
//Mat BlackWhite_To_RedGreen(Mat& I)
//{
//	struct RGB {
//		uchar blue;
//		uchar green;
//		uchar red;
//	};
//	// accept only char type matrices
//	CV_Assert(I.depth() == CV_8U);
//
//	int channels = I.channels();
//
//	int nRows = I.rows;
//	int nCols = I.cols * channels;
//
//	//if (I.isContinuous())
//	//{
//	//	nCols *= nRows;
//	//	nRows = 1;
//	//}
//
//	int i, j;
//	uchar* p;
//	for (i = 0; i < nRows; ++i)
//	{
//		p = I.ptr<uchar>(i);
//		for (j = 0; j < nCols; j += channels)
//		{
//			uchar* color = &(p[j]);
//
//			//cout << to_string(color[0]) << "  " << to_string(color[1]) << to_string(color[2]) << endl;
//
//			if ((color[0] + color[1] + color[2]) > 150)
//			{
//				color[0] = 0; color[1] = 0; color[2] = 255;
//			}
//			else
//			{
//				color[0] = 0; color[1] = 255; color[2] = 0;
//			}
//
//		}
//	}
//	return I;
//}
//
//
//
//int main(int argc, char* argv[])
//{
//	//Mat image = imread("image.png");
//	//image = BlackWhite_To_RedGreen(image);
//	//imwrite("cb.png", image);
//	//return 0;
//
//	// The point clouds we will be using
//	PointCloudT::Ptr cloud1(new PointCloudT);///src
//
//	std::cout << "Loading Files" << endl;
//
//	LoadFile("test.xyz", cloud1);
//	//LoadFile(argv[2], cloud_original2);
//	std::cout << "Loaded: " << cloud1->size() << " points" << endl;
//
//
//	uint imgWidth = 640, imgHeight = 480;
//
//
//	//PointT p(1, 1, 1);
//	//p.x = 0; p.y = 0; p.z = 1000;
//	//mainViewer->addLine<pcl::PointXYZRGB>(PointT(1, 1, 1), p, 1, 0, 0, "line");
//	////mainViewer->addSphere<pcl::PointXYZRGB>(PointT(1, 1, 1), 10, 1, 0, 0, "origin");
//	//ShowPointCloud(cloud1);
//	//Mat image = ToImage(cloud1, 300, imgWidth, imgHeight), smallImage, bigImage;
//
//	Mat image;
//	{
//		//Vector3f camPosition, camVector;
//		float focalLength = 50;
//
//
//		//camPosition = Vector3f(100, 0, 0);
//		//camVector = Vector3f(-18, -3, 500) - camPosition;
//		//camVector.normalize();
//
//		AngleAxisf aa((-18.0/180.0*M_PI), Vector3f(0,1,0));
//		Vector4f vec(0, 0, 1, 0), origin(0,0,0,1), planeP(0, 0, 50, 1);
//		Matrix4f tra = Matrix4f::Identity();
//		tra(0, 3) = 50;
//		tra.block(0,0,3,3) = aa.toRotationMatrix();
//		origin = tra * origin;
//		planeP = tra * planeP;
//		vec = tra * vec;
//		vec.normalize();
//
//
//		Plane camPlane(Vector3f(planeP.x(), planeP.y(), planeP.z()), Vector3f(vec.x(), vec.y(), vec.z()));
//		//{
//		//	Vector3f temp;
//		//	temp[0] = camPosition.x() + focalLength * camVector.x();
//		//	temp[1] = camPosition.y() + focalLength * camVector.y();
//		//	temp[2] = camPosition.z() + focalLength * camVector.z();
//		//	camPlane = Plane(temp, camVector);
//		//}
//
//		PointT p(1, 1, 1), p1(0, 0, 0);
//		p.x = origin.x(); p.y = origin.y(); p.z = origin.z();
//		p1.x = origin.x() + 500*vec.x(); p1.y = origin.x() + 500 * vec.y(); p1.z = origin.x() + 500 * vec.z();
//		mainViewer->addLine<pcl::PointXYZRGB>(p, p1, 255, 0, 0, "line");
//		//mainViewer->addSphere<pcl::PointXYZRGB>(PointT(1, 1, 1), 10, 1, 0, 0, "origin");
//		mainViewer->addCoordinateSystem(50);
//		ShowPointCloud(cloud1);
//		cout << "To image" << endl;
//
//
//		image = ToImage(cloud1, Vector3f(origin.x(), origin.y(), origin.z()), camPlane, tra);
//	}
//
//	string windowName = "Cloud2Image";
//
//	namedWindow(windowName, WINDOW_NORMAL);
//	resizeWindow(windowName, 360 * imgWidth / imgHeight, 360);
//
//	imshow(windowName, image);
//	waitKey();
//
//
//	//cv::resize(image, smallImage, cv::Size(), 0.25, 0.25);
//	//cv::resize(smallImage, bigImage, cv::Size(), 4, 4);
//
//	//imshow(windowName, smallImage);
//	//waitKey();
//	//imshow(windowName, bigImage);
//	//waitKey();
//
//	imwrite("CloudImage.png", image);
//	return (0);
//}
//
//
//
