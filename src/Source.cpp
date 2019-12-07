#include "pch.h"
//
//#include <fstream>
//#include <iostream>
//#include <string>
//
//
//
//#define PointTypeLocal pcl::PointXYZRGB
//
//
//using namespace std;
//using namespace Eigen;
//
//
//
//
//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//typedef boost::shared_ptr<PointCloudT> PointCloudPtr;
//
//
//
//pcl::visualization::PCLVisualizer::Ptr mainViewer(new pcl::visualization::PCLVisualizer("3D Viewer"));;
//
//
//
//double Round(double x, int p)
//{
//	if (x != 0.0) {
//		return ((floor((fabs(x)*pow(double(10.0), p)) + 0.5)) / pow(double(10.0), p))*(x / fabs(x));
//	}
//	else {
//		return 0.0;
//	}
//}
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
//
//
//void SaveMatrix(string file, MatrixXd mat)
//{
//	ofstream ofile(file);
//	for (size_t i = 0; i < mat.rows(); i++)
//	{
//		for (size_t j = 0; j < mat.cols(); j++)
//		{
//			ofile << mat(i, j) << "\t";
//		}
//		ofile << endl;
//	}
//	ofile.close();
//}
//
//
//
//
//Matrix4d GetTransformationMatrixFromFile(std::string fileName)
//{
//	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
//
//	std::ifstream f(fileName);
//	if (!f)
//	{
//		std::cout << "Cannot open file: " + fileName << std::endl;
//	}
//
//	float temp = 0;
//	for (size_t i = 0; i < 4; i++)
//	{
//		for (size_t j = 0; j < 4; j++)
//		{
//			f >> temp;
//			matrix(i, j) = Round(temp, 6);
//		}
//	}
//
//	f.close();
//
//
//	//std::cout << "robot pose:" << endl << matrix << endl;
//
//	return matrix;
//}
//
//
//
//
//void GetOXYPointsFromFile(string fileName, PointTypeLocal &O, PointTypeLocal &X, PointTypeLocal &Y)
//{
//	std::ifstream f(fileName);
//	if (!f)
//	{
//		std::cout << "Cannot open file: " + fileName << std::endl;
//	}
//
//	f >> O.x >> O.y >> O.z;
//	f >> X.x >> X.y >> X.z;
//	f >> Y.x >> Y.y >> Y.z;
//
//	f.close();
//}
//
//
//
//
//void GetClustersByRegionGrowing(boost::shared_ptr<pcl::PointCloud<PointTypeLocal> > cloudScene, size_t minSize, vector<pcl::PointCloud<PointTypeLocal>::Ptr> &clusterClouds)
//{
//	clusterClouds.clear();
//
//	pcl::search::KdTree<PointTypeLocal>::Ptr tree(new pcl::search::KdTree<PointTypeLocal>);
//
//
//	std::vector<pcl::PointIndices> clusterIndices;
//	pcl::EuclideanClusterExtraction<PointTypeLocal> ec;
//	ec.setClusterTolerance(2); // 2cm
//	ec.setMinClusterSize(minSize);
//	ec.setMaxClusterSize(10000);
//	ec.setSearchMethod(tree);
//	ec.setInputCloud(cloudScene);
//	ec.extract(clusterIndices);
//
//	//vector < pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <PointT>::Ptr > > clusterClouds;
//	for (int i = 0; i < clusterIndices.size(); i++)
//	{
//		pcl::PointCloud<PointTypeLocal>::Ptr monoCluster(new pcl::PointCloud<PointTypeLocal>);
//		for (int j = 0; j < clusterIndices.at(i).indices.size(); j++)
//		{
//			monoCluster->push_back(cloudScene->at(clusterIndices.at(i).indices.at(j)));
//		}
//		clusterClouds.push_back(monoCluster);
//
//		//pcl::io::savePCDFileBinary("Cluster"+ std::to_string(i) + ".pcd", *mono_cluster);
//	}
//}
//
//
//
//
//std::pair<PointTypeLocal, PointTypeLocal> BestLineFromPoints(const vector <PointTypeLocal> &points)
//{
//	std::vector<Eigen::Vector3f> c(points.size());
//
//	for (size_t i = 0; i < points.size(); i++)
//	{
//		c[i] = Eigen::Vector3f(points[i].x, points[i].y, points[i].z);
//	}
//
//
//	// copy coordinates to  matrix in Eigen format
//	size_t numAtoms = c.size();
//	Eigen::Matrix< Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > centers(numAtoms, 3);
//	for (size_t i = 0; i < numAtoms; ++i) centers.row(i) = c[i];
//
//	Eigen::Vector3f origin = centers.colwise().mean();
//	Eigen::MatrixXf centered = centers.rowwise() - origin.transpose();
//	Eigen::MatrixXf cov = centered.adjoint() * centered;
//	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);
//	Vector3f axis = eig.eigenvectors().col(2).normalized();
//
//	PointTypeLocal ori; ori.x = origin[0]; ori.y = origin[1]; ori.z = origin[2];
//	PointTypeLocal axi; axi.x = axis[0]; axi.y = axis[1]; axi.z = axis[2];
//	return std::make_pair(ori, axi);
//}
//
//
//
//
//PointTypeLocal GetIntersection(PointTypeLocal p1, PointTypeLocal v1, PointTypeLocal p2, PointTypeLocal v2)
//{
//	/// x1 + a1t1 = x2 + a2t2
//	/// y1 + b1t1 = y2 + b2t2
//	/// z1 + c1t1 = z2 + c2t2
//	/// solve for t1 and t2
//
//	float t1 = ((p1.y - p2.y)*v2.x - (p1.x - p2.x)*v2.y) / (v1.x*v2.y - v1.y*v2.x);
//	float t2 = ((p1.y - p2.y)*v1.x - (p1.x - p2.x)*v1.y) / (v1.x*v2.y - v1.y*v2.x);
//
//	PointTypeLocal intersection;
//	intersection.x = p1.x + v1.x*t1; intersection.y = p1.y + v1.y*t1; intersection.z = p1.z + v1.z*t1;
//
//
//	intersection.x = p2.x + v2.x*t2; intersection.y = p2.y + v2.y*t2; intersection.z = p2.z + v2.z*t2;
//
//	return intersection;
//}
//
//
//
//
//void InistialiseViewer()
//{
//	std::pair<unsigned int, unsigned int> viewerWindowPosition;
//	std::pair<unsigned int, unsigned int> viewerWindowSize;
//	viewerWindowSize = std::make_pair<unsigned int, unsigned int>(800, 650);
//	viewerWindowPosition = std::make_pair<unsigned int, unsigned int>(500, 25);
//
//	mainViewer->setPosition(viewerWindowPosition.first, viewerWindowPosition.second);
//	mainViewer->setSize(viewerWindowSize.first, viewerWindowSize.second);
//
//	pcl::PointCloud<PointTypeLocal>::Ptr cloud(new pcl::PointCloud<PointTypeLocal>);
//	cloud->points.push_back(PointTypeLocal(0, 0, 0));
//	mainViewer->addPointCloud(cloud);
//	mainViewer->spinOnce();
//}
//
//
//
//
//void Something(string filePrefix, int numberOfPoses)
//{
//	MatrixXd mat(numberOfPoses, 3);
//	for (size_t i = 0; i < numberOfPoses; i++)
//	{
//		Matrix4d t2w = GetTransformationMatrixFromFile("Calibration_Poses/" + filePrefix + to_string(i + 1) + ".coords");
//		mat.block(i, 0, 1, 3) = t2w.block(0, 3, 3, 1).transpose();
//	}
//
//	SaveMatrix("points.txt", mat);
//
//	cout << "Here is the matrix m:" << endl << mat << endl;
//
//	JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
//	cout << "Its singular values are:" << endl << svd.singularValues() << endl;
//	cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
//	cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
//
//}
//
//
//
//int main(int argc, char* argv[])
//{
//	InistialiseViewer();
//
//	int numberOfPoses = 5;
//	std::vector<double, std::allocator<double>> tcp(16);
//	MatrixXd s2t;
//
//
//
//
//	cout << "done" << endl;
//	mainViewer->spin();
//	//getchar();
//}
