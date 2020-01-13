#include "pch.h"

#include <fstream>
#include <iostream>
#include <string>



#define PointTypeLocal pcl::PointXYZRGB
#define INRANGE(x, x1, x2) (x >= x1 && x <= x2)

using namespace std;
using namespace Eigen;
using namespace cv;



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudPtr;


const float resolutionForCube = 1; ///1 mm

pcl::visualization::PCLVisualizer::Ptr mainViewer(new pcl::visualization::PCLVisualizer("3D Viewer"));;
string leftWindowName = "Left image";
string rightWindowName = "Right image";
string centerWindowName = "Center image";


double Round(double x, int p)
{
	if (x != 0.0) {
		return ((floor((fabs(x)*pow(double(10.0), p)) + 0.5)) / pow(double(10.0), p))*(x / fabs(x));
	}
	else {
		return 0.0;
	}
}


#pragma region backup

/// <summary>
/// Loads the xyz and mdl file to the global variable _points; the scan of the Lorry
/// </summary>
/// <param name="fileName">Name of the file</param>
/// <returns>Length of one frame length; 0 means loading failed</returns>
template <typename PointT>
int LoadFile(string fileName, boost::shared_ptr<pcl::PointCloud<PointT> > return_cloud)
{
	///PCD file
	if (fileName.substr(fileName.length() - 3, 3).compare("xyz"))
	{
		if (pcl::io::loadPCDFile<PointT>(fileName, *return_cloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file %s \n", fileName);
			return (-1);
		}
	}
	else ///XYZ file
	{
		std::ifstream infile(fileName);

		if (infile.good())
		{
			string line;
			std::getline(infile, line);
			while (!infile.eof())
			{
				std::getline(infile, line);
				std::istringstream iss(line);

				PointT point;
				int r, g, b;
				iss >> point.x >> point.y >> point.z >> r >> g >> b;///Rest is ignored
				point.r = r; point.g = g; point.b = b;
				return_cloud->push_back(point);
			}
		}


		infile.close();
		return_cloud->width = return_cloud->points.size();
		return_cloud->height = 1;
	}
	return return_cloud->width * return_cloud->height;
}

void SaveMatrix(string file, MatrixXd mat)
{
	ofstream ofile(file);
	for (size_t i = 0; i < mat.rows(); i++)
	{
		for (size_t j = 0; j < mat.cols(); j++)
		{
			ofile << mat(i, j) << "\t";
		}
		ofile << endl;
	}
	ofile.close();
}

Matrix4d GetTransformationMatrixFromFile(std::string fileName)
{
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

	std::ifstream f(fileName);
	if (!f)
	{
		std::cout << "Cannot open file: " + fileName << std::endl;
	}

	float temp = 0;
	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 4; j++)
		{
			f >> temp;
			matrix(i, j) = Round(temp, 6);
		}
	}

	f.close();


	//std::cout << "robot pose:" << endl << matrix << endl;

	return matrix;
}

void GetOXYPointsFromFile(string fileName, PointTypeLocal &O, PointTypeLocal &X, PointTypeLocal &Y)
{
	std::ifstream f(fileName);
	if (!f)
	{
		std::cout << "Cannot open file: " + fileName << std::endl;
	}

	f >> O.x >> O.y >> O.z;
	f >> X.x >> X.y >> X.z;
	f >> Y.x >> Y.y >> Y.z;

	f.close();
}

void GetClustersByRegionGrowing(boost::shared_ptr<pcl::PointCloud<PointTypeLocal> > cloudScene, size_t minSize, vector<pcl::PointCloud<PointTypeLocal>::Ptr> &clusterClouds)
{
	clusterClouds.clear();

	pcl::search::KdTree<PointTypeLocal>::Ptr tree(new pcl::search::KdTree<PointTypeLocal>);


	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<PointTypeLocal> ec;
	ec.setClusterTolerance(2); // 2cm
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(10000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudScene);
	ec.extract(clusterIndices);

	//vector < pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <PointT>::Ptr > > clusterClouds;
	for (int i = 0; i < clusterIndices.size(); i++)
	{
		pcl::PointCloud<PointTypeLocal>::Ptr monoCluster(new pcl::PointCloud<PointTypeLocal>);
		for (int j = 0; j < clusterIndices.at(i).indices.size(); j++)
		{
			monoCluster->push_back(cloudScene->at(clusterIndices.at(i).indices.at(j)));
		}
		clusterClouds.push_back(monoCluster);

		//pcl::io::savePCDFileBinary("Cluster"+ std::to_string(i) + ".pcd", *mono_cluster);
	}
}

std::pair<PointTypeLocal, PointTypeLocal> BestLineFromPoints(const vector <PointTypeLocal> &points)
{
	std::vector<Eigen::Vector3f> c(points.size());

	for (size_t i = 0; i < points.size(); i++)
	{
		c[i] = Eigen::Vector3f(points[i].x, points[i].y, points[i].z);
	}


	// copy coordinates to  matrix in Eigen format
	size_t numAtoms = c.size();
	Eigen::Matrix< Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > centers(numAtoms, 3);
	for (size_t i = 0; i < numAtoms; ++i) centers.row(i) = c[i];

	Eigen::Vector3f origin = centers.colwise().mean();
	Eigen::MatrixXf centered = centers.rowwise() - origin.transpose();
	Eigen::MatrixXf cov = centered.adjoint() * centered;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);
	Vector3f axis = eig.eigenvectors().col(2).normalized();

	PointTypeLocal ori; ori.x = origin[0]; ori.y = origin[1]; ori.z = origin[2];
	PointTypeLocal axi; axi.x = axis[0]; axi.y = axis[1]; axi.z = axis[2];
	return std::make_pair(ori, axi);
}

PointTypeLocal GetIntersection(PointTypeLocal p1, PointTypeLocal v1, PointTypeLocal p2, PointTypeLocal v2)
{
	/// x1 + a1t1 = x2 + a2t2
	/// y1 + b1t1 = y2 + b2t2
	/// z1 + c1t1 = z2 + c2t2
	/// solve for t1 and t2

	float t1 = ((p1.y - p2.y)*v2.x - (p1.x - p2.x)*v2.y) / (v1.x*v2.y - v1.y*v2.x);
	float t2 = ((p1.y - p2.y)*v1.x - (p1.x - p2.x)*v1.y) / (v1.x*v2.y - v1.y*v2.x);

	PointTypeLocal intersection;
	intersection.x = p1.x + v1.x*t1; intersection.y = p1.y + v1.y*t1; intersection.z = p1.z + v1.z*t1;


	intersection.x = p2.x + v2.x*t2; intersection.y = p2.y + v2.y*t2; intersection.z = p2.z + v2.z*t2;

	return intersection;
}

void Jacobian(string filePrefix, int numberOfPoses)
{
	MatrixXd mat(numberOfPoses, 3);
	for (size_t i = 0; i < numberOfPoses; i++)
	{
		Matrix4d t2w = GetTransformationMatrixFromFile("Calibration_Poses/" + filePrefix + to_string(i + 1) + ".coords");
		mat.block(i, 0, 1, 3) = t2w.block(0, 3, 3, 1).transpose();
	}

	SaveMatrix("points.txt", mat);

	cout << "Here is the matrix m:" << endl << mat << endl;

	JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
	cout << "Its singular values are:" << endl << svd.singularValues() << endl;
	cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
	cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

}

#pragma endregion 


void InitialiseViewer()
{
	std::pair<unsigned int, unsigned int> viewerWindowPosition;
	std::pair<unsigned int, unsigned int> viewerWindowSize;
	viewerWindowSize = std::make_pair<unsigned int, unsigned int>(800, 650);
	//viewerWindowPosition = std::make_pair<unsigned int, unsigned int>(500, 25);
	viewerWindowPosition = std::make_pair<unsigned int, unsigned int>(550, 50);

	mainViewer->setPosition(viewerWindowPosition.first, viewerWindowPosition.second);
	mainViewer->setSize(viewerWindowSize.first, viewerWindowSize.second);

	pcl::PointCloud<PointTypeLocal>::Ptr cloud(new pcl::PointCloud<PointTypeLocal>);
	cloud->points.push_back(PointTypeLocal(0, 0, 0));
	mainViewer->addPointCloud(cloud);
	mainViewer->spinOnce();

	uint imgWidth = 640, imgHeight = 480;
	namedWindow(leftWindowName, WINDOW_NORMAL);
	namedWindow(rightWindowName, WINDOW_NORMAL);
	namedWindow(centerWindowName, WINDOW_NORMAL);
	resizeWindow(leftWindowName, 360 * imgWidth / imgHeight, 360);
	resizeWindow(rightWindowName, 360 * imgWidth / imgHeight, 360);
	resizeWindow(centerWindowName, 360 * imgWidth / imgHeight, 360);
}


void ShowPointCloud(PointCloudPtr cloud, string name = "")
{
	mainViewer->addPointCloud<PointT>(cloud, name);
	mainViewer->spin();
	mainViewer->removeAllPointClouds();
	mainViewer->spinOnce();
}

void AddPointCloud(PointCloudPtr cloud, string name = "")
{
	mainViewer->addPointCloud<PointT>(cloud, name);
	mainViewer->spin();
}

inline void AddCube(PointT center, string name = "", float length = 1, int r = 255, int g = 255, int b = 255)
{
	mainViewer->addCube((center.x - length / 2), (center.x + length / 2), (center.y - length / 2), (center.y + length / 2), (center.z - length / 2), (center.z + length / 2), (double)r, (double)g, (double)b, name);
	//mainViewer->spin();
	//mainViewer->removeAllShapes();
}

void ClearViewer()
{
	mainViewer->removeAllPointClouds();
	mainViewer->spinOnce();
}

inline Vector3f PointT_To_Vector3f(PointT point)
{
	return Vector3f(point.x, point.y, point.z);
}

inline PointT Vector3f_To_PointT(Vector3f p)
{
	PointT point(255, 255, 255);
	point.x = p[0]; point.y = p[1]; point.z = p[2];
	return point;
}

inline float ToRadian(float degree)
{
	return degree * M_PI / 180;
}

struct Plane
{
	Vector3f point;
	Vector3f normal;
	Plane(Vector3f p, Vector3f n) : point(p), normal(n)
	{

	}
	Plane()
	{
	}
};
struct Cube
{
	PointCloudPtr cloud;
	vector<Plane> cubeFaces;
	Matrix4f toWorld;
};
struct Device
{
	Vector3f position;
	Vector3f normalVectorFromSource;
	float vFOV;
	float hFOV;
	int vResolution;
	int hResolution;
	vector<Vector3f> fovCornersOriginal;
	PointCloudPtr planeCloud;
	float focalLength;
	Plane screenPlane;
};
struct Emitter : Device
{
	vector<vector <Vector3f>> rays;
	PointCloudPtr screen;
	Matrix4f toCamera;
};
struct Camera :Device
{
	Matrix3f intrinsic;
	Matrix4f toWorld;
};



Plane FitPlane(Vector3f p1, Vector3f p2, Vector3f p3)
{
	Vector3f normal = (p1 - p2).cross(p3 - p2);
	normal.normalize();
	//float d = (-a * p1.x() - b * p1.y() - c * p1.z());
	Vector3f center((p1.x() + p2.x() + p3.x()) / 3, (p1.y() + p2.y() + p3.y()) / 3, (p1.z() + p2.z() + p3.z()) / 3);

	return	Plane(center, normal);
}

Vector3f GetIntersectionPointVectorAndPlane(Vector3f rayV, Vector3f rayP, Vector3f planeN, Vector3f planeP)
{
	float d = planeP.dot(-planeN);
	float t = -(d + rayP.z() * planeN.z() + rayP.y() * planeN.y() + rayP.x() * planeN.x()) / (rayV.z() * planeN.z() + rayV.y() * planeN.y() + rayV.x() * planeN.x());
	return rayP + t * rayV;
}

Vector3f GetIntersectionPointVectorAndPlane1(Vector3f rayVector, Vector3f rayPoint, Vector3f planeNormal, Vector3f planePoint)
{
	Vector3f diff = rayPoint - planePoint;
	double prod1 = diff.dot(planeNormal);
	double prod2 = rayVector.dot(planeNormal);
	double prod3 = prod1 / prod2;
	return rayPoint - rayVector * prod3;
}

inline bool IsPointOnPlane(Vector3f point, Vector3f planeNormal, Vector3f planePoint, float epsilon = 1E-10)
{
	//float asd =(planePoint - point).dot(planeNormal);
	float d = (planeNormal.x()*planePoint.x() + planeNormal.y()*planePoint.y() + planeNormal.z()*planePoint.z());
	float ax_by_cz = -(planeNormal.x()*point.x() + planeNormal.y()*point.y() + planeNormal.z()*point.z());
	return (abs(ax_by_cz + d) < epsilon);
}

inline float DistanceToPlane(Vector3f point, Vector3f planeNormal, Vector3f planePoint)
{
	float d = -(planeNormal.x()*planePoint.x() + planeNormal.y()*planePoint.y() + planeNormal.z()*planePoint.z());
	float distance = (fabs((planeNormal.x() * point.x() + planeNormal.y() * point.y() +
		planeNormal.z() * point.z() + d)))
		/ sqrt(planeNormal.x() * planeNormal.x() + planeNormal.y() *
			planeNormal.y() + planeNormal.z() * planeNormal.z());
	return distance;
}

inline int ToVecIndex(int i, int j, int maxCol)
{
	return i * maxCol + j;
}

int ColorCategory(Vector3i color)
{

	if (color[0] == 255 && color[1] == 0 && color[2] == 0)
		return 0;
	else if (color[0] == 255 && color[1] == 255 && color[2] == 0)
		return 1;
	else if (color[0] == 0 && color[1] == 255 && color[2] == 0)
		return 2;
	else if (color[0] == 0 && color[1] == 255 && color[2] == 255)
		return 3;
	else if (color[0] == 0 && color[1] == 0 && color[2] == 255)
		return 4;
	else if (color[0] == 255 && color[1] == 0 && color[2] == 255)
		return 5;
	else
		return -1;

}


///A Cube actually has same dimensions in all sides but question is confusing and might be referring cuboid
void CreateCube(float length, float width, float height, Matrix4f toWorld, Cube &cube)
{
	cube.toWorld = toWorld;
	cube.cloud = PointCloudPtr(new PointCloudT);
	cube.cubeFaces.clear();

	///Create Corners
	{
		PointT temp(255, 255, 255);
		///Order matters for faces; Don't change
		/*0*/temp.x = 0; temp.y = 0; temp.z = 0;				cube.cloud->points.push_back(temp);
		/*1*/temp.x = 0; temp.y = 0; temp.z = length;			cube.cloud->points.push_back(temp);
		/*2*/temp.x = 0; temp.y = length; temp.z = 0;			cube.cloud->points.push_back(temp);
		/*3*/temp.x = 0; temp.y = length; temp.z = length;		cube.cloud->points.push_back(temp);
		/*4*/temp.x = length; temp.y = 0; temp.z = 0;			cube.cloud->points.push_back(temp);
		/*5*/temp.x = length; temp.y = 0; temp.z = length;		cube.cloud->points.push_back(temp);
		/*6*/temp.x = length; temp.y = length; temp.z = 0;		cube.cloud->points.push_back(temp);
		/*7*/temp.x = length; temp.y = length; temp.z = length;	cube.cloud->points.push_back(temp);
		cube.cloud->width = 8;
		cube.cloud->height = 1;
	}
	pcl::transformPointCloud(*cube.cloud, *cube.cloud, toWorld);
	///Create Faces
	{
		/*three face from 0, 0, 0*/
		cube.cubeFaces.push_back(FitPlane(PointT_To_Vector3f(cube.cloud->points[0]), PointT_To_Vector3f(cube.cloud->points[1]), PointT_To_Vector3f(cube.cloud->points[2])));
		cube.cubeFaces.push_back(FitPlane(PointT_To_Vector3f(cube.cloud->points[0]), PointT_To_Vector3f(cube.cloud->points[1]), PointT_To_Vector3f(cube.cloud->points[4])));
		cube.cubeFaces.push_back(FitPlane(PointT_To_Vector3f(cube.cloud->points[0]), PointT_To_Vector3f(cube.cloud->points[2]), PointT_To_Vector3f(cube.cloud->points[4])));
		/*three face from 1, 1, 1*/
		cube.cubeFaces.push_back(FitPlane(PointT_To_Vector3f(cube.cloud->points[7]), PointT_To_Vector3f(cube.cloud->points[6]), PointT_To_Vector3f(cube.cloud->points[3])));
		cube.cubeFaces.push_back(FitPlane(PointT_To_Vector3f(cube.cloud->points[7]), PointT_To_Vector3f(cube.cloud->points[6]), PointT_To_Vector3f(cube.cloud->points[5])));
		cube.cubeFaces.push_back(FitPlane(PointT_To_Vector3f(cube.cloud->points[7]), PointT_To_Vector3f(cube.cloud->points[3]), PointT_To_Vector3f(cube.cloud->points[5])));
	}



	///Create points
	{
		cube.cloud->points.clear();
		PointT temp(255, 255, 255);
		for (size_t i = 0; i < length; i += resolutionForCube)
		{
			for (size_t j = 0; j < width; j += resolutionForCube)
			{
				for (size_t k = 0; k < height; k += resolutionForCube)
				{
					if ((i == 0 || i == length - 1) || (j == 0 || j == width - 1) || (k == 0 || k == height - 1)) ///For only faces
					{
						temp.x = i; temp.y = j; temp.z = k;
						cube.cloud->points.push_back(temp);
					}
				}
			}
		}


		cube.cloud->width = cube.cloud->points.size();
		cube.cloud->height = 1;
	}

	pcl::transformPointCloud(*cube.cloud, *cube.cloud, toWorld);

	//for (Plane& plane : cube.cubeFaces)
	//{
	//	plane.normal = toWorld.block(0, 0, 3, 3) * plane.normal;
	//	plane.point = toWorld * Vector4f(plane.point.x(), plane.point.y(), plane.point.z(), 1);
	//}

	//ShowPointCloud(cube.cloud);
}


void CreateEmitter(float verticalFOV, float horizontalFOV, int verticalResolution, int horizontalResolution, int screenDistance, Matrix4f toWorld, Emitter &emitter)
{
	emitter.toCamera = toWorld;
	emitter.hFOV = horizontalFOV;
	emitter.vFOV = verticalFOV;
	emitter.hResolution = horizontalResolution;
	emitter.vResolution = verticalResolution;

	emitter.screen = PointCloudPtr(new PointCloudT);


	/////Create a projected ray board towards z-axis
	//{
	//	PointT temp(255,255,255);
	//	for (int i = (-emitter.hResolution / 2) + 0.5; i < emitter.hResolution / 2; i++)
	//	{
	//		for (int j = (-emitter.vResolution / 2) + 0.5; j < emitter.vResolution / 2; j++)
	//		{
	//			temp.x = i; temp.y = j; temp.z = 1000;
	//			cloud->points.push_back(temp);
	//		}
	//	}
	//	cloud->width = cloud->points.size();
	//	cloud->height = 1;
	//}
	/////Find z to satisfy FOV
	//{
	//	Vector3f v1, v2;
	//	v1[0] = (-emitter.hResolution / 2) + 0.5;
	//	v1[1] = (emitter.vResolution / 2) - 0.5;
	//	v1[2] = 0;
	//	v2[0] = (emitter.hResolution / 2) - 0.5;
	//	v2[1] = (emitter.vResolution / 2) - 0.5;
	//	v2[2] = 0;
	//}


	///FOV vectors: top left, top right, bottom left, bottom right
	vector<Vector3f> vectorsFOV;
	///at distance "factor": if resolution comes too high, rays will be too close and may be distorted because of rounding off issue from system.
	{
		Vector3f tempVector(0, 0, 1);
		AngleAxisf aa;


		///top left
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(-emitter.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(emitter.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, screenDistance));

		emitter.fovCornersOriginal.push_back(tempVector);

		///top right
		tempVector = Vector3f(0, 0, 1);
		aa = AngleAxisf(ToRadian(-emitter.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(-emitter.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, screenDistance));
		emitter.fovCornersOriginal.push_back(tempVector);

		///bottom left
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(emitter.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(emitter.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, screenDistance));
		emitter.fovCornersOriginal.push_back(tempVector);

		///bottom right
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(emitter.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(-emitter.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, screenDistance));
		emitter.fovCornersOriginal.push_back(tempVector);
	}


	///Create a projected ray board towards z-axis
	{
		float horizontalIncrement = abs((emitter.fovCornersOriginal[0].x()) - (emitter.fovCornersOriginal[1].x())) / emitter.hResolution;
		float verticalIncrement = abs((emitter.fovCornersOriginal[0].y()) - (emitter.fovCornersOriginal[2].y())) / emitter.vResolution;
		PointT temp(255, 0, 0);
		temp.x = 0; temp.y = 0; temp.z = 0;
		//emitter.screen->points.push_back(temp);///Emitter itself; for transformation with the point cloud;

		for (float j = emitter.fovCornersOriginal[3].y(); j < (emitter.fovCornersOriginal[0].y() - verticalIncrement * 0.5); j += verticalIncrement)
		{
			for (float i = (emitter.fovCornersOriginal[3].x()); i < (emitter.fovCornersOriginal[0].x() - horizontalIncrement * 0.5); i += horizontalIncrement)
			{
				temp.x = i; temp.y = j, temp.z = emitter.fovCornersOriginal[0].z();
				emitter.screen->points.push_back(temp);
			}
		}

		emitter.screen->width = emitter.screen->points.size();
		emitter.screen->height = 1;
	}

	pcl::transformPointCloud(*emitter.screen, *emitter.screen, toWorld);

	///Fill rays and emitter pose
	{
		Vector4f temp(0, 0, 0, 1);
		temp = toWorld * temp;
		emitter.position = Vector3f(temp.x(), temp.y(), temp.z());

		emitter.normalVectorFromSource = toWorld.block(0, 0, 3, 3) * Vector3f(0, 0, 1);

		emitter.rays = vector<vector <Vector3f>>(emitter.vResolution, vector<Vector3f>(emitter.hResolution));
		for (size_t i = 0; i < emitter.screen->points.size(); i++)
		{
			emitter.rays[i / emitter.hResolution][i % emitter.hResolution] = PointT_To_Vector3f(emitter.screen->points[i]) - emitter.position;
			emitter.rays[i / emitter.hResolution][i % emitter.hResolution].normalize();
		}
	}


}


void CreateCamera(float verticalFOV, float horizontalFOV, int verticalResolution, int horizontalResolution, Matrix3f cameraMatrix, Vector3f camPosition, Vector3f camVector, Matrix4f toWorld, Camera &camera)
{
	camera.toWorld = toWorld;
	camera.hFOV = horizontalFOV;
	camera.vFOV = verticalFOV;
	camera.hResolution = horizontalResolution;
	camera.vResolution = verticalResolution;
	camera.focalLength = cameraMatrix(0, 0);
	camera.intrinsic = cameraMatrix;
	camera.position = camPosition;
	camera.planeCloud = PointCloudPtr(new PointCloudT);

	{
		Vector3f temp;
		temp[0] = camPosition.x() + camera.focalLength * camVector.x();
		temp[1] = camPosition.y() + camera.focalLength * camVector.y();
		temp[2] = camPosition.z() + camera.focalLength * camVector.z();

		camera.screenPlane = Plane(temp, camVector);
	}

	/////Create a projected ray board towards z-axis
	//{
	//	PointT temp(255,255,255);
	//	for (int i = (-emitter.hResolution / 2) + 0.5; i < emitter.hResolution / 2; i++)
	//	{
	//		for (int j = (-emitter.vResolution / 2) + 0.5; j < emitter.vResolution / 2; j++)
	//		{
	//			temp.x = i; temp.y = j; temp.z = 1000;
	//			cloud->points.push_back(temp);
	//		}
	//	}
	//	cloud->width = cloud->points.size();
	//	cloud->height = 1;
	//}
	/////Find z to satisfy FOV
	//{
	//	Vector3f v1, v2;
	//	v1[0] = (-emitter.hResolution / 2) + 0.5;
	//	v1[1] = (emitter.vResolution / 2) - 0.5;
	//	v1[2] = 0;
	//	v2[0] = (emitter.hResolution / 2) - 0.5;
	//	v2[1] = (emitter.vResolution / 2) - 0.5;
	//	v2[2] = 0;
	//}


	//vector<Vector3f> vectorsFOV;///FOV vectors: top left, top right, bottom left, bottom right

	///at distance "factor": if resolution comes too high, rays will be too close and may be distorted because of rounding off issue from system.
	{
		Vector3f tempVector(0, 0, 1);
		AngleAxisf aa;


		///top left
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(-camera.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(camera.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, camera.focalLength));
		camera.fovCornersOriginal.push_back(tempVector);

		///top right
		tempVector = Vector3f(0, 0, 1);
		aa = AngleAxisf(ToRadian(-camera.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(-camera.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, camera.focalLength));
		camera.fovCornersOriginal.push_back(tempVector);

		///bottom left
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(camera.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(camera.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, camera.focalLength));
		camera.fovCornersOriginal.push_back(tempVector);

		///bottom right
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(camera.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(-camera.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, camera.focalLength));
		camera.fovCornersOriginal.push_back(tempVector);
	}



	//for (Vector3f& point : camera.fovCorners)
	//{
	//	Vector4f temp(point.x(), point.y(), point.z(), 1);
	//	temp = toWorld * temp;
	//	point[0] = temp.x();
	//	point[1] = temp.y();
	//	point[2] = temp.z();
	//}


	///Create a projected ray board towards z-axis
	{
		//float horizontalIncrement = abs((camera.vectorsFOV[0].x()) - (camera.vectorsFOV[1].x())) / camera.hResolution;
		//float verticalIncrement = abs((camera.vectorsFOV[0].y()) - (camera.vectorsFOV[2].y())) / camera.vResolution;
		//PointT temp(255, 0, 0);
		//temp.x = 0; temp.y = 0; temp.z = 0;
		//camera.screen->points.push_back(temp);///Emitter itself

		//for (float j = camera.vectorsFOV[2].y(); j < (camera.vectorsFOV[1].y() - verticalIncrement * 0.5); j += verticalIncrement)
		//{
		//	for (float i = (camera.vectorsFOV[2].x()); i < (camera.vectorsFOV[1].x() - horizontalIncrement * 0.5); i += horizontalIncrement)
		//	{
		//		temp.x = i; temp.y = j, temp.z = camera.vectorsFOV[0].z();
		//		camera.screen->points.push_back(temp);
		//	}
		//}

		//camera.screen->width = camera.screen->points.size();
		//camera.screen->height = 1;
	}

	//pcl::transformPointCloud(*camera.screen, *camera.screen, toWorld);
}


vector<int> GetRaySplashOnCube(Vector3f rayVector, Vector3f rayPoint, Cube &cube, float splashArea = resolutionForCube * 0.51, float maxDistance = 1000)
{
	float minTraverseIncrement = resolutionForCube;
	vector<Vector3f> intersectionOnFaces;


	pcl::search::KdTree<PointT> kdtree;
	kdtree.setInputCloud(cube.cloud);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	float distance = 0;

	PointT searchPoint;
	searchPoint.x = rayPoint.x() + rayVector.x() * distance;
	searchPoint.y = rayPoint.y() + rayVector.y() * distance;
	searchPoint.z = rayPoint.z() + rayVector.z() * distance;

	kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	distance = sqrt(pointNKNSquaredDistance[0]);

	///Just a thought of saving cost:
	///instead of reducing increment to 1mm (need lower if point cloud resolution is lower) and repeat kdtree search many times,
	///search in bigger area and then find the intersection of plane and update neighbours of the intersecting point.

	do
	{
		searchPoint.x = rayPoint.x() + rayVector.x() * distance;
		searchPoint.y = rayPoint.y() + rayVector.y() * distance;
		searchPoint.z = rayPoint.z() + rayVector.z() * distance;

		/// Test the ray tracing and the searched neighbours
		{
			//if (distance == 120)
			//{
			//	cout << "here" << endl;
			//	//searchPoint.x = 0; searchPoint.y = 0; searchPoint.z = 100;
			//	kdtree.radiusSearch(searchPoint, minTraverseIncrement / 2, pointIdxNKNSearch, pointNKNSquaredDistance);
			//	PointCloudPtr cloud(new PointCloudT);
			//	for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
			//	{
			//		cloud->points.push_back(cube.cloud->points[pointIdxNKNSearch[i]]);
			//	}
			//	cloud->width = cloud->points.size();
			//	cloud->height = 1;
			//	PointT temp3; temp3.x = 0; temp3.y = 0; temp3.z = 0;
			//	mainViewer->removeAllPointClouds();
			//	mainViewer->addLine(temp3, searchPoint, 255, 0, 0, "line1");
			//	mainViewer->addCoordinateSystem(100);
			//	ShowPointCloud(cloud);
			//	ShowPointCloud(cube.cloud);
			//	mainViewer->removeAllShapes();
			//}
		}


		kdtree.radiusSearch(searchPoint, minTraverseIncrement, pointIdxNKNSearch, pointNKNSquaredDistance);
		distance += minTraverseIncrement;

	} while ((pointIdxNKNSearch.size() == 0) && distance < maxDistance);

	if (pointIdxNKNSearch.size() > 0)
	{
		/// Test the ray tracing and the searched neighbours
		{
			//cout << "here" << endl;
			////searchPoint.x = 0; searchPoint.y = 0; searchPoint.z = 100;
			//kdtree.radiusSearch(searchPoint, minTraverseIncrement / 2, pointIdxNKNSearch, pointNKNSquaredDistance);
			//PointCloudPtr cloud(new PointCloudT);
			//for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
			//{
			//	cloud->points.push_back(cube.cloud->points[pointIdxNKNSearch[i]]);
			//}
			//cloud->width = cloud->points.size();
			//cloud->height = 1;
			//PointT temp3; temp3.x = 0; temp3.y = 0; temp3.z = 0;
			//mainViewer->removeAllPointClouds();
			//mainViewer->addLine(temp3, searchPoint, 255, 0, 0, "line1");
			//mainViewer->addCoordinateSystem(100);
			//ShowPointCloud(cloud);
			//ShowPointCloud(cube.cloud);
			//mainViewer->removeAllShapes();
		}

		Plane face;
		///Find closest plane;
		{
			vector<float> distanceToPlanes;
			Vector3f vec(cube.cloud->points[pointIdxNKNSearch[0]].x, cube.cloud->points[pointIdxNKNSearch[0]].y, cube.cloud->points[pointIdxNKNSearch[0]].z);
			for_each(cube.cubeFaces.begin(), cube.cubeFaces.end(), [vec, &distanceToPlanes](Plane& p) { return distanceToPlanes.push_back(DistanceToPlane(vec, p.normal, p.point)); });
			face = cube.cubeFaces[min_element(distanceToPlanes.begin(), distanceToPlanes.end()) - distanceToPlanes.begin()];
		}


		Vector3f point = GetIntersectionPointVectorAndPlane(rayVector, rayPoint, face.normal, face.point);

		///Test the intersection point and the plane selected
		{
			//PointT temp1, temp2, temp3, temp4, temp5;
			//temp1.x = face.point.x(); temp1.y = face.point.y(); temp1.z = face.point.z();
			//temp2.x = point.x(); temp2.y = point.y(); temp2.z = point.z();
			//temp4.x = rayPoint.x() + 100 * rayVector.x(); temp4.y = rayPoint.y() + 100 * rayVector.y(); temp4.z = rayPoint.z() + 100 * rayVector.z();
			//temp3.x = 0; temp3.y = 0; temp3.z = 0;
			//temp5 = cube.cloud->points[pointIdxNKNSearch[0]];
			//mainViewer->addLine(temp3, temp4, 255, 0, 0, "line1");
			//mainViewer->addLine(temp3, temp5, 0, 255, 0, "line2");
			//mainViewer->addLine(temp3, temp1, 0, 0, 255, "line3");
			//mainViewer->addLine(temp3, temp2, 255, 0, 255, "line4");
			////mainViewer->addSphere(temp2, 10, 0, 0, 255);
			//mainViewer->addCoordinateSystem(30);
			//AddPointCloud(cube.cloud);
			//mainViewer->removeAllShapes();
			//mainViewer->spinOnce();
			//ClearayVectoriewer();
		}


		if (point != Vector3f(0, 0, 0))
		{
			pointIdxNKNSearch.clear();
			PointT p = Vector3f_To_PointT(point);
			kdtree.radiusSearch(p, splashArea, pointIdxNKNSearch, pointNKNSquaredDistance);
			return pointIdxNKNSearch;
		}
		else
		{
			return vector<int>();
		}

	}
	else
	{
		return vector<int>();
	}
}


Vector3f GetRayIntersectionhOnCube(Vector3f rayVector, Vector3f rayPoint, Cube &cube, float maxDistance = 1000)
{
	float minTraverseIncrement = resolutionForCube;
	vector<Vector3f> intersectionOnFaces;


	pcl::search::KdTree<PointT> kdtree;
	kdtree.setInputCloud(cube.cloud);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	float distance = 0;

	PointT searchPoint;
	searchPoint.x = rayPoint.x() + rayVector.x() * distance;
	searchPoint.y = rayPoint.y() + rayVector.y() * distance;
	searchPoint.z = rayPoint.z() + rayVector.z() * distance;

	///To speed up; why waste time in searching empty space
	kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	distance = sqrt(pointNKNSquaredDistance[0]);

	///Just a thought of saving cost:
	///instead of reducing increment to 1mm (need lower if point cloud resolution is lower) and repeat kdtree search many times,
	///search in bigger area and then find the intersection of plane and update neighbours of the intersecting point.

	do
	{
		searchPoint.x = rayPoint.x() + rayVector.x() * distance;
		searchPoint.y = rayPoint.y() + rayVector.y() * distance;
		searchPoint.z = rayPoint.z() + rayVector.z() * distance;

		/// Test the ray tracing and the searched neighbours
		{
			//if (distance == 120)
			//{
			//	cout << "here" << endl;
			//	//searchPoint.x = 0; searchPoint.y = 0; searchPoint.z = 100;
			//	kdtree.radiusSearch(searchPoint, minTraverseIncrement / 2, pointIdxNKNSearch, pointNKNSquaredDistance);
			//	PointCloudPtr cloud(new PointCloudT);
			//	for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
			//	{
			//		cloud->points.push_back(cube.cloud->points[pointIdxNKNSearch[i]]);
			//	}
			//	cloud->width = cloud->points.size();
			//	cloud->height = 1;
			//	PointT temp3; temp3.x = 0; temp3.y = 0; temp3.z = 0;
			//	mainViewer->removeAllPointClouds();
			//	mainViewer->addLine(temp3, searchPoint, 255, 0, 0, "line1");
			//	mainViewer->addCoordinateSystem(100);
			//	ShowPointCloud(cloud);
			//	ShowPointCloud(cube.cloud);
			//	mainViewer->removeAllShapes();
			//}
		}


		kdtree.radiusSearch(searchPoint, minTraverseIncrement, pointIdxNKNSearch, pointNKNSquaredDistance);
		distance += minTraverseIncrement;

	} while ((pointIdxNKNSearch.size() == 0) && distance < maxDistance);

	if (pointIdxNKNSearch.size() > 0)
	{
		/// Test the ray tracing and the searched neighbours
		{
			//cout << "here" << endl;
			////searchPoint.x = 0; searchPoint.y = 0; searchPoint.z = 100;
			//kdtree.radiusSearch(searchPoint, minTraverseIncrement / 2, pointIdxNKNSearch, pointNKNSquaredDistance);
			//PointCloudPtr cloud(new PointCloudT);
			//for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
			//{
			//	cloud->points.push_back(cube.cloud->points[pointIdxNKNSearch[i]]);
			//}
			//cloud->width = cloud->points.size();
			//cloud->height = 1;
			//PointT temp3; temp3.x = 0; temp3.y = 0; temp3.z = 0;
			//mainViewer->removeAllPointClouds();
			//mainViewer->addLine(temp3, searchPoint, 255, 0, 0, "line1");
			//mainViewer->addCoordinateSystem(100);
			//ShowPointCloud(cloud);
			//ShowPointCloud(cube.cloud);
			//mainViewer->removeAllShapes();
		}

		Plane face;
		///Find closest plane;
		{
			vector<float> distanceToPlanes;
			Vector3f vec(cube.cloud->points[pointIdxNKNSearch[0]].x, cube.cloud->points[pointIdxNKNSearch[0]].y, cube.cloud->points[pointIdxNKNSearch[0]].z);
			for_each(cube.cubeFaces.begin(), cube.cubeFaces.end(), [vec, &distanceToPlanes](Plane& p) { return distanceToPlanes.push_back(DistanceToPlane(vec, p.normal, p.point)); });
			face = cube.cubeFaces[min_element(distanceToPlanes.begin(), distanceToPlanes.end()) - distanceToPlanes.begin()];
		}


		Vector3f point = GetIntersectionPointVectorAndPlane(rayVector, rayPoint, face.normal, face.point);

		return point;
	}
	else
	{
		return Vector3f(0, 0, 0);
	}
}


void IlluminateWCSFromEmitter(Emitter &emitter, Cube &cube, PointCloudPtr world, float maxDistance = 100, vector<size_t> colsToIlluminate = vector<size_t>())
{
	//PointCloudPtr world(new PointCloudT);
	//*world = *cube.cloud;
	world->points.clear();
	world->points.reserve(emitter.vResolution*emitter.hResolution / 2);
	pcl::search::KdTree<PointT> kdtree;
	kdtree.setInputCloud(cube.cloud);

	if (colsToIlluminate.size())
	{
		for (size_t i : colsToIlluminate)
		{
			for (size_t j = 0; j < emitter.vResolution; j++)
			{
				vector<int> affectedPoints = GetRaySplashOnCube(emitter.rays[j][i], emitter.position, cube);
				for (int index : affectedPoints)
				{
					PointT p = cube.cloud->points[index];
					switch (i % 6)
					{
					case 0:	p.r = 255; p.g = 0; p.b = 0; break;
					case 1: p.r = 255; p.g = 255; p.b = 0; break;
					case 2:	p.r = 0; p.g = 255; p.b = 0; break;
					case 3:	p.r = 0; p.g = 255; p.b = 255; break;
					case 4:	p.r = 0; p.g = 0; p.b = 255; break;
					case 5:	p.r = 255; p.g = 0; p.b = 255; break;
					default:	p.r = 0; p.g = 0; p.b = 0; break;
					}
					world->points.push_back(p);
				}

				//Vector3f point = GetRayIntersectionhOnCube(emitter.rays[j][i], emitter.position, cube);
				//PointT p;
				//p.x = point.x(); p.y = point.y(); p.z = point.z();
				//switch (i % 6)
				//{
				//case 0:	p.r = 255; p.g = 0; p.b = 0; break;
				//case 1: p.r = 255; p.g = 255; p.b = 0; break;
				//case 2:	p.r = 0; p.g = 255; p.b = 0; break;
				//case 3:	p.r = 0; p.g = 255; p.b = 255; break;
				//case 4:	p.r = 0; p.g = 0; p.b = 255; break;
				//case 5:	p.r = 255; p.g = 0; p.b = 255; break;
				//default:	p.r = 0; p.g = 0; p.b = 0; break;
				//}
				//world->points.push_back(p);
			}
		}
	}
	else
	{
		for (size_t i = 0; i < emitter.hResolution; i++)
		{
			for (size_t j = 0; j < emitter.vResolution; j += 1)
			{
				vector<int> affectedPoints = GetRaySplashOnCube(emitter.rays[j][i], emitter.position, cube);
				for (int index : affectedPoints)
				{
					PointT p = cube.cloud->points[index]; p.r = 255; p.g = 0; p.b = 0;
					world->points.push_back(p);
				}

				//Vector3f point = GetRayIntersectionhOnCube(emitter.rays[j][i], emitter.position, cube);
				//PointT p;
				//p.x = point.x(); p.y = point.y(); p.z = point.z();
				//p.r = 255; p.g = 0; p.b = 0;
				//world->points.push_back(p);

			}
		}
	}
	world->width = world->points.size();
	world->height = 1;
}



cv::Mat ToImage(PointCloudPtr cloud, Vector3f camPosition, Plane camPlane, vector<Vector3f> fovCorners, Matrix4f toWorld, uint imgWidth = 160, uint imgHeight = 120)
{
	cv::Mat image(imgHeight, imgWidth, CV_8UC3, cv::Scalar(0, 0, 0));

	PointCloudT::Ptr imgCloud(new PointCloudT);///tgt
	imgCloud->resize(cloud->size());

	Vector3f rayPoint, rayVector;

	for (size_t i = 0; i < cloud->size(); i++)
	{
		rayPoint[0] = cloud->points[i].x;
		rayPoint[1] = cloud->points[i].y;
		rayPoint[2] = cloud->points[i].z;
		rayVector = rayPoint - camPosition;

		rayVector.normalize();

		Vector3f temp = GetIntersectionPointVectorAndPlane(rayVector, rayPoint, camPlane.normal, camPlane.point);

		imgCloud->points[i].x = temp.x();
		imgCloud->points[i].y = temp.y();
		imgCloud->points[i].z = temp.z();
		imgCloud->points[i].r = cloud->points[i].r;
		imgCloud->points[i].g = cloud->points[i].g;
		imgCloud->points[i].b = cloud->points[i].b;
	}

	cout << "to imgCloud" << endl;

	PointT p(1, 1, 1), p1(0, 0, 0);
	p.x = camPosition.x(); p.y = camPosition.y(); p.z = camPosition.z();
	p1.x = camPosition.x() + 50 * camPlane.normal.x(); p1.y = camPosition.y() + 50 * camPlane.normal.y(); p1.z = camPosition.z() + 50 * camPlane.normal.z();
	mainViewer->addLine<pcl::PointXYZRGB>(p, p1, 255, 0, 0, "line");
	mainViewer->addCoordinateSystem(25);
	ShowPointCloud(cloud, "sdfcsdf");
	ShowPointCloud(imgCloud, "sdfcsdf");
	mainViewer->removeAllShapes();

	//SaveXYZPointCloud("imgCloud.xyz", cloud2->points);

	Matrix4f trans = Matrix4f::Identity();

	pcl::transformPointCloud(*imgCloud, *imgCloud, toWorld.inverse());



	{
		//PointCloudPtr cloud1(new PointCloudT);
		//for (size_t i = 0; i < fovCorners.size(); i++)
		//{
		//	PointT point(0, 255, 0);
		//	point.x = fovCorners[i].x();
		//	point.y = fovCorners[i].y();
		//	point.z = fovCorners[i].z();
		//	cloud1->points.push_back(point);
		//}
		//cloud1->width = cloud1->points.size();
		//cloud1->height = 1;
		//cout << "here" << endl;
		//AddPointCloud(cloud1, "asa");
		//ShowPointCloud(imgCloud, "sdfcsdf");
	}

	//ShowPointCloud(imgCloud, "sdfcsdf");

	//int minX = -cWidth / 2, maxX = cWidth / 2;
	//int minY = -cHeight / 2, maxY = cHeight / 2;
	int minX = fovCorners[1].x(), maxX = fovCorners[0].x();
	int minY = fovCorners[2].y(), maxY = fovCorners[0].y();

	int cWidth = (maxY - minY) * imgWidth / imgHeight, cHeight = maxY - minY;

	for_each(imgCloud->points.begin(), imgCloud->points.end(), [minX, minY, cWidth, cHeight, imgWidth, imgHeight](PointT &point)
	{
		point.x = imgWidth - (int)((((float)(point.x - minX)) / (float)cWidth) * imgWidth);
		point.y = imgHeight - (int)((((float)(point.y - minY)) / (float)cHeight) * imgHeight);
	});


	for (size_t i = 0; i < imgCloud->size(); i++)
	{
		if (INRANGE(imgCloud->points[i].x, 0, imgWidth - 1) && INRANGE(imgCloud->points[i].y, 0, imgHeight - 1))
		{
			image.at<Vec3b>(Point(imgCloud->points[i].x, imgCloud->points[i].y)) = Vec3b(imgCloud->points[i].b, imgCloud->points[i].g, imgCloud->points[i].r);
		}
	}
	return image;
}


cv::Mat ToImage(PointCloudPtr cloud, Matrix3f intrinsic, Matrix4f toWorld, uint imgWidth = 160, uint imgHeight = 120)
{
	cv::Mat image(imgHeight, imgWidth, CV_8UC3, cv::Scalar(0, 0, 0));

	PointCloudT::Ptr imgCloud(new PointCloudT);///tgt
	imgCloud->resize(cloud->size());

	pcl::transformPointCloud(*cloud, *imgCloud, toWorld);

	mainViewer->addCoordinateSystem(25);
	//ShowPointCloud(cloud);
	//ShowPointCloud(imgCloud);

	for_each(imgCloud->points.begin(), imgCloud->points.end(), [intrinsic](PointT &point)
	{
		Vector3f p(-point.x / point.z, -point.y / point.z, 1);
		p = intrinsic * p;
		point.x = p[0];
		point.y = p[1];
		point.z = 1;

		//point.x = point.x *intrinsic(0, 0) / point.z + intrinsic(0, 2);
		//point.y = point.y *intrinsic(0, 0) / point.z + intrinsic(1, 2);
	});
	//ShowPointCloud(imgCloud);

	for (PointT point : imgCloud->points)
	{
		if (INRANGE(point.x, 0, imgWidth - 1) && INRANGE(point.y, 0, imgHeight - 1))
		{
			image.at<Vec3b>(Point(point.x, point.y)) = Vec3b(point.b, point.g, point.r);
		}
	}

	return image;
}



struct Line
{
	int colorCategory;
	vector<Vector2i> indices;
	Line()
	{}
	Line(int c, Vector2i v) : colorCategory(c)
	{
		indices.push_back(v);
	}
};
void GetLinesIndices(Mat image, vector<Line>& lines)
{

	//imshow("asd", image);
	//waitKey();

	uint latestRow = 0;

	for (int c = 0; c < image.cols; ++c)
	{
		for (int r = 0; r < image.rows; ++r)
		{
			Vector3i color((int)(image.at<Vec3b>(r, c))[2], (int)(image.at<Vec3b>(r, c))[1], (int)(image.at<Vec3b>(r, c))[0]);
			int colorCategory = ColorCategory(color);
			if (colorCategory != -1)
			{
				bool found = false;
				for (int i = 0; ((int)(lines.size()) - 1 - i) >= 0 && i < 5; i++)
				{
					if (lines[lines.size() - 1 - i].colorCategory == colorCategory)
					{
						found = true;
						lines[lines.size() - 1 - i].indices.push_back(Vector2i(r, c));
					}
				}

				if (!found)
				{
					lines.push_back(Line(colorCategory, Vector2i(r, c)));
				}
			}
		}
	}

	///from top to bottom: in image CS, rows had been stored as first element(x) and cols as second(y)
	for (Line& line : lines)
	{
		sort(line.indices.begin(), line.indices.end(), [](const Eigen::Vector2i &p1, const Eigen::Vector2i &p2) { return p1.x() < p2.x(); });
	}

}


std::pair<int, int> MatchLines(vector<Line> camLines, vector<Line> emLines)
{
	int indexC = 6, indexE = 6, i = 0, j = 0;
	for (i = 0; i < 6; i++)
	{
		indexC = find_if(emLines.begin(), emLines.end(), [camLines, i](Line& line) { return camLines[i].colorCategory == line.colorCategory; }) - emLines.begin();
		if (indexC < 6)
			break;
	}

	for (j = 0; j < 6; j++)
	{
		indexE = find_if(camLines.begin(), camLines.end(), [emLines, j](Line& line) { return emLines[j].colorCategory == line.colorCategory; }) - camLines.begin();
		if (indexE < 6)
			break;
	}


	if (indexC < indexE)
	{
		if (indexC < 6)
		{
			return  make_pair(indexC, i);
		}
		else
		{
			return make_pair(-1, -1);
		}
	}
	else
	{
		if (indexE < 6)
		{
			return	make_pair(j, indexE);
		}
		else
		{
			return make_pair(-1, -1);
		}
	}


}



template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T, 3>& line)
{
	//Line is given as a*x + b*y + c = 0
	return std::fabsf(line(0)*point.x + line(1)*point.y + line(2))
		/ std::sqrt(line(0)*line(0) + line(1)*line(1));
}



void findFundamentalMatrix(std::vector<cv::Point2d> imagePointsLeftCamera, std::vector<cv::Point2d> imagePointsRightCamera)
{
	///http://ros-developer.com/2019/01/01/computing-essential-and-fundamental-matrix-using-opencv-8-points-algorithm-with-c/

	std::vector<cv::Point3d> imagePointsLeftCameraHomogeneous, imagePointsRightCameraHomogeneous;
	cv::convertPointsToHomogeneous(imagePointsLeftCamera, imagePointsLeftCameraHomogeneous);
	cv::convertPointsToHomogeneous(imagePointsRightCamera, imagePointsRightCameraHomogeneous);

	double u_prime, v_prime, u, v;
	cv::Mat A = cv::Mat_<double>(imagePointsLeftCamera.size(), 9);
	for (int i = 0; i < imagePointsLeftCamera.size(); i++)
	{
		u_prime = imagePointsLeftCamera.at(i).x;
		v_prime = imagePointsLeftCamera.at(i).y;

		u = imagePointsRightCamera.at(i).x;
		v = imagePointsRightCamera.at(i).y;

		A.at<double>(i, 0) = u_prime * u;
		A.at<double>(i, 1) = u_prime * v;
		A.at<double>(i, 2) = u_prime;
		A.at<double>(i, 3) = v_prime * u;
		A.at<double>(i, 4) = v_prime * v;
		A.at<double>(i, 5) = v_prime;
		A.at<double>(i, 6) = u;
		A.at<double>(i, 7) = v;
		A.at<double>(i, 8) = 1;

	}

	cv::Mat U, SingularValuesVector, VT;
	cv::Mat SigmaMatrix = cv::Mat::zeros(A.rows, A.cols, CV_64F);
	cv::SVD::compute(A.clone(), SingularValuesVector, U, VT);

	//////////////////////////////////Buliding U (Buliding Square Matrix U)///////////////////////////////////

	cv::Mat completeU = cv::Mat_<double>(U.rows, U.rows);
	cv::Mat missingElementsOfU = cv::Mat::zeros(U.rows, U.rows - U.cols, CV_64F);
	cv::hconcat(U, missingElementsOfU, completeU);

	//////////////////////////////////Buliding Sigma Matrix ///////////////////////////////////

	cv::Mat completeSigma = cv::Mat::zeros(completeU.cols, VT.rows, CV_64F);
	for (std::size_t i = 0; i < SingularValuesVector.rows; i++)
	{
		completeSigma.at<double>(i, i) = SingularValuesVector.at<double>(i, 0);
	}


	//////////////////////////////////Checking A=completeU*completeSigma*Vt ///////////////////////////////////

	std::cout << "checking A-U*Sigma*VT=0" << std::endl;
	std::cout << cv::sum(A - completeU * completeSigma*VT).val[0] << std::endl;

	///////////////////////////////////Building F Matrix From F vector /////////////////////////////////////////////
	cv::Mat F_vec = VT.col(VT.cols - 1);
	std::cout << F_vec.cols << std::endl;
	cv::Mat F = cv::Mat(3, 3, cv::DataType<double>::type);

	F.at<double>(0, 0) = F_vec.at<double>(0, 0);
	F.at<double>(0, 1) = F_vec.at<double>(1, 0);
	F.at<double>(0, 2) = F_vec.at<double>(2, 0);
	F.at<double>(1, 0) = F_vec.at<double>(3, 0);
	F.at<double>(1, 1) = F_vec.at<double>(4, 0);
	F.at<double>(1, 2) = F_vec.at<double>(5, 0);
	F.at<double>(2, 0) = F_vec.at<double>(6, 0);
	F.at<double>(2, 1) = F_vec.at<double>(7, 0);
	F.at<double>(2, 2) = F_vec.at<double>(8, 0);

	///////////////////////////////////Computing SVD of F /////////////////////////////////////////////

	cv::SVD::compute(F.clone(), SingularValuesVector, U, VT);
	std::cout << "F singular values" << std::endl;
	std::cout << SingularValuesVector << std::endl;

	///////////////////////////////////Setting The Smallest Eigen Value to Zero/////////////////////////////////////////////
	SingularValuesVector.at<double>(SingularValuesVector.rows - 1, 0) = 0;

	//////////////////////////////////Buliding U (Buliding Square Matrix U)///////////////////////////////////

	completeU = cv::Mat_<double>(U.rows, U.rows);
	missingElementsOfU = cv::Mat::zeros(U.rows, U.rows - U.cols, CV_64F);
	cv::hconcat(U, missingElementsOfU, completeU);

	//////////////////////////////////Buliding Sigma Matrix ///////////////////////////////////

	completeSigma = cv::Mat::zeros(completeU.cols, VT.rows, CV_64F);
	for (std::size_t i = 0; i < SingularValuesVector.rows; i++)
	{
		completeSigma.at<double>(i, i) = SingularValuesVector.at<double>(i, 0);
	}
	/////////////////////////////////////Building New F matrix///////////////////////////////////////

	cv::Mat NewF = completeU * completeSigma*VT;
	std::cout << "Fundamental Matrix is:" << std::endl;
	std::cout << NewF << std::endl;
}


void Rectify(MatrixXf cameraMatrix, AngleAxisf emiPose, Vector3f emiPosition)
{
	string folderName = "Images";

	Mat imgLeft = imread(folderName + "/left.png");
	Mat imgRight = imread(folderName + "/right.png");
	imshow(leftWindowName, imgLeft);
	imshow(rightWindowName, imgRight);
	waitKey();


	vector<Line> leftLines, rightLines;
	GetLinesIndices(imgLeft, leftLines);
	GetLinesIndices(imgRight, rightLines);


	///Test line indices
	{
		//Mat tempImage(imgLeft.rows, imgLeft.cols, imgLeft.type(), Scalar(0, 0, 0));
		//for (size_t i = 0; i < leftLines.size(); i++)
		//{
		//	for (size_t j = 0; j < leftLines[i].indices.size(); j++)
		//	{
		//		tempImage.at<Vec3b>(Point(leftLines[i].indices[j].y(), leftLines[i].indices[j].x())) = Vec3b(255, 255, 255);
		//	}
		//}
		//imshow(rightWindowName, tempImage);
		//waitKey();
	}



	pair<int, int> p = MatchLines(rightLines, leftLines);

	//cout << p.first << p.second << endl;


	vector<cv::Point2f> ptsLeft, ptsRight;
	Point2f cornerPtsLeft[4], cornerPtsRight[4];
	///Get matching points and  corner points in images
	{
		if (p.first != -1 && p.second != -1)
		{
			Point2f corner11, corner12, corner21, corner22;
			if (p.first > p.second)
			{
				for (size_t i = p.first, j = p.second; i < leftLines.size(); i++, j++)
				{
					if (j == leftLines.size() - 1)
					{
						corner12 = cv::Point2d(leftLines[i].indices[0].x(), leftLines[i].indices[0].y());
						corner22 = cv::Point2d(rightLines[j].indices[0].x(), rightLines[j].indices[0].y());
					}
					for (size_t k = 0; k < leftLines[i].indices.size() && k < rightLines[j].indices.size(); k++)
					{
						ptsLeft.push_back(cv::Point2d(leftLines[i].indices[k].x(), leftLines[i].indices[k].y()));
						ptsRight.push_back(cv::Point2d(rightLines[j].indices[k].x(), rightLines[j].indices[k].y()));
					}
					if (i == p.first)
					{
						corner11 = ptsLeft[ptsLeft.size() - 1];
						corner21 = ptsRight[ptsRight.size() - 1];
					}
				}
			}
			else
			{
				for (size_t i = p.first, j = p.second; j < rightLines.size(); i++, j++)
				{
					if (j == rightLines.size() - 1)
					{
						corner12 = cv::Point2d(leftLines[i].indices[0].x(), leftLines[i].indices[0].y());
						corner22 = cv::Point2d(rightLines[j].indices[0].x(), rightLines[j].indices[0].y());
					}
					for (size_t k = 0; k < leftLines[i].indices.size() && k < rightLines[j].indices.size(); k++)
					{
						ptsLeft.push_back(cv::Point2d(leftLines[i].indices[k].x(), leftLines[i].indices[k].y()));
						ptsRight.push_back(cv::Point2d(rightLines[j].indices[k].x(), rightLines[j].indices[k].y()));
					}
					if (i == p.first)
					{
						corner11 = ptsLeft[ptsLeft.size() - 1];
						corner21 = ptsRight[ptsRight.size() - 1];
					}
				}
			}


			///get Rectangle by first and last points

			cornerPtsLeft[0] = Point2f(ptsLeft[0].x, ptsLeft[0].y);
			cornerPtsLeft[1] = Point2f(corner11.x, corner11.y);
			cornerPtsLeft[2] = Point2f(corner12.x, corner12.y);
			cornerPtsLeft[3] = Point2f(ptsLeft[ptsLeft.size() - 1].x, ptsLeft[ptsLeft.size() - 1].y);

			cornerPtsRight[0] = Point2f(ptsRight[0].x, ptsRight[0].y);
			cornerPtsRight[1] = Point2f(corner21.x, corner21.y);
			cornerPtsRight[2] = Point2f(corner22.x, corner22.y);
			cornerPtsRight[3] = Point2f(ptsRight[ptsRight.size() - 1].x, ptsRight[ptsRight.size() - 1].y);

			//tempImage.at<Vec3b>(Point(ptsInPt2f[0].y, ptsInPt2f[0].x)) = Vec3b(255, 0, 255);
			//tempImage.at<Vec3b>(Point(ptsInPt2f[1].y, ptsInPt2f[1].x)) = Vec3b(255, 0, 255);
			//tempImage.at<Vec3b>(Point(ptsInPt2f[2].y, ptsInPt2f[2].x)) = Vec3b(255, 0, 255);
			//tempImage.at<Vec3b>(Point(ptsInPt2f[3].y, ptsInPt2f[3].x)) = Vec3b(255, 0, 255);

			Mat image1(imgLeft.rows, imgLeft.cols, imgLeft.type(), Scalar(0, 0, 0)), image2(imgRight.rows, imgRight.cols, imgRight.type(), Scalar(0, 0, 0));
			image1 = imgLeft.clone();
			image2 = imgRight.clone();

			image1.at<Vec3b>(Point(cornerPtsLeft[0].y, cornerPtsLeft[0].x)) = Vec3b(255, 255, 255);
			image1.at<Vec3b>(Point(cornerPtsLeft[1].y, cornerPtsLeft[1].x)) = Vec3b(255, 255, 255);
			image1.at<Vec3b>(Point(cornerPtsLeft[2].y, cornerPtsLeft[2].x)) = Vec3b(255, 255, 255);
			image1.at<Vec3b>(Point(cornerPtsLeft[3].y, cornerPtsLeft[3].x)) = Vec3b(255, 255, 255);

			image2.at<Vec3b>(Point(cornerPtsRight[0].y, cornerPtsRight[0].x)) = Vec3b(255, 255, 255);
			image2.at<Vec3b>(Point(cornerPtsRight[1].y, cornerPtsRight[1].x)) = Vec3b(255, 255, 255);
			image2.at<Vec3b>(Point(cornerPtsRight[2].y, cornerPtsRight[2].x)) = Vec3b(255, 255, 255);
			image2.at<Vec3b>(Point(cornerPtsRight[3].y, cornerPtsRight[3].x)) = Vec3b(255, 255, 255);

			imshow(leftWindowName, image1);
			imshow(rightWindowName, image2);
			waitKey();
		}
	}

	emiPosition[0] = -20;
	Matrix3f essentialMatrix, fundamentalMatrix;
	Matrix3f tx;
	tx << 0, -emiPosition[2], emiPosition[1]
		, emiPosition[2], 0, -emiPosition[0]
		, -emiPosition[1], emiPosition[0], 0;

	essentialMatrix = tx * emiPose.toRotationMatrix();

	Matrix3f intrinsic = cameraMatrix.block(0, 0, 3, 3);

	fundamentalMatrix = intrinsic.transpose().inverse() * essentialMatrix * intrinsic.inverse();
	cout << fundamentalMatrix << endl;


	Vector3f A = intrinsic * emiPose.toRotationMatrix() * emiPosition;
	Matrix3f C;
	C(0, 0) = 0; C(0, 1) = -A[2]; C(0, 2) = A[1];
	C(1, 0) = A[2]; C(1, 1) = 0; C(1, 2) = -A[0];
	C(2, 0) = -A[1]; C(2, 1) = A[0]; C(2, 2) = 0;
	fundamentalMatrix = intrinsic.inverse().transpose() * emiPose.toRotationMatrix() * intrinsic.transpose() * C;
	cout << fundamentalMatrix << endl;



	for (size_t i = 0; i < ptsRight.size(); i+=5)
	{
		Vector3f line(0, 0, 0);
		Vector3f pt(ptsRight[i].x, ptsRight[i].y, 1), pt2;


		line = fundamentalMatrix * pt;

		float x1 = 0, x2 = imgLeft.size().width;

		float y1 = (-line[2] - line[0] * (x1)) / line[1];
		float y2 = (-line[2] - line[0] * (x2)) / line[1];


		cv::line(imgLeft, Point2i(x1, y1), Point2i(x2, y2), Scalar(0, 0, 255), 1);


		imgRight.at<Vec3b>(ptsRight[i].x, ptsRight[i].y) = Vec3b(255, 255, 255);

		imshow(leftWindowName, imgLeft);
		imshow(rightWindowName, imgRight);
		waitKey();
		cout << "#" << endl;
	}


}



///Uses only first camera position; not sure about different emitter poses with camera
void CV_Test(MatrixXf cameraMatrix, Vector3f cubeDimension, Vector3f cubePosition, AngleAxisf cubePose
	, float emitterFOVv, float emitterFOVh, int emiResv, int emiResh, AngleAxisf emiPose, Vector3f emiPosition
	, float camFOVv, float camFOVh, int camResv, int camResh
	, vector<Vector3f> cameraPositions, vector<AngleAxisf> cameraPoses, string folderName)
{
	Cube cube;
	Camera camera;
	float focalLength = cameraMatrix(0, 0);
	Vector4f camVector(0, 0, 1, 0), camPosition(0, 0, 0, 1), camPlanePoint(0, 0, focalLength, 1);
	Emitter emitter;


	///Create Cube
	{
		AngleAxisf aa(cubePose);
		Matrix4f trans = Matrix4f::Identity();
		trans(0, 3) = cubePosition.x();
		trans(1, 3) = cubePosition.y();
		trans(2, 3) = cubePosition.z();
		trans.block(0, 0, 3, 3) = aa.toRotationMatrix();

		CreateCube(cubeDimension[0], cubeDimension[1], cubeDimension[2], trans, cube);
		cout << "Cube Created" << endl;
	}

	///Create Camera
	{
		///Create camera
		AngleAxisf aa(cameraPoses[0].angle(), Vector3f(0, 1, 0));
		Matrix4f transformation = Matrix4f::Identity();
		transformation(0, 3) = cameraPositions[0][0];
		transformation(1, 3) = cameraPositions[0][1];
		transformation(2, 3) = cameraPositions[0][2];
		transformation.block(0, 0, 3, 3) = aa.toRotationMatrix();
		camPosition = transformation * camPosition;
		camPlanePoint = transformation * camPlanePoint;
		camVector = transformation * camVector;

		CreateCamera(camFOVv, camFOVh, camResv, camResh, cameraMatrix, Vector3f(camPosition.x(), camPosition.y(), camPosition.z()), Vector3f(camVector.x(), camVector.y(), camVector.z()), transformation, camera);
		cout << "Camera Created" << endl;
	}

	///Create Emitter
	{
		float focalLength = cameraMatrix(0, 0);
		Matrix4f transformation = Matrix4f::Identity();
		transformation = Matrix4f::Identity();
		AngleAxisf aa(emiPose);
		transformation(0, 3) = emiPosition[0];
		transformation(1, 3) = emiPosition[1];
		transformation(2, 3) = emiPosition[2];
		transformation.block(0, 0, 3, 3) = aa.toRotationMatrix();

		CreateEmitter(emitterFOVv, emitterFOVh, emiResv, emiResh, focalLength, transformation, emitter);
		cout << "Emitter Created" << endl;
	}



	PointCloudPtr world(new PointCloudT);
	int colFreq = 5;
	///Get illuminated points in world from the rays in the given column frequency: colFreq=1 every col, colFreq=2 every 2nd col
	{
		vector<size_t> colsToIlluminate(ceil((float)emitter.hResolution / colFreq));
		for (size_t i = 0, j = 0; i < ceil((float)emitter.hResolution / colFreq); i++, j += colFreq) { colsToIlluminate[i] = j; }
		for (size_t i = 0; i < emitter.screen->points.size(); i++)
		{
			if ((i%emitter.hResolution) % colFreq)
			{
				emitter.screen->points[i].r = 255;
				emitter.screen->points[i].g = 255;
				emitter.screen->points[i].b = 255;
			}
		}
		IlluminateWCSFromEmitter(emitter, cube, world, 100, colsToIlluminate);
	}


	///Display results
	{
		mainViewer->addCoordinateSystem(25);
		cout << "Cube display" << endl;
		AddPointCloud(cube.cloud, "cube");
		PointT emiPosi; emiPosi.x = emitter.position.x(); emiPosi.y = emitter.position.y(); emiPosi.z = emitter.position.z();
		AddCube(emiPosi, "emitter_pos", 2);
		cout << "Emitter display" << endl;
		AddPointCloud(emitter.screen, "emitter");

		cout << "Illuminated World display" << endl;
		AddPointCloud(world, "world");

		ClearViewer();
	}


	Mat camImage, emitterImage;
	///Grab images
	{
		Vector3f camVector_3f(camVector.x(), camVector.y(), camVector.z());
		camVector_3f.normalize();
		Plane camPlane(Vector3f(camPlanePoint.x(), camPlanePoint.y(), camPlanePoint.z()), camVector_3f);
		cout << "Image at cam:" << endl;
		//camImage = ToImage(world, Vector3f(camPosition.x(), camPosition.y(), camPosition.z()), camPlane, camera.fovCornersOriginal, camera.toWorld, camera.hResolution, camera.vResolution);
		camImage = ToImage(world, cameraMatrix, camera.toWorld, camera.hResolution, camera.vResolution);

		Vector3f emitterPlanePoint;
		emitterPlanePoint[0] = emitter.screen->points[emitter.screen->points.size() / 2].x;
		emitterPlanePoint[1] = emitter.screen->points[emitter.screen->points.size() / 2].y;
		emitterPlanePoint[2] = emitter.screen->points[emitter.screen->points.size() / 2].z;
		cout << "Image at emitter:" << endl;
		//emitterImage = ToImage(world, emitter.position, Plane(emitterPlanePoint, emitter.normalVectorFromSource), emitter.fovCornersOriginal, Matrix4f::Identity(), camera.hResolution, camera.vResolution);
		emitterImage = ToImage(world, cameraMatrix, camera.toWorld.inverse()*emitter.toCamera.inverse(), camera.hResolution, camera.vResolution);
	}

	//cout << camera.toWorld*emitter.toCamera << endl;



	Mat imgLeft, imgRight;

	if (0)///Check which one shall be left
	{
		imgLeft = camImage.clone();
		imgRight = emitterImage.clone();
	}
	else
	{
		imgLeft = emitterImage.clone();
		imgRight = camImage.clone();
	}


	cout << endl << "Images grabbed shown" << endl;
	imshow(leftWindowName, imgLeft);
	imshow(rightWindowName, imgRight);
	waitKey();

	imwrite(folderName + "/left.png", imgLeft);
	imwrite(folderName + "/right.png", imgRight);


	//Mat imgLeft = imread(folderName + "/left.png");
	//Mat imgRight = imread(folderName + "/right.png");
	//imshow(leftWindowName, imgLeft);
	//imshow(rightWindowName, imgRight);
	//waitKey();





	cout << "done" << endl;
	//waitKey();
}



int main(int argc, char* argv[])
{

	//Mat image = imread("image.png", IMREAD_COLOR);
	//vector<vector<Vector2i>> lines;
	//GetLinesIndices(image, lines);
	//Mat let, ri;
	//DepthImage(let, ri);
	//Rectify();
	//return 0;

	InitialiseViewer();

	Matrix3f cameraMatrix; Vector3f cubeDimension; Vector3f cubePosition; AngleAxisf cubePose
		; float emitterFOVv; float emitterFOVh; int emiResv; int emiResh; AngleAxisf emiPose; Vector3f emiPosition
		; float camFOVv; float camFOVh; int camResv; int camResh
		; vector<Vector3f> cameraPositions; vector<AngleAxisf> cameraPoses; string folderName
		; float focalLength;

	focalLength = 280;
	cameraMatrix = Matrix3f::Identity();
	cameraMatrix(0, 0) = focalLength;
	cameraMatrix(1, 1) = focalLength;
	cameraMatrix(0, 2) = 80;
	cameraMatrix(1, 2) = 60;

	camResh = 2 * cameraMatrix(0, 2); camResv = 2 * cameraMatrix(1, 2);
	///https://stackoverflow.com/questions/39992968/how-to-calculate-field-of-view-of-the-camera-from-camera-intrinsic-matrix
	camFOVh = (2 * atan(camResh / (2 * cameraMatrix(0, 0)))) * 180 / M_PI; camFOVv = (2 * atan(camResv / (2 * cameraMatrix(1, 1)))) * 180 / M_PI;
	emitterFOVh = 32; emitterFOVv = 24;  emiResh = 96; emiResv = 200;

	int cubeLength = 20;
	cubeDimension = Vector3f(cubeLength, cubeLength, cubeLength);
	cubePosition = Vector3f(-5, -10, 60);
	cubePose = AngleAxisf((-50.0 / 180.0*M_PI), Vector3f(0, 1, 0));
	folderName = "Images";

	emiPosition = Vector3f(20, 0, 0);
	emiPose = AngleAxisf((-20 / 180.0*M_PI), Vector3f(0, 1, 0));
	cameraPositions.push_back(Vector3f(-20, 0, 0));
	cameraPoses.push_back(AngleAxisf((20 / 180.0*M_PI), Vector3f(0, 1, 0)));
	//cameraPositions.push_back(Vector3f(0, 0, 0));
	//cameraPoses.push_back(AngleAxisf(0, Vector3f(0, 0, 0)));


	//CV_Test(cameraMatrix, cubeDimension, cubePosition, cubePose
	//	, emitterFOVv, emitterFOVh, emiResv, emiResh, emiPose, emiPosition, camFOVv, camFOVh, camResv, camResh, cameraPositions, cameraPoses, folderName);


	Rectify(cameraMatrix, emiPose, emiPosition);

	cv::destroyAllWindows();
	cout << "done" << endl;

	return 0;

	//////red:x, green:y blue:z
	//mainViewer->addLine(temp1, temp, 255, 255, 0, "line1");
	////mainViewer->addSphere(temp, 5, 255, 255, 255, "asd");
	//AddCube(temp, "cd", 2);
	//mainViewer->addCoordinateSystem(5);
	//mainViewer->spin();
	//mainViewer->removeAllShapes();
	//return 0;
	//Vector3f point(4, 0, 0), planeN(1,0,0), planeP(1,0,0);
	//cout << DistanceToPlane(point,planeN, planeP);
	//return 0;




	//mainViewer->spin();
	//getchar();


}