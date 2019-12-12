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



pcl::visualization::PCLVisualizer::Ptr mainViewer(new pcl::visualization::PCLVisualizer("3D Viewer"));;



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
	viewerWindowPosition = std::make_pair<unsigned int, unsigned int>(500, 25);

	mainViewer->setPosition(viewerWindowPosition.first, viewerWindowPosition.second);
	mainViewer->setSize(viewerWindowSize.first, viewerWindowSize.second);

	pcl::PointCloud<PointTypeLocal>::Ptr cloud(new pcl::PointCloud<PointTypeLocal>);
	cloud->points.push_back(PointTypeLocal(0, 0, 0));
	mainViewer->addPointCloud(cloud);
	mainViewer->spinOnce();
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

void ClearayVectoriewer()
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
struct Emitter: Device
{
	vector<vector <Vector3f>> rays;
	PointCloudPtr screen;
	Matrix4f toCamera;
};
struct Camera:Device
{
	Matrix4f toWorld;
};

const float resolutionForCube = 1; ///1 mm


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
					//if ((i==0 || i == length-1) || (j == 0 || j == width - 1) || (k == 0 || k == height - 1)) ///For only faces
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
		emitter.screen->points.push_back(temp);///Emitter itself

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
		emitter.position = Vector3f(emitter.screen->points[0].x, emitter.screen->points[0].y, emitter.screen->points[0].z);
		Vector3f o = PointT_To_Vector3f(emitter.screen->points[0]);
		emitter.screen->points.erase(emitter.screen->points.begin());

		emitter.normalVectorFromSource = toWorld.block(0, 0, 3, 3) * Vector3f(0, 0, 1);

		emitter.rays = vector<vector <Vector3f>>(emitter.vResolution, vector<Vector3f>(emitter.hResolution));
		for (size_t i = 0; i < emitter.screen->points.size(); i++)
		{
			emitter.rays[i / emitter.hResolution][i % emitter.hResolution] = PointT_To_Vector3f(emitter.screen->points[i]) - o;
			emitter.rays[i / emitter.hResolution][i % emitter.hResolution].normalize();
		}
	}


}


void CreateCamera(float verticalFOV, float horizontalFOV, int verticalResolution, int horizontalResolution, float focalLength, Vector3f camPosition, Vector3f camVector, Matrix4f toWorld, Camera &camera)
{
	camera.toWorld = toWorld;
	camera.hFOV = horizontalFOV;
	camera.vFOV = verticalFOV;
	camera.hResolution = horizontalResolution;
	camera.vResolution = verticalResolution;
	camera.focalLength = focalLength;
	camera.position = camPosition;
	camera.planeCloud = PointCloudPtr(new PointCloudT);

	{
		Vector3f temp;
		temp[0] = camPosition.x() + focalLength * camVector.x();
		temp[1] = camPosition.y() + focalLength * camVector.y();
		temp[2] = camPosition.z() + focalLength * camVector.z();

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
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, focalLength));
		camera.fovCornersOriginal.push_back(tempVector);

		///top right
		tempVector = Vector3f(0, 0, 1);
		aa = AngleAxisf(ToRadian(-camera.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(-camera.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, focalLength));
		camera.fovCornersOriginal.push_back(tempVector);

		///bottom left
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(camera.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(camera.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, focalLength));
		camera.fovCornersOriginal.push_back(tempVector);

		///bottom right
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(ToRadian(camera.vFOV / 2), Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(ToRadian(-camera.hFOV / 2), Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;
		tempVector = GetIntersectionPointVectorAndPlane(tempVector, Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, focalLength));
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


vector<int> GetRaySplashOnCube(Vector3f rayVector, Vector3f rayPoint, Cube &cube, float splashArea = resolutionForCube * 0.55, float maxDistance = 1000)
{
	float minTraverseIncrement = resolutionForCube * 2;
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


void IlluminateWCSFromEmitter(Emitter &emitter, Cube &cube, PointCloudPtr world, float maxDistance = 100, vector<size_t> colsToIlluminate = vector<size_t>())
{
	//PointCloudPtr world(new PointCloudT);
	//*world = *cube.cloud;
	world->points.clear();
	world->points.resize(emitter.vResolution*emitter.hResolution / 2);
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

				//for (size_t k = 0; k < indices.size(); k++)
				//{
				//	world->points.push_back(cube.cloud->points[indices[k]]);
				//}
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
	mainViewer->addCoordinateSystem(50);
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
	if (indexC > 5)
	{
		for (j = 0; j < 6; j++)
		{
			indexE = find_if(camLines.begin(), camLines.end(), [emLines, j](Line& line) { return emLines[j].colorCategory == line.colorCategory; }) - camLines.begin();
			if (indexE < 6)
				break;
		}
	}


	if (indexC < 6)
	{
		return make_pair(indexC, i);
	}
	else if (indexE < 6)
	{
		return make_pair(j, indexE);
	}
	else
	{
		return make_pair(0, 0);
	}


}


void DepthImage(Mat leftimg, Mat rightimg)
{
	//const char *windowDisparity = "Disparity";

	////-- 1. Read the images
	//imgLeft = imread("left.png", IMREAD_GRAYSCALE);
	//imgRight = imread("right.png", IMREAD_GRAYSCALE);
	////-- And create the image in which we will save our disparities
	//Mat imgDisparity16S = Mat(imgLeft.rows, imgLeft.cols, CV_16S);
	//Mat imgDisparity8U = Mat(imgLeft.rows, imgLeft.cols, CV_8UC1);

	//if (imgLeft.empty() || imgRight.empty())
	//{
	//	std::cout << " --(!) Error reading images " << std::endl; return;
	//}

	////-- 2. Call the constructor for StereoBM
	//int ndisparities = 8;   /**< Range of disparity */
	//int SADWindowSize = 21; /**< Size of the block window. Must be odd */

	//Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);

	////-- 3. Calculate the disparity image
	//sbm->compute(imgLeft, imgRight, imgDisparity16S);

	////-- Check its extreme values
	//double minVal; double maxVal;

	//minMaxLoc(imgDisparity16S, &minVal, &maxVal);

	//printf("Min disp: %f Max value: %f \n", minVal, maxVal);

	////-- 4. Display it as a CV_8UC1 image
	//imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255 / (maxVal - minVal));

	//namedWindow(windowDisparity, WINDOW_NORMAL);
	//imshow(windowDisparity, imgDisparity8U);

	////-- 5. Save the image
	//imwrite("SBM_sample.png", imgDisparity16S);




	//leftimg = imread("left.png");
	//rightimg = imread("right.png");
	//cv::Size imagesize = leftimg.size();
	//cv::Mat g1, g2, disp, disp8;
	//cv::cvtColor(leftimg, g1, cv::COLOR_BGR2GRAY);
	//cv::cvtColor(rightimg, g2, cv::COLOR_BGR2GRAY);

	//cv::StereoBM *sbm = StereoBM::create(96, 3); //createStereoBM(16, 2);

	//sbm->setMinDisparity(1); 
	//sbm->setDisp12MaxDiff(30);
	////sbm->setSpeckleRange(8);
	////sbm->setSpeckleWindowSize(0);
	//sbm->setUniquenessRatio(5);
	////sbm->setTextureThreshold(507);
	////sbm->setPreFilterCap(61);
	////sbm->setPreFilterSize(5);
	//sbm->compute(g1, g2, disp);
	//normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

	//cv::imshow("left", leftimg);
	//cv::imshow("right", rightimg);
	//cv::imshow("disp", disp8);
}


void StereoCalibration(Mat img1, Mat img2)
{
	vector< vector< Point3f > > object_points;
	vector< vector< Point2f > > imagePoints1, imagePoints2;
	vector< Point2f > corners1, corners2;
	vector< vector< Point2f > > left_img_points, right_img_points;

	Mat gray1, gray2;
	cvtColor(img1, gray1, CV_BGR2GRAY);
	cvtColor(img2, gray2, CV_BGR2GRAY);

	bool found1 = true, found2 = true;

	Size board_size = Size(9, 20);
	int board_n = 9 * 20;

	cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
		cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	cv::drawChessboardCorners(gray1, board_size, corners1, found1);

	cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
		cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	cv::drawChessboardCorners(gray2, board_size, corners2, found2);
}


///NOT FINISHED !!!!
void CV_Test(Mat cameraMatrix, float length, float width, float height, Vector3f cubePosition, AngleAxisf cubePose
	, float emitterFOVv, float emitterFOVh, int emiResv, int emiResh, AngleAxisf emiPose, vector<AngleAxisf> cameraPoses, string folderName)
{

	Matrix4f trans = Matrix4f::Identity();
	trans(0, 3) = -10;
	trans(1, 3) = -10;
	trans(2, 3) = 50;

	Cube cube;
	CreateCube(20, 20, 20, trans, cube);
	cout << "Cube Created" << endl;


	float focalLength = 50;

	Emitter emitter;
	CreateEmitter(24, 32, 48, 64, focalLength, Matrix4f::Identity(), emitter);
	cout << "Emitter Created" << endl;


	///Create camera
	AngleAxisf aa((-45.0 / 180.0*M_PI), Vector3f(0, 1, 0));
	Vector4f camVector(0, 0, 1, 0), camPosition(0, 0, 0, 1), camPlanePoint(0, 0, 50, 1);
	Matrix4f transformation = Matrix4f::Identity();
	transformation(0, 3) = 50;
	transformation.block(0, 0, 3, 3) = aa.toRotationMatrix();
	camPosition = transformation * camPosition;
	camPlanePoint = transformation * camPlanePoint;
	camVector = transformation * camVector;
	Vector3f camVector1(camVector.x(), camVector.y(), camVector.z());
	camVector1.normalize();
	Camera camera;
	CreateCamera(24, 32, 48, 64, focalLength, Vector3f(camPosition.x(), camPosition.y(), camPosition.z()), Vector3f(camVector.x(), camVector.y(), camVector.z()), transformation, camera);
	cout << "Camera Created" << endl;



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
	cout << "Illuminated" << endl;


	///Display results
	{
		mainViewer->addCoordinateSystem(50);
		cout << "Cube display" << endl;
		AddPointCloud(cube.cloud, "cube");
		cout << "Emitter display" << endl;
		AddPointCloud(emitter.screen, "emitter");

		cout << "World display" << endl;
		AddPointCloud(world, "world");

		ClearayVectoriewer();
	}


	Mat camImage, emitterImage;
	///Grab images
	{
		Plane camPlane(Vector3f(camPlanePoint.x(), camPlanePoint.y(), camPlanePoint.z()), camVector1);

		camImage = ToImage(world, Vector3f(camPosition.x(), camPosition.y(), camPosition.z()), camPlane, camera.fovCornersOriginal, transformation);
		Vector3f emitterPlanePoint;
		emitterPlanePoint[0] = emitter.screen->points[emitter.screen->points.size() / 2].x;
		emitterPlanePoint[1] = emitter.screen->points[emitter.screen->points.size() / 2].y;
		emitterPlanePoint[2] = emitter.screen->points[emitter.screen->points.size() / 2].z;
		emitterImage = ToImage(world, emitter.position, Plane(emitterPlanePoint, emitter.normalVectorFromSource), emitter.fovCornersOriginal, Matrix4f::Identity());
	}


	uint imgWidth = 640, imgHeight = 480;
	string windowName = "Cloud2Image";
	namedWindow(windowName, WINDOW_NORMAL);
	resizeWindow(windowName, 360 * imgWidth / imgHeight, 360);

	imshow(windowName, camImage);
	waitKey();
	imshow(windowName, emitterImage);
	waitKey();


	imwrite("left.png", camImage);
	imwrite("right.png", emitterImage);



	vector<Line> camLines, emLines;
	GetLinesIndices(camImage, camLines);
	GetLinesIndices(emitterImage, emLines);

	pair<int, int> p = MatchLines(emLines, camLines);

	//cout << p.first << p.second << endl;

	vector<Vector2i> points1, points2;
	if (p.first > p.second)
	{

		for (size_t i = p.first, j = p.second; i < camLines.size(); i++, j++)
		{
			for (size_t k = 0; k < camLines[i].indices.size() && k < emLines[j].indices.size(); k++)
			{
				points1.push_back(Vector2i(camLines[i].indices[k]));
				points2.push_back(Vector2i(emLines[j].indices[k]));
			}
		}
	}
	else
	{
		for (size_t i = p.first, j = p.second; j < emLines.size(); i++, j++)
		{
			for (size_t k = 0; k < camLines[i].indices.size() && k < emLines[j].indices.size(); k++)
			{
				points1.push_back(Vector2i(camLines[i].indices[k]));
				points2.push_back(Vector2i(emLines[j].indices[k]));
			}
		}
	}


	//Mat FundamentalMatrix = findFundamentalMat(points1, points2, FM_8POINT);

	cout << "done" << endl;
}


int main(int argc, char* argv[])
{

	//Mat image = imread("image.png", IMREAD_COLOR);
	//vector<vector<Vector2i>> lines;
	//GetLinesIndices(image, lines);
	//Mat let, ri;
	//DepthImage(let, ri);
	//return 0;

	InitialiseViewer();

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


	Matrix4f trans = Matrix4f::Identity();
	trans(0, 3) = -10;
	trans(1, 3) = -10;
	trans(2, 3) = 50;

	Cube cube;
	CreateCube(20, 20, 20, trans, cube);
	cout << "Cube Created" << endl;


	float focalLength = 50;

	Emitter emitter;
	CreateEmitter(24, 32, 48, 64, focalLength, Matrix4f::Identity(), emitter);
	cout << "Emitter Created" << endl;


	///Create camera
	AngleAxisf aa((-45.0 / 180.0*M_PI), Vector3f(0, 1, 0));
	Vector4f camVector(0, 0, 1, 0), camPosition(0, 0, 0, 1), camPlanePoint(0, 0, 50, 1);
	Matrix4f transformation = Matrix4f::Identity();
	transformation(0, 3) = 50;
	transformation.block(0, 0, 3, 3) = aa.toRotationMatrix();
	camPosition = transformation * camPosition;
	camPlanePoint = transformation * camPlanePoint;
	camVector = transformation * camVector;
	Vector3f camVector1(camVector.x(), camVector.y(), camVector.z());
	camVector1.normalize();
	Camera camera;
	CreateCamera(24, 32, 48, 64, focalLength, Vector3f(camPosition.x(), camPosition.y(), camPosition.z()), Vector3f(camVector.x(), camVector.y(), camVector.z()), transformation, camera);
	cout << "Camera Created" << endl;



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
	cout << "Illuminated" << endl;


	///Display results
	{
		mainViewer->addCoordinateSystem(50);
		cout << "Cube display" << endl;
		AddPointCloud(cube.cloud, "cube");
		cout << "Emitter display" << endl;
		AddPointCloud(emitter.screen, "emitter");

		cout << "World display" << endl;
		AddPointCloud(world, "world");

		ClearayVectoriewer();
	}


	Mat camImage, emitterImage;
	///Grab images
	{
		Plane camPlane(Vector3f(camPlanePoint.x(), camPlanePoint.y(), camPlanePoint.z()), camVector1);

		camImage = ToImage(world, Vector3f(camPosition.x(), camPosition.y(), camPosition.z()), camPlane, camera.fovCornersOriginal, transformation);
		Vector3f emitterPlanePoint;
		emitterPlanePoint[0] = emitter.screen->points[emitter.screen->points.size() / 2].x;
		emitterPlanePoint[1] = emitter.screen->points[emitter.screen->points.size() / 2].y;
		emitterPlanePoint[2] = emitter.screen->points[emitter.screen->points.size() / 2].z;
		emitterImage = ToImage(world, emitter.position, Plane(emitterPlanePoint, emitter.normalVectorFromSource), emitter.fovCornersOriginal, Matrix4f::Identity());
	}


	uint imgWidth = 640, imgHeight = 480;
	string windowName = "Cloud2Image";
	namedWindow(windowName, WINDOW_NORMAL);
	resizeWindow(windowName, 360 * imgWidth / imgHeight, 360);

	imshow(windowName, camImage);
	waitKey();
	imshow(windowName, emitterImage);
	waitKey();


	imwrite("left.png", camImage);
	imwrite("right.png", emitterImage);

	

	vector<Line> camLines, emLines;
	GetLinesIndices(camImage, camLines);
	GetLinesIndices(emitterImage, emLines);

	pair<int, int> p = MatchLines(emLines, camLines);

	//cout << p.first << p.second << endl;

	vector<Vector2i> points1, points2;
	if (p.first > p.second)
	{

		for (size_t i = p.first, j = p.second; i < camLines.size(); i++, j++)
		{
			for (size_t k = 0; k < camLines[i].indices.size() && k < emLines[j].indices.size(); k++)
			{
				points1.push_back(Vector2i(camLines[i].indices[k]));
				points2.push_back(Vector2i(emLines[j].indices[k]));
			}
		}
	}
	else
	{
		for (size_t i = p.first, j = p.second; j < emLines.size(); i++, j++)
		{
			for (size_t k = 0; k < camLines[i].indices.size() && k < emLines[j].indices.size(); k++)
			{
				points1.push_back(Vector2i(camLines[i].indices[k]));
				points2.push_back(Vector2i(emLines[j].indices[k]));
			}
		}
	}


	//Mat FundamentalMatrix = findFundamentalMat(points1, points2, FM_8POINT);

	cout << "done" << endl;
	//mainViewer->spin();
	getchar();
}
