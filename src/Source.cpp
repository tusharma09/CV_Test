#include "pch.h"

#include <fstream>
#include <iostream>
#include <string>



#define PointTypeLocal pcl::PointXYZRGB


using namespace std;
using namespace Eigen;




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

void ShowPointCloud(PointCloudPtr cloud, string name = "")
{
	mainViewer->addPointCloud<PointT>(cloud, name);
	mainViewer->spin();
	mainViewer->removeAllPointClouds();
	mainViewer->spinOnce();
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

#pragma endregion 


inline Vector3f PointT_To_Vector3f(PointT point)
{
	return Vector3f(point.x, point.y, point.z);
}

inline PointT Vector3f_To_PointT(Vector3f p)
{
	PointT point(255,255,255);
	point.x = p[0]; point.y = p[1]; point.z = p[2];
	return point;
}


struct Plane
{
	Vector3f point;
	Vector3f normal;
	Plane(Vector3f p, Vector3f n) : point(p), normal(n)
	{

	}
};
struct Cube
{
	PointCloudPtr cloud;
	vector<Plane> cubeFaces;
};
struct Emitter
{
	Vector3f position;
	Vector3f normalVectorFromEmitter;
	float vFOV;
	float hFOV;
	int vResolution;
	int hResolution;
	vector<vector <Vector3f>> rays;

};

float resolutionForCube = 1; ///1 mm


Plane FitPlane(Vector3f p1, Vector3f p2, Vector3f p3)
{
	float a1 = p2.x() - p1.x();
	float b1 = p2.y() - p1.y();
	float c1 = p2.z() - p2.z();
	float a2 = p3.x() - p1.x();
	float b2 = p3.y() - p1.y();
	float c2 = p3.z() - p1.z();

	float a = b1 * c2 - b2 * c1;
	float b = a2 * c1 - a1 * c2;
	float c = a1 * b2 - b1 * a2;
	float d = (-a * p1.x() - b * p1.y() - c * p1.z());

	return	Plane(Vector3f(p1.x(), p1.y(), p1.z()), Vector3f(a, b, c));
}

Vector3f GetIntersectionPointVectorAndPlane(Vector3f rayVector, Vector3f rayPoint, Vector3f planeNormal, Vector3f planePoint) 
{
	Vector3f diff = rayPoint - planePoint;
	double prod1 = diff.dot(planeNormal);
	double prod2 = rayVector.dot(planeNormal);
	double prod3 = prod1 / prod2;
	return rayPoint - rayVector * prod3;
}

inline bool IsPointOnPlane(Vector3f point, Vector3f planeNormal, Vector3f planePoint, float epsilon = 1E-10)
{
	return ((planePoint - point).dot(planeNormal) < epsilon);
}


inline int ToVecIndex(int i, int j, int maxCol)
{
	return i * maxCol + j;
}



///A Cube actually has same dimensions in all sides but question is confusing and might be referring cuboid
void CreateCube(float length, float width, float height, Matrix4f toWorld, Cube &cube)
{
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
		for (size_t i = 0; i < length; i+= resolutionForCube)
		{
			for (size_t j = 0; j < width; j += resolutionForCube)
			{
				for (size_t k = 0; k < height; k += resolutionForCube)
				{
					temp.x = i; temp.y = j; temp.z = k;
					cube.cloud->points.push_back(temp);
				}
			}
		}


		cube.cloud->width = cube.cloud->points.size();
		cube.cloud->height = 1;
	}

	pcl::transformPointCloud(*cube.cloud, *cube.cloud, toWorld);

	ShowPointCloud(cube.cloud);
}


void CreateEmitter(float verticalFOV, float horizontalFOV, int verticalResolution, int horizontalResolution, Matrix4f toWorld, Emitter &emitter)
{
	emitter.hFOV = horizontalFOV;
	emitter.vFOV = verticalFOV;
	emitter.hResolution = horizontalResolution;
	emitter.vResolution = verticalResolution;

	PointCloudPtr cloud(new PointCloudT);


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


	vector<Vector3f> vectorsFOV;///at distance 1000: if resolution comes too high, rays will be too close and may be distorted because of rounding off issue from system.
	///Find FOV vectors: top left, top right, bottom left, bottom right
	{
		Vector3f tempVector(0, 0, 1);
		AngleAxisf aa;

		///top left
		aa = AngleAxisf(emitter.vFOV / 2,Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(emitter.hFOV / 2, Vector3f(0, 1, 0));	
		tempVector = aa.toRotationMatrix() * tempVector;

		vectorsFOV.push_back(tempVector * 1000);

		///top right
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(emitter.vFOV / 2, Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(-emitter.hFOV / 2, Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		vectorsFOV.push_back(tempVector * 1000);

		///bottom left
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(-emitter.vFOV / 2, Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(emitter.hFOV / 2, Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		vectorsFOV.push_back(tempVector * 1000);

		///bottom right
		tempVector = Vector3f(0, 0, 1);

		aa = AngleAxisf(-emitter.vFOV / 2, Vector3f(1, 0, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		aa = AngleAxisf(-emitter.hFOV / 2, Vector3f(0, 1, 0));
		tempVector = aa.toRotationMatrix() * tempVector;

		vectorsFOV.push_back(tempVector * 1000);
	}


	///Create a projected ray board towards z-axis
	{
		float horizontalIncrement = abs((vectorsFOV[0].x()) - (vectorsFOV[1].x())) / emitter.hResolution;
		float verticalIncrement = abs((vectorsFOV[0].y()) - (vectorsFOV[2].y())) / emitter.vResolution;
		PointT temp(255, 255, 255);
		cloud->points.push_back(temp);///Emitter itself
		for (float i = (vectorsFOV[0].x()); i < (vectorsFOV[1].x()); i += horizontalIncrement)
		{
			for (float j = vectorsFOV[0].y(); j < vectorsFOV[2].y(); j += verticalIncrement)
			{
				temp.x = i; temp.y = j, temp.z = vectorsFOV[0].z();
				cloud->points.push_back(temp);
			}
		}

		cloud->width = cloud->points.size();
		cloud->height = 1;
	}
	
	pcl::transformPointCloud(*cloud, *cloud, toWorld);

	///Fill rays and emitter pose
	{
		emitter.position = Vector3f(cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);
		Vector3f o = PointT_To_Vector3f(cloud->points[0]);
		cloud->points.erase(cloud->points.begin());

		emitter.normalVectorFromEmitter = toWorld.block(0,0,3,3) * Vector3f(0, 0, 1);

		emitter.rays = vector<vector <Vector3f>>(emitter.hResolution, vector<Vector3f>(emitter.vResolution));
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			emitter.rays[i/ emitter.hResolution][i%emitter.hResolution] = PointT_To_Vector3f(cloud->points[i]) - o;
			emitter.rays[i / emitter.hResolution][i%emitter.hResolution].normalize();
		}
	}


}


vector<int> GetRayIntersectionsOnCube(Vector3f rayVector, Vector3f rayPoint, Cube &cube, float maxDistance=1000)
{
	float minTraverseIncrement = resolutionForCube * 10;
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
		kdtree.radiusSearch(searchPoint, minTraverseIncrement/2, pointIdxNKNSearch, pointNKNSquaredDistance);
		distance += minTraverseIncrement;

	} while ((pointIdxNKNSearch.size() == 0) && distance < maxDistance);

	if (pointIdxNKNSearch.size() > 0)
	{
		Vector3f vec(cube.cloud->points[pointIdxNKNSearch[0]].x, cube.cloud->points[pointIdxNKNSearch[0]].y, cube.cloud->points[pointIdxNKNSearch[0]].z);
		Plane face = *(find_if(cube.cubeFaces.begin(), cube.cubeFaces.end(), [vec](Plane& p) { return IsPointOnPlane(vec, p.normal, p.point); }));
		Vector3f point = GetIntersectionPointVectorAndPlane(rayVector, rayPoint, face.normal, face.point);

		kdtree.radiusSearch(Vector3f_To_PointT(point), resolutionForCube, pointIdxNKNSearch, pointNKNSquaredDistance);

		///Beam Ray affects all these points
		return pointIdxNKNSearch;
	}
	else
	{
		return vector<int>();
	}
}


void IlluminateWCSFromEmitter(Emitter &emitter, Cube &cube, float maxDistance= 100)
{
	PointCloudPtr world(new PointCloudT);
	*world = *cube.cloud;
	vector<int> indices;
	for (size_t i = 0; i < emitter.hResolution; i++)
	{
		for (size_t j = 0; j < emitter.vResolution; j++)
		{
			indices = GetRayIntersectionsOnCube(emitter.rays[i][j], emitter.position, cube);

			for (size_t j = 0; j < indices.size(); j++)
			{
				world->points[indices[j]].g = 0;
				world->points[indices[j]].b = 0;
			}
		}
	}
}



int main(int argc, char* argv[])
{
	InitialiseViewer();
	
	Cube cube;
	CreateCube(100, 100, 100, Matrix4f::Identity(), cube);

	Emitter emitter;
	//CreateEmitter(60, 70, 64, 48, Matrix4f::Identity(), emitter);





	cout << "done" << endl;
	mainViewer->spin();
	//getchar();
}
