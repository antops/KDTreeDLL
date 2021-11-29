#include "MyTimer.h"
#include <thread>
#include <vtkSTLReader.h>
#include <omp.h>
#include <vector>
#include "RAY.h"
#include "STL.h"
#include "KDTree.h"
#include "Point_3D.h"
#include "Vector3.h"

// 增加场景可视化需要的头文件
#include <vtkAutoInit.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkOutlineFilter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolygon.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkTriangleFilter.h>
#include <vtkLineSource.h>
#include <vtkDataSetMapper.h>
#include <vtkPolyLine.h>
#include <vtkTransform.h>
#include <vtkCamera.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAxesActor.h>

class _declspec(dllexport) CPURayTracing
{
public:
	CPURayTracing();
	~CPURayTracing();
	void setRay(const std::vector<Vector3>& startPiont, const std::vector<Vector3>& direction);
	void setSTL(void* _polyData);
	int run();
	void getRes(std::vector<Vector3> &nomal, std::vector<Vector3> &intersection, std::vector<bool> &isIntersect, std::vector<float> &port);
	void buildKDTree(void* _polyData);

	void setNumThread(int num);
	int getNumThread();

	int runTest(const std::vector<Vector3>& oris);
	int runTest(const Vector3& ori);
	int runByForce();
	int runByForce(const std::vector<Vector3>& oris);
	int runByForce(const Vector3& ori);
	bool runByForceHepler(const RAY& ray, Point_3D& intersection, Point_3D& triMeshNormal, Point_3D& reflectLight, float& distance, double& t);
	void generateTestRay(const std::vector<Vector3>& oris);
	void generateTestRay(const Vector3& ori);
	bool isIntersect_TRIMESH(const RAY& ray, const STL& stl, Point_3D& intersection, Point_3D& triMeshNormal, Point_3D& reflectLight, float& distance, double& t);
	void printSTLs();
	void printRays();
	int sceneVisualization(void* _polyData);

private:
	int numThread;
	int numSTL;
	std::vector<STL> stls;
	int numRay;
	std::vector<RAY> rays;

	KdTree kdtree;

	std::vector<bool> isIntersect;
	std::vector<Point_3D> intersection;
	std::vector<int> triMeshID;
	std::vector<float> distance;
	std::vector<Point_3D> triMeshNormal;
	std::vector<Point_3D> reflectLight;
};

inline CPURayTracing::CPURayTracing()
{
	numThread = std::thread::hardware_concurrency();
	numSTL = 0;
	stls.resize(0);
	numRay = 0;
	rays.resize(0);
	isIntersect.resize(0);
	intersection.resize(0);
	triMeshID.resize(0);
	distance.resize(0);
	triMeshNormal.resize(0);
	reflectLight.resize(0);
}

inline CPURayTracing::~CPURayTracing()
{
}

void CPURayTracing::setRay(const std::vector<Vector3>& startPiont, const std::vector<Vector3>& direction)
{
	this->numRay = direction.size();
	this->rays.resize(this->numRay);
	Point_3D ori, dir;
	for (int i = 0; i < this->numRay; ++i)
	{
		ori.set(startPiont[i].x, startPiont[i].y, startPiont[i].z);
		dir.set(direction[i].x, direction[i].y, direction[i].z);
		this->rays[i].setOriDir(ori, dir);
	}
	std::cout << "initial rays successfully， rays' size: " << this->numRay << " -- " << this->rays.size() << std::endl;
}

void CPURayTracing::setSTL(void * _polyData)
{
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData = (vtkPolyData*)(_polyData);
	this->numSTL = polyData->GetNumberOfCells();
	this->stls.resize(this->numSTL);
	vtkIdList* p;
	double *point;
	std::vector<int> stlsID(this->numSTL);
	for (int i = 0; i < this->numSTL; ++i)
	{
		stlsID[i] = i;
		p = polyData->GetCell(i)->GetPointIds();
		point = polyData->GetPoint(p->GetId(0));
		this->stls[i].setPoint1(Point_3D(point[0], point[1], point[2]));
		point = polyData->GetPoint(p->GetId(1));
		this->stls[i].setPoint2(Point_3D(point[0], point[1], point[2]));
		point = polyData->GetPoint(p->GetId(2));
		this->stls[i].setPoint3(Point_3D(point[0], point[1], point[2]));
	}
	std::cout << "initial stls successfully, stls' size: " << this->numSTL << " -- " << this->stls.size() << std::endl;
}

int CPURayTracing::run()
{
	//std::cout << "start ray tracing" << std::endl;
	if (this->numRay == 0) {
		std::cout << "no ray(s) initialized, please init rays first" << std::endl;
		return 1;
	}
	if (this->numSTL == 0) {
		std::cout << "no stl(s) initialized, please init trimeshes first" << std::endl;
		return 1;
	}
	if (this->kdtree.getRoot() == NULL)
	{
		std::cout << "fail to build KD_Tree" << std::endl;
		return 1;
	}
	// 开始射线追踪
	MyTimer mt;
	mt.startTimer();
	// 初始化vector
	this->isIntersect.resize(this->numRay);
	this->intersection.resize(this->numRay);
	this->triMeshID.resize(this->numRay);
	this->distance.resize(this->numRay);
	this->triMeshNormal.resize(this->numRay);
	this->reflectLight.resize(this->numRay);
	int count = 0;
	omp_set_num_threads(this->numThread);
#pragma omp parallel for reduction(+:count)
	for (int i = 0; i < this->numRay; ++i) {
		this->isIntersect[i] =
			this->kdtree.traverse(
				this->kdtree.getRoot(),
				this->rays[i],
				this->intersection[i],
				this->triMeshID[i],
				this->distance[i],
				this->triMeshNormal[i],
				this->reflectLight[i],
				this->stls);
		if (isIntersect[i]) {
			++count;
		}
	}
	mt.endTimer();
	std::cout << "threads: " << this->numThread << ", Emit " << this->numRay << " rays, hits " << count << ", misses "
		<< this->numRay - count << ", hit rate: "
		<< ((count / (double)this->numRay)) * 100 << "%" << std::endl;
	std::cout << "intersection calculate completely, it takes " << mt.getTime() << " ms" << std::endl;
	return 0;
}

inline void CPURayTracing::getRes(std::vector<Vector3>& nomal, std::vector<Vector3>& intersection, std::vector<bool>& isIntersect, std::vector<float>& port)
{
	isIntersect.assign(this->isIntersect.begin(), this->isIntersect.end());
	port.assign(this->distance.begin(), this->distance.end());
	nomal.resize(this->numRay);
	intersection.resize(this->numRay);
	for (int i = 0; i < this->numRay; ++i)
	{
		nomal[i].set(this->triMeshNormal[i].getX(), this->triMeshNormal[i].getY(), this->triMeshNormal[i].getZ());
		intersection[i].set(this->intersection[i].getX(), this->intersection[i].getY(), this->intersection[i].getZ());
	}
}

inline void CPURayTracing::buildKDTree(void * _polyData)
{
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData = (vtkPolyData*)(_polyData);
	double bounds[6];
	polyData->GetBounds(bounds);
	MyTimer mt;
	mt.startTimer();
	BBOX bbox(Point_3D(bounds[0], bounds[2], bounds[4]),Point_3D(bounds[1], bounds[3], bounds[5]));
	std::vector<int> stlsID(this->numSTL);
	for (int i = 0; i < this->numSTL; ++i)
	{
		stlsID[i] = i;
	}
	bbox.setSTLID(stlsID);
	int maxDepth = 8 + 1.3 * log10(this->numSTL);
	/*std::cout << "max tree limit depth: " << maxDepth + 2 << std::endl;
	std::cout << "input num to choose subBox model: 0 - xy orderly, 1 - the longest axis" << std::endl;*/
	int op = 1;
	//cin >> op;
	this->kdtree.setSubBoxModel(op);
	this->kdtree.setMaxDepth(maxDepth);
	this->kdtree.build_kdtree(bbox, this->kdtree.getRoot(), 0, this->stls);
	mt.endTimer();
	std::cout << "succeed, build kdtree time: " << mt.getTime() << " ms, kdtree nodes: "
		<< this->kdtree.getNumNodes(this->kdtree.getRoot()) << ", max depth: "
		<< this->kdtree.getMaxDepth(this->kdtree.getRoot()) << std::endl;
}

void CPURayTracing::setNumThread(int num)
{
	this->numThread = num;
}

int CPURayTracing::getNumThread()
{
	return this->numThread;
}

int CPURayTracing::runTest(const std::vector<Vector3>& oris)
{
	this->generateTestRay(oris);
	std::cout << "start ray tracing" << std::endl;
	if (this->rays.size() == 0) {
		std::cout << "no ray(s) initialized, please init rays first" << std::endl;
		return 1;
	}
	if (this->stls.size() == 0) {
		std::cout << "no stl(s) initialized, please init trimeshes first" << std::endl;
		return 1;
	}
	if (this->kdtree.getRoot() == NULL)
	{
		std::cout << "fail to build KD_Tree" << std::endl;
		return 1;
	}
	// 开始射线追踪
	MyTimer mt;
	mt.startTimer();
	// 初始化vector
	this->isIntersect.resize(this->numRay);
	this->intersection.resize(this->numRay);
	this->triMeshID.resize(this->numRay);
	this->distance.resize(this->numRay);
	this->triMeshNormal.resize(this->numRay);
	this->reflectLight.resize(this->numRay);
	int count = 0;
	omp_set_num_threads(this->numThread);
#pragma omp parallel for reduction(+:count)
	for (int i = 0; i < this->numRay; ++i) {
		this->isIntersect[i] =
			this->kdtree.traverse(
				this->kdtree.getRoot(),
				this->rays[i],
				this->intersection[i],
				this->triMeshID[i],
				this->distance[i],
				this->triMeshNormal[i],
				this->reflectLight[i],
				this->stls);
		if (isIntersect[i]) {
			++count;
		}
	}
	mt.endTimer();
	std::cout << "Emit " << this->numRay << " rays, hits " << count << ", misses "
		<< this->numRay - count << ", hit rate: "
		<< ((count / (double)this->numRay)) * 100 << "%" << std::endl;
	std::cout << "intersection calculate completely, it takes " << mt.getTime() << " ms" << std::endl;
	return 0;
}

inline int CPURayTracing::runTest(const Vector3 & ori)
{
	this->generateTestRay(ori);
	std::cout << "start ray tracing" << std::endl;
	if (this->rays.size() == 0) {
		std::cout << "no ray(s) initialized, please init rays first" << std::endl;
		return 1;
	}
	if (this->stls.size() == 0) {
		std::cout << "no stl(s) initialized, please init trimeshes first" << std::endl;
		return 1;
	}
	if (this->kdtree.getRoot() == NULL)
	{
		std::cout << "fail to build KD_Tree" << std::endl;
		return 1;
	}
	// 开始射线追踪
	MyTimer mt;
	mt.startTimer();
	// 初始化vector
	this->isIntersect.resize(this->numRay);
	this->intersection.resize(this->numRay);
	this->triMeshID.resize(this->numRay);
	this->distance.resize(this->numRay);
	this->triMeshNormal.resize(this->numRay);
	this->reflectLight.resize(this->numRay);
	int count = 0;
	omp_set_num_threads(this->numThread);
#pragma omp parallel for reduction(+:count)
	for (int i = 0; i < this->numRay; ++i) {
		this->isIntersect[i] =
			this->kdtree.traverse(
				this->kdtree.getRoot(),
				this->rays[i],
				this->intersection[i],
				this->triMeshID[i],
				this->distance[i],
				this->triMeshNormal[i],
				this->reflectLight[i],
				this->stls);
		if (isIntersect[i]) {
			++count;
		}
	}
	mt.endTimer();
	std::cout << "Emit " << this->numRay << " rays, hits " << count << ", misses "
		<< this->numRay - count << ", hit rate: "
		<< ((count / (double)this->numRay)) * 100 << "%" << std::endl;
	std::cout << "intersection calculate completely, it takes " << mt.getTime() << " ms" << std::endl;
	return 0;
}

inline int CPURayTracing::runByForce()
{ 
	std::cout << "start ray tracing by force" << std::endl;
	// 初始化vector
	this->isIntersect.resize(this->numRay);
	this->intersection.resize(this->numRay);
	this->triMeshID.resize(this->numRay);
	this->distance.resize(this->numRay);
	this->triMeshNormal.resize(this->numRay);
	this->reflectLight.resize(this->numRay);
	int count = 0;
	MyTimer mt;
	mt.startTimer();
#pragma omp parallel for reduction(+:count)
	for (int i = 0; i < this->numRay; ++i)
	{
		double t = 0;
		this->isIntersect[i] = runByForceHepler(this->rays[i], this->intersection[i], this->triMeshNormal[i], this->reflectLight[i], this->distance[i], t);
		if (isIntersect[i]) {
			++count;
		}
	}
	mt.endTimer();
	std::cout << "Emit " << this->numRay << " rays, hits " << count << ", misses "
		<< this->numRay - count << ", hit rate: "
		<< ((count / (double)this->numRay)) * 100 << "%" << std::endl;
	std::cout << "intersection calculate completely, it takes " << mt.getTime() << " ms" << std::endl;
	return 0;
}

inline int CPURayTracing::runByForce(const std::vector<Vector3>& oris)
{
	this->generateTestRay(oris);
	std::cout << "start ray tracing by force" << std::endl;
	// 初始化vector
	this->isIntersect.resize(this->numRay);
	this->intersection.resize(this->numRay);
	this->triMeshID.resize(this->numRay);
	this->distance.resize(this->numRay);
	this->triMeshNormal.resize(this->numRay);
	this->reflectLight.resize(this->numRay);
	int count = 0;
	double t = 0;
	MyTimer mt;
	mt.startTimer();
#pragma omp parallel for reduction(+:count)
	for (int i = 0; i < this->numRay; ++i)
	{
		double t = 0;
		this->isIntersect[i] = runByForceHepler(this->rays[i], this->intersection[i], this->triMeshNormal[i], this->reflectLight[i], this->distance[i], t);
		if (isIntersect[i]) {
			++count;
		}
	}
	mt.endTimer();
	std::cout << "Emit " << this->numRay << " rays, hits " << count << ", misses "
		<< this->numRay - count << ", hit rate: "
		<< ((count / (double)this->numRay)) * 100 << "%" << std::endl;
	std::cout << "intersection calculate completely, it takes " << mt.getTime() << " ms" << std::endl;
	return 0;
}

inline int CPURayTracing::runByForce(const Vector3 & ori)
{
	this->generateTestRay(ori);
	std::cout << "start ray tracing by force" << std::endl;
	// 初始化vector
	this->isIntersect.resize(this->numRay);
	this->intersection.resize(this->numRay);
	this->triMeshID.resize(this->numRay);
	this->distance.resize(this->numRay);
	this->triMeshNormal.resize(this->numRay);
	this->reflectLight.resize(this->numRay);
	int count = 0;
	MyTimer mt;
	mt.startTimer();
#pragma omp parallel for reduction(+:count)
	for (int i = 0; i < this->numRay; ++i)
	{
		double t = 0;
		this->isIntersect[i] = runByForceHepler(this->rays[i], this->intersection[i], this->triMeshNormal[i], this->reflectLight[i], this->distance[i], t);
		if (isIntersect[i]) {
			++count;
		}
	}
	mt.endTimer();
	std::cout << "Emit " << this->numRay << " rays, hits " << count << ", misses "
		<< this->numRay - count << ", hit rate: "
		<< ((count / (double)this->numRay)) * 100 << "%" << std::endl;
	std::cout << "intersection calculate completely, it takes " << mt.getTime() << " ms" << std::endl;
	return 0;
}

inline bool CPURayTracing::runByForceHepler(const RAY & ray, Point_3D & intersection, Point_3D & triMeshNormal, Point_3D & reflectLight, float & distance, double & t)
{
	for (int j = 0; j < this->numSTL; ++j)
	{
		if (isIntersect_TRIMESH(ray, this->stls[j], intersection, triMeshNormal, reflectLight, distance, t))
		{
			return true;
		}
	}
	return false;
}

void CPURayTracing::generateTestRay(const std::vector<Vector3>& oris)
{
	if (this->numSTL == 0)
	{
		std::cout << "generate test rays by trimeshes' center need to load trimeshes first" << std::endl;
		return;
	}
	this->numRay = this->numSTL;
	this->rays.resize(this->numRay);
	for (int i = 0; i < this->numSTL; ++i) {
		STL stl = this->stls[i];
		Point_3D point1, point2, point3;
		this->stls[i].getPoint(point1, point2, point3);
		this->rays[i] = (
			RAY(
				Point_3D(oris[i].x, oris[i].y, oris[i].z),
				Point_3D(
				(point1.getX() + point2.getX() + point3.getX()) / 3 - oris[i].x,
					(point1.getY() + point2.getY() + point3.getY()) / 3 - oris[i].y,
					(point1.getZ() + point2.getZ() + point3.getZ()) / 3 - oris[i].z
				))
		);
	}
	std::cout << "generate test rays by trimeshes' center with start points successfully" << std::endl;
}

inline void CPURayTracing::generateTestRay(const Vector3 & ori)
{
	if (this->numSTL == 0)
	{
		std::cout << "generate test rays by trimeshes' center need to load trimeshes first" << std::endl;
		return;
	}
	this->numRay = this->numSTL;
	this->rays.resize(this->numRay);
	for (int i = 0; i < this->numSTL; ++i) {
		STL stl = this->stls[i];
		Point_3D point1, point2, point3;
		this->stls[i].getPoint(point1, point2, point3);
		this->rays[i] = (
			RAY(
				Point_3D(ori.x, ori.y, ori.z),
				Point_3D(
				(point1.getX() + point2.getX() + point3.getX()) / 3 - ori.x,
					(point1.getY() + point2.getY() + point3.getY()) / 3 - ori.y,
					(point1.getZ() + point2.getZ() + point3.getZ()) / 3 - ori.z
				))
			);
	}
	std::cout << "generate rays by defined ori and stls' center successfully" << std::endl;
}

inline bool CPURayTracing::isIntersect_TRIMESH(const RAY & ray, const STL & stl, Point_3D & intersection, Point_3D & triMeshNormal, Point_3D & reflectLight, float & distance, double & t)
{
	Point_3D point1, point2, point3;
	stl.getPoint(point1, point2, point3);
	Point_3D ori, dir;
	ray.getOriDir(ori, dir);
	double u, v;
	// E1
	Point_3D E1 = point2 - point1;
	// E2
	Point_3D E2 = point3 - point1;
	// P
	// Point_3D P = ray.dir.Cross(E2);
	Point_3D P = dir.Cross(E2);

	// determinant
	double det = E1.Dot(P);

	Point_3D T;
	//T = ray.ori - triMesh.point1;
	T = ori - point1;

	// If determinant is near zero, ray lies in plane of triangle
	//if (det < 0.00000001 && det > -0.00000001)
	//	return false;
	// Calculate u and make sure u <= 1
	u = T.Dot(P);
	double fInvDet = 1.0f / det;
	u *= fInvDet;
	if (u < 0.0 || u > 1)
		return false;
	// Q
	Point_3D Q = T.Cross(E1);
	// Calculate v and make sure u + v <= 1
	v = dir.Dot(Q);
	v *= fInvDet;
	if (v < 0.0 || u + v > 1)
		return false;
	// Calculate t, scale parameters, ray intersects triangle
	t = E2.Dot(Q);
	t *= fInvDet;
	// 交点
	intersection = ori + dir * t;
	return true;
	/*
	// 计算交点到射线源点的距离
	double ix, iy, iz;
	ix = intersection.getX();
	iy = intersection.getY();
	iz = intersection.getZ();
	double ox, oy, oz;
	ox = ori.getX();
	oy = ori.getY();
	oz = ori.getZ();
	distance = pow(pow(ix - ox, 2) + pow(iy - oy, 2) + pow(iz - oz, 2), 0.5);
	// Calculate stls normal
	Point_3D tempa = point1 - point2;
	Point_3D tempb = point1 - point3;
	triMeshNormal = tempa.Cross(tempb);  //法向量
										 // Calculate reflect light
	if (dir.Dot(triMeshNormal) > 0)
		triMeshNormal = Point_3D(0, 0, 0) - triMeshNormal;
	//先单位化
	double absa = pow(dir.Dot(dir), 0.5);
	double absn = pow(triMeshNormal.Dot(triMeshNormal), 0.5);
	tempa = dir * (1 / absa);
	tempb = triMeshNormal * (1 / absn);
	double I = 2 * tempb.Dot(tempa);
	if (I < 0) {
		I = -I;
	}
	else {
		tempa = Point_3D(0.0, 0.0, 0.0) - tempa;
	}
	reflectLight = tempb * I + tempa;
	return true;
	*/
}

void CPURayTracing::printSTLs()
{
	for (int i = 0; i < this->numSTL; ++i)
	{
		if (i % 100 == 0)
		{
			std::cout << "output stls' points Inf: y to continue or n to break" << std::endl;
			char c = getchar();
			switch (c)
			{
			case 'n':
			case 'N':
				return;
			default:
				break;
			}
		}
		std::cout << "Location of the " << i << "th stl: ";
		this->stls[i].print();
	}
}

void CPURayTracing::printRays()
{
	for (int i = 0; i < this->numRay; ++i)
	{
		if (i % 100 == 0)
		{
			std::cout << "output rays' points Inf: y to continue or n to break" << std::endl;
			char c = getchar();
			switch (c)
			{
			case 'n':
			case 'N':
				return;
			default:
				break;
			}
		}
		std::cout << "Location of the " << i << "th ray: ";
		this->rays[i].print();
	}
}

int CPURayTracing::sceneVisualization(void* _polyData)
{
	vtkSmartPointer<vtkPolyData> reflectorpolyData = vtkSmartPointer<vtkPolyData>::New();
	reflectorpolyData = (vtkPolyData*)(_polyData);
	if (this->rays.size() == 0)
	{
		std::cout << "no incident ray" << endl;
		return 1;
	}
	if (this->reflectLight.size() == 0)
	{
		std::cout << "no reflect rays" << endl;
		return 1;
	}
	if (this->stls.size() == 0)
	{
		std::cout << "no trimeshes" << endl;
		return 1;
	}
	// 基于三维空间中两点画直线
	vtkPoints* points = vtkPoints::New();
	vtkSmartPointer<vtkPolyLine> polyLine =
		vtkSmartPointer<vtkPolyLine>::New();
	vtkSmartPointer<vtkCellArray> cells =
		vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyData> polyData =
		vtkSmartPointer<vtkPolyData>::New();
	int id = 0;
	// 入射射线
	Point_3D ori, dir;
	for (int i = 0; i < this->isIntersect.size(); ++i)
	{
		if (this->isIntersect[i])
		{
			this->rays[i].getOriDir(ori, dir);
			points->InsertNextPoint(ori.getX(), ori.getY(), ori.getZ());
			//points->InsertNextPoint(this->intersection[i].getX(), this->intersection[i].getY(), this->intersection[i].getZ());
			points->InsertNextPoint(ori.getX() + dir.getX(), ori.getY() + dir.getY(), ori.getZ() + dir.getZ());
			polyLine->GetPointIds()->SetNumberOfIds(2);
			polyLine->GetPointIds()->SetId(0, id++);
			polyLine->GetPointIds()->SetId(1, id++);
			cells->InsertNextCell(polyLine);
		}
		else {
			/*std::cout << "判交失败的射线：ray " << i << ", fail at depth:" << this->triMeshID[i] << std::endl;
			this->rays[i].print();*/
		}
	}
	// 反射射线
	/*for (int i = 0; i < this->isIntersect.size(); ++i)
	{
		if (this->isIntersect[i])
		{
			points->InsertNextPoint(this->intersection[i].getX(), this->intersection[i].getY(), this->intersection[i].getZ());
			points->InsertNextPoint(this->reflectLight[i].getX(), this->reflectLight[i].getY(), this->reflectLight[i].getZ());
			polyLine->GetPointIds()->SetNumberOfIds(2);
			polyLine->GetPointIds()->SetId(0, id++);
			polyLine->GetPointIds()->SetId(1, id++);
			cells->InsertNextCell(polyLine);
		}
	}*/
	polyData->SetPoints(points);
	polyData->SetLines(cells);
	vtkSmartPointer<vtkPolyDataMapper> reflectLightMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	reflectLightMapper->SetInputData(polyData);

	vtkSmartPointer<vtkActor> reflectLight =
		vtkSmartPointer<vtkActor>::New();
	reflectLight->SetMapper(reflectLightMapper);
	reflectLight->GetProperty()->SetColor(1, 0, 0);
	// 坐标轴
	vtkSmartPointer<vtkAxesActor> axis =
		vtkSmartPointer<vtkAxesActor>::New();
	axis->SetPosition(0, 0, 0);
	axis->SetTotalLength(0.05, 0.05, 0.05);
	axis->SetShaftType(0);
	axis->SetAxisLabels(1);
	axis->SetCylinderRadius(0.001);
	// 反射面
	vtkSmartPointer<vtkPolyDataMapper> reflectorMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	reflectorMapper->SetInputData(reflectorpolyData);
	vtkSmartPointer<vtkActor> reflector =
		vtkSmartPointer<vtkActor>::New();
	reflector->SetMapper(reflectorMapper);
	reflector->GetProperty()->SetColor(0.2, 0.3, 0.4);
	// 设置网格显示
	reflector->GetProperty()->SetRepresentationToWireframe();
	// 设置点显示
	// actor->GetProperty()->SetRepresentationToPoints();
	// 设置面显示
	// actor->GetProperty()->SetRepresentationToSurface();
	// 相机视角
	vtkCamera* aCamera = vtkCamera::New();
	aCamera->SetViewUp(0, 0, 1);//设视角位置
	aCamera->SetPosition(0, -3 * 0.8, 0);//设观察对象位
	aCamera->SetFocalPoint(0, 0, 0);//设焦点
	aCamera->ComputeViewPlaneNormal();//自动
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(reflectLight);
	renderer->AddActor(reflector);
	renderer->AddActor(axis);
	renderer->SetBackground(1, 1, 1);
	renderer->SetActiveCamera(aCamera);
	vtkSmartPointer<vtkRenderWindow> rw =
		vtkSmartPointer<vtkRenderWindow>::New();
	rw->AddRenderer(renderer);
	rw->SetSize(800, 600);;
	rw->SetWindowName("STL pro");
	rw->Render();
	vtkSmartPointer<vtkRenderWindowInteractor> rwi =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	rwi->SetRenderWindow(rw);
	rwi->Start();
	return 0;
}
