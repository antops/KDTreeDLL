#pragma once
#include <omp.h>
#include <iostream>
#include <algorithm>
#include "RAY.h"
#include "AABB_BOX.h"

typedef AABB_BOX BBOX;

struct KdTreeNode
{
	BBOX bbox;
	// Point_3D m_Min, m_Max;
	int depth;
	//std::vector<int> data; // inner node: data's size is 1 which indicates the node's depth, left node: store the id of stls which are surrounded in the left node's box;
	KdTreeNode* left;
	KdTreeNode* right;
	KdTreeNode()
	{
		depth = 0;
		this->left = NULL;
		this->right = NULL;
		//data.resize(1);
	}
};

class KdTree
{
public:
	KdTree();
	KdTree(int numSTL);
	~KdTree();
	void setMaxDepth(int depth);
	int getMaxDepth();
	void setSubBoxModel(int model);
	int getSubBoxModel();
	// current invalid builde k-d tree method
	// KdTreeNode* build_kdtree(KdTreeNode* T, const Point_3D& m_Min, const Point_3D m_Max, int depth, const std::vector<STL>& stls);
	KdTreeNode* build_kdtree(const BBOX& BBOX, KdTreeNode* T, int depth, const std::vector<STL>& trimesh);
	KdTreeNode* getRoot();
	bool traverse(KdTreeNode* const T, const RAY& ray, Point_3D& intersect, int& ID, float& distance, Point_3D& triMeshNormal, Point_3D& reflectLight, const std::vector<STL>& stls);
	bool isPointInBBox(const Point_3D& point, const Point_3D& m_Min, const Point_3D& m_Max);
	bool isIntersect_BBOX(const RAY& ray, const Point_3D& m_Min, const Point_3D& m_Max);
	bool isIntersect_TRIMESH(const RAY& ray, const STL& stl, Point_3D& intersection, Point_3D& triMeshNormal, Point_3D& reflectLight, float& distance, double& t);
	int getMaxDepth(KdTreeNode* const T);
	int getNumNodes(KdTreeNode* const T);

private:
	KdTreeNode* root;
	int maxDepth;
	int subBoxModel;
};

inline KdTree::KdTree()
{
	this->root = NULL;
	this->maxDepth = 25;
	this->subBoxModel = 0;
}

inline KdTree::KdTree(int numSTL)
{
	this->root = NULL;
	this->maxDepth = 8 + 1.3 * log10(numSTL);
	this->subBoxModel = 0;
}

inline KdTree::~KdTree()
{
}

inline void KdTree::setMaxDepth(int depth)
{
	this->maxDepth = depth;
}

inline int KdTree::getMaxDepth()
{
	return this->maxDepth;
}

inline void KdTree::setSubBoxModel(int model)
{
	this->subBoxModel = model;
}

inline int KdTree::getSubBoxModel()
{
	return this->subBoxModel;
}

//inline KdTreeNode * KdTree::build_kdtree(KdTreeNode * T, const Point_3D & m_Min, const Point_3D m_Max, int depth, const std::vector<STL>& stls)
//{
//	//ofstream write("./Record/KDTreeNodeInfo.txt", ios::app);//打开文件，以ios::app追加的方式输入
//	//if (depth == 0)
//	//{
//	//	fstream file("./Record/KDTreeNodeInfo.txt", ios::out);//清空文件
//	//}
//	//if (!write && depth == 0)
//	//{
//	//	std::cout << "KDTreeNodeInfo.txt open fail" << std::endl;
//	//}
//	//write << "depth: " << depth << " " << ", bbox m_Min: [" << m_Min.getX() << " " << m_Min.getY() << " " << m_Min.getZ() << "], m_Max: ["
//	//	<< m_Max.getX() << " " << m_Max.getY() << " " << m_Max.getZ() << "], stlsID's size: " << stlsID.size() << ", stls.size: " << stls.size() << endl;
//	//write.close();//关闭文件
//	if (depth >= maxDepth) {
//		T = new KdTreeNode();
//		T->m_Min = m_Min;
//		T->m_Max = m_Max;
//		T->left = NULL;
//		T->right = NULL;
//		Point_3D p1, p2, p3;
//		std::vector<int> stlsID;
//		for (int i = 0; i < stls.size(); ++i)
//		{
//			stls[i].getPoint(p1, p2, p3);
//			if (isPointInBBox(p1,T->m_Min,T->m_Max) || isPointInBBox(p2, T->m_Min, T->m_Max)|| isPointInBBox(p3, T->m_Min, T->m_Max))
//			{
//				stlsID.push_back(i);
//			}
//		}
//		int size = stlsID.size();
//		T->data.swap(stlsID);
//		/*std::cout << "left node, depth: " << depth << ", stlsID's size: " << size << " - " << T->data.size() << ", bbox: [[" << T->m_Min.getX() << " " << T->m_Min.getY() << " " << T->m_Min.getZ() << "], ["
//			<< T->m_Max.getX() << " " << T->m_Max.getY() << " " << T->m_Max.getZ() << "]]" << ", stls' size: " << stls.size() << std::endl;*/
//		return T;
//	}
//	T = new KdTreeNode();
//	T->m_Min = m_Min;
//	T->m_Max = m_Max;
//	T->data[0] = depth;
//	/*if (depth == 0)
//	{
//		Point_3D p1, p2, p3;
//		std::vector<int> stlsID;
//		for (int i = 0; i < stls.size(); ++i)
//		{
//			stls[i].getPoint(p1, p2, p3);
//			if (isPointInBBox(p1, T->m_Min, T->m_Max) || isPointInBBox(p2, T->m_Min, T->m_Max) || isPointInBBox(p3, T->m_Min, T->m_Max))
//			{
//				stlsID.push_back(i);
//			}
//		}
//		std::cout << "init bbox has id size: " << stlsID.size() << " , stls's size: " << stls.size() << std::endl;
//	}*/
//	// 划分节点
//	double minX = m_Min.getX(), minY = m_Min.getY(), minZ = m_Min.getZ();
//	double maxX = m_Max.getX(), maxY = m_Max.getY(), maxZ = m_Max.getZ();
//	double subX = maxX - minX, subY = maxY - minY, subZ = maxZ - minZ;
//	int split = subZ > subY ? subZ > subX ? 2 : 0 : subY > subX ? 1 : 0;
//	Point_3D m_Min1, m_Max1;
//	Point_3D m_Min2, m_Max2;
//	double mid = 0;
//	switch (split)
//	{
//	case 0:
//		mid = minX + subX / 2;
//		m_Min1.set(minX, minY, minZ);
//		m_Max1.set(mid, maxY, maxZ);
//		m_Min2.set(mid, minY, minZ);
//		m_Max2.set(maxX, maxY, maxZ);
//		break;
//	case 1:
//		mid = minY + subY / 2;
//		m_Min1.set(minX, minY, minZ);
//		m_Max1.set(maxX, mid, maxZ);
//		m_Min2.set(minX, mid, minZ);
//		m_Max2.set(maxX, maxY, maxZ);
//		break;
//	case 2:
//		mid = minZ + subZ / 2;
//		m_Min1.set(minX, minY, minZ);
//		m_Max1.set(mid, maxY, mid);
//		m_Min2.set(mid, minY, mid);
//		m_Max2.set(maxX, maxY, maxZ);
//		break;
//	default:
//		mid = minX + subX / 2;
//		m_Min1.set(minX, minY, minZ);
//		m_Max1.set(mid, maxY, maxZ);
//		m_Min2.set(mid, minY, minZ);
//		m_Max2.set(maxX, maxY, maxZ);
//		break;
//	}
//	++depth;
//	T->left = build_kdtree(T->left, m_Min1, m_Max1, depth, stls, maxDepth);
//	T->right = build_kdtree(T->right, m_Min2, m_Max2, depth, stls, maxDepth);
//	this->root = T;
//	return T;
//}

KdTreeNode* KdTree::build_kdtree(const BBOX& bbox, KdTreeNode* T, int depth, const std::vector<STL>& stls)
{
	int size = bbox.getSTLIDSize();
	if (depth >= this->maxDepth || size <= 128) {
		if (size > 0)
		{
			T = new KdTreeNode();
			T->depth = depth;
			T->bbox = bbox;
			T->left = NULL;
			T->right = NULL;
			return T;
		}
		return NULL;
	}
	T = new KdTreeNode();
	T->depth = depth;
	T->bbox = bbox;
	BBOX bbox1, bbox2;
	switch (this->subBoxModel)
	{
	case 0:
		bbox.subBBox(bbox1, bbox2, stls);
	case 1:
		bbox.subBBox_V2(bbox1, bbox2, stls);
	default:
		bbox.subBBox(bbox1, bbox2, stls);
		break;
	}
#pragma omp parallel sections//并行区域
	{
#pragma omp section //负责这个区域的线程生成左子树
		T->left = build_kdtree(bbox1, T->left, depth + 1, stls);
#pragma omp section //负责这个区域的线程生成右子树
		T->right = build_kdtree(bbox2, T->right, depth + 1, stls);
	}
	this->root = T;
	return T;
}

inline KdTreeNode* KdTree::getRoot()
{
	return this->root;
}

inline bool KdTree::traverse(KdTreeNode* const T, const RAY& ray, Point_3D& intersect, int& ID, float& distance, Point_3D& triMeshNormal, Point_3D& reflectLight, const std::vector<STL>& stls)
{
	if (T == nullptr) {
		return false;
	}
	// 已达到叶子节点
	Point_3D m_Min, m_Max;
	T->bbox.getBounds(m_Min, m_Max);
	if (T->left == nullptr && T->right == nullptr) {
		// 射线与叶子节点的包围盒不相交
		if (!isIntersect_BBOX(ray, m_Min, m_Max))
		{
			return false;
		}
		// 遍历叶节点的三角面片，判断是否有交点
		std::vector<int> triMeshID;
		T->bbox.getSTLID(triMeshID);
		// #pragma omp parallel for reduction(+:isInter)
		for (int i = 0; i < triMeshID.size(); ++i)
		{
			double t;
			if (isIntersect_TRIMESH(ray, stls[triMeshID[i]], intersect, triMeshNormal, reflectLight, distance, t))
			{
				ID = triMeshID[i];
				return true;
			}
		}
		return false;
	}
	// 与当前树结点的包围盒相交，没有到达叶子结点，进入子结点继续判交
	if (isIntersect_BBOX(ray, m_Min, m_Max))
	{
		return traverse(T->left, ray, intersect, ID, distance, triMeshNormal, reflectLight, stls) ||
			traverse(T->right, ray, intersect, ID, distance, triMeshNormal, reflectLight, stls);
	}
	ID = T->depth;
	return false;
}

//inline bool KdTree::traverse(KdTreeNode* const T, const RAY& ray, Point_3D& intersect, int& ID, float& distance, Point_3D& triMeshNormal, Point_3D& reflectLight, const std::vector<STL>& stls)
//{
//	if (T == nullptr) {
//		return false;
//	}
//	// 已达到叶子节点
//	if (T->left == nullptr && T->right == nullptr) {
//		// 射线与叶子节点的包围盒不相交
//		if (!isIntersect_BBOX(ray, T->m_Min, T->m_Max))
//		{
//			return false;
//		}
//		// 遍历叶节点的三角面片，判断是否有交点
//		for (int i = 0; i < T->data.size(); ++i)
//		{
//			double t = 0;
//			if (isIntersect_TRIMESH(ray, stls[T->data[i]], intersect, triMeshNormal, reflectLight, distance, t))
//			{
//				ID = T->data[i];
//				return true;
//			}
//		}
//		return false;
//	}
//	// 与当前树结点的包围盒相交，没有到达叶子结点，进入子结点继续判交
//	if (isIntersect_BBOX(ray, T->m_Min, T->m_Max))
//	{
//		return traverse(T->left, ray, intersect, ID, distance, triMeshNormal, reflectLight, stls) ||
//			traverse(T->right, ray, intersect, ID, distance, triMeshNormal, reflectLight, stls);
//	}
//	ID = T->data[0];
//	return false;
//}

inline bool KdTree::isPointInBBox(const Point_3D & point, const Point_3D & m_Min, const Point_3D & m_Max)
{
	double delta = 0.0001;
	if (point.getX() > m_Max.getX() + delta || point.getX() < m_Min.getX() - delta)
	{
		return false;
	}
	if (point.getY() > m_Max.getY() + delta || point.getY() < m_Min.getY() - delta)
	{
		return false;
	}
	if (point.getZ() > m_Max.getZ() + delta || point.getZ() < m_Min.getZ() - delta)
	{
		return false;
	}
	return true;
}

inline bool KdTree::isIntersect_BBOX(const RAY & ray, const Point_3D & m_Min, const Point_3D & m_Max)
{
	Point_3D ori, dir;
	ray.getOriDir(ori, dir);
	double ox = ori.getX(), oy = ori.getY(), oz = ori.getZ();
	double dx = dir.getX(), dy = dir.getY(), dz = dir.getZ();
	double tx_min = 0, ty_min = 0, tz_min = 0;
	double tx_max = 0, ty_max = 0, tz_max = 0;
	//x0,y0,z0为包围体的最小顶点
	//x1,y1,z1为包围体的最大顶点
	double x0 = m_Min.getX(), y0 = m_Min.getY(), z0 = m_Min.getZ();
	double x1 = m_Max.getX(), y1 = m_Max.getY(), z1 = m_Max.getZ();
	if (abs(dx) < 0.0f)
	{
		//若射线方向矢量的x轴分量为0且原点不在盒体内
		if (ox < x1 || ox > x0)
		{
			return false;
		}
	}
	else
	{
		if (dx >= 0)
		{
			tx_min = (x0 - ox) / dx;
			tx_max = (x1 - ox) / dx;
		}
		else {
			tx_min = (x1 - ox) / dx;
			tx_max = (x0 - ox) / dx;
		}
	}
	if (abs(dy) < 0.0f)
	{
		//若射线方向矢量的x轴分量为0且原点不在盒体内
		if (oy < y1 || oy > y0)
		{
			return false;
		}
	}
	else {
		if (dy >= 0)
		{
			ty_min = (y0 - oy) / dy;
			ty_max = (y1 - oy) / dy;
		}
		else {
			ty_min = (y1 - oy) / dy;
			ty_max = (y0 - oy) / dy;
		}
	}
	if (abs(dz) < 0.0f) {
		//若射线方向矢量的x轴分量为0且原点不在盒体内
		if (oz < z1 || oz > z0)
		{
			return false;
		}
	}
	else {
		if (dz >= 0)
		{
			tz_min = (z0 - oz) / dz;
			tz_max = (z1 - oz) / dz;
		}
		else {
			tz_min = (z1 - oz) / dz;
			tz_max = (z0 - oz) / dz;
		}
	}
	double t0, t1;
	//光线进入平面处（最靠近的平面）的最大t值
	// t0 = std::max(tx_min, ty_min);
	// t0 = std::max(tz_min, t0);
	t0 = tx_min > ty_min ? (tx_min > tz_min ? tx_min : tz_min) : (ty_min > tz_min ? ty_min : tz_min);
	//光线离开平面处（最远离的平面）的最小t值
	// t1 = std::min(tx_max, ty_max);
	// t1 = std::min(tz_max, t1);
	t1 = tx_max < ty_max ? (tx_max < tz_max ? tx_max : tz_max) : (ty_max < tz_max ? ty_max : tz_max);
	return t0 < t1;
}

inline bool KdTree::isIntersect_TRIMESH(const RAY& ray, const STL& stl, Point_3D& intersection, Point_3D& triMeshNormal, Point_3D& reflectLight, float& distance, double& t)
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
	// 计算交点到射线源点的距离
	double ix, iy, iz;
	ix = intersection.getX();
	iy = intersection.getY();
	iz = intersection.getZ();
	double ox, oy, oz;
	ox = ori.getX();
	oy = ori.getY();
	oz = ori.getZ();
	//distance = pow(pow(ix - ox, 2) + pow(iy - oy, 2) + pow(iz - oz, 2), 0.5);
	distance = t;
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
}

inline int KdTree::getMaxDepth(KdTreeNode* const T)
{
	if (T == nullptr) {
		return 0;
	}
	int left = getMaxDepth(T->left);
	int right = getMaxDepth(T->right);
	return (left > right ? left : right) + 1;
}

inline int KdTree::getNumNodes(KdTreeNode* const T)
{
	return T == nullptr ? 0 : 1 + getNumNodes(T->left) + getNumNodes(T->right);
}