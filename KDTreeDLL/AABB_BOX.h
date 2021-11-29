#pragma once
// include head files
#include <vector>
#include "Point_3D.h"
#include "STL.h"

class AABB_BOX
{
public:
	AABB_BOX();
	AABB_BOX(Point_3D _Bmin, Point_3D _Bmax);
	~AABB_BOX();
	void setSplit(int val);
	int getSplit() const;
	void setSTLID(std::vector<int>& triMeshID);
	void getSTLID(std::vector<int>& triMeshID) const;
	int getSTLIDSize() const;
	void setBounds(const Point_3D& tm_Min, const Point_3D& tm_Max);
	void getBounds(Point_3D& m_Min, Point_3D& m_Max);
	bool IsPointInBBox(const Point_3D& point) const;
	bool IsPointInBBox_V2(const Point_3D& point) const;
	void subBBox(AABB_BOX& bbox1, AABB_BOX& bbox2, const std::vector<STL>& stls) const;
	void subBBox_V2(AABB_BOX& bbox1, AABB_BOX& bbox2, const std::vector<STL>& stls) const;
	AABB_BOX& operator=(const AABB_BOX& bbox)
	{
		if (this != &bbox) {
			this->split = bbox.split;
			this->stlsID.assign(bbox.stlsID.begin(), bbox.stlsID.end());
			this->m_Min = bbox.m_Min;
			this->m_Max = bbox.m_Max;
		}
		return *this;
	}
	void print() const;

private:
	int split;
	std::vector<int> stlsID;
	Point_3D m_Min;
	Point_3D m_Max;
};

AABB_BOX::AABB_BOX()
{
	this->split = 0;
	this->stlsID.resize(0);
}

AABB_BOX::AABB_BOX(Point_3D _Bmin, Point_3D _Bmax)
{
	this->m_Min = _Bmin;
	this->m_Max = _Bmax;
}

AABB_BOX::~AABB_BOX()
{
}

void AABB_BOX::setSplit(int split)
{
	this->split = split;
}

int AABB_BOX::getSplit() const
{
	return this->split;
}

void AABB_BOX::setSTLID(std::vector<int>& stlsID)
{
	this->stlsID.swap(stlsID);
}

void AABB_BOX::getSTLID(std::vector<int>& stlsID) const
{
	stlsID.assign(this->stlsID.begin(), this->stlsID.end());
}

inline int AABB_BOX::getSTLIDSize() const
{
	return this->stlsID.size();
}

void AABB_BOX::setBounds(const Point_3D& tm_Min, const Point_3D& tm_Max)
{
	this->m_Min = tm_Min;
	this->m_Max = tm_Max;
}

void AABB_BOX::getBounds(Point_3D& m_Min, Point_3D& m_Max)
{
	m_Min = this->m_Min;
	m_Max = this->m_Max;
}

bool AABB_BOX::IsPointInBBox(const Point_3D& point) const
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

bool AABB_BOX::IsPointInBBox_V2(const Point_3D& point) const
{
	double delta = 0.0001;
	return
		(point.getX() >= this->m_Min.getX() - delta) && (point.getX() <= this->m_Max.getX() + delta) &&
		(point.getY() >= this->m_Min.getY() - delta) && (point.getY() <= this->m_Max.getY() + delta) &&
		(point.getZ() >= this->m_Min.getZ() - delta) && (point.getZ() <= this->m_Max.getZ() + delta);
}

void AABB_BOX::subBBox(AABB_BOX& bbox1, AABB_BOX& bbox2, const std::vector<STL>& stls) const
{
	// xy依次划分
	int newSplit = this->split == 0 ? 1 : 0;
	bbox1.setSplit(newSplit);
	bbox2.setSplit(newSplit);
	double mid = 0;
	switch (newSplit)
	{
	case 0:
		mid = (double)this->m_Min.getX() + ((double)this->m_Max.getX() - (double)this->m_Min.getX()) / 2;
		bbox1.setBounds(
			Point_3D(this->m_Min.getX(), this->m_Min.getY(), this->m_Min.getZ()),
			Point_3D(mid, this->m_Max.getY(), this->m_Max.getZ()));
		bbox2.setBounds(
			Point_3D(mid, this->m_Min.getY(), this->m_Min.getZ()),
			Point_3D(this->m_Max.getX(), this->m_Max.getY(), this->m_Max.getZ()));
		break;
	case 1:
		mid = (double)this->m_Min.getY() + ((double)this->m_Max.getY() - (double)this->m_Min.getY()) / 2;
		bbox1.setBounds(
			Point_3D(this->m_Min.getX(), this->m_Min.getY(), this->m_Min.getZ()),
			Point_3D(this->m_Max.getX(), mid, this->m_Max.getZ()));
		bbox2.setBounds(
			Point_3D(this->m_Min.getX(), mid, this->m_Min.getZ()),
			Point_3D(this->m_Max.getX(), this->m_Max.getY(), this->m_Max.getZ()));
		break;
	case 2:
		mid = (double)this->m_Min.getZ() + ((double)this->m_Max.getZ() - (double)this->m_Min.getZ()) / 2;
		bbox1.setBounds(
			Point_3D(this->m_Min.getX(), this->m_Min.getY(), this->m_Min.getZ()),
			Point_3D(this->m_Max.getX(), this->m_Max.getY(), mid));
		bbox2.setBounds(
			Point_3D(this->m_Max.getX(), this->m_Min.getY(), mid),
			Point_3D(this->m_Max.getX(), this->m_Max.getY(), this->m_Max.getZ()));
		break;
	default:
		break;
	}
	// 先分子包围盒，然后根据三角面片属于哪个子包围盒来进行划分
	Point_3D p1, p2, p3;
	std::vector<int> newSTLID1, newSTLID2;
	bool inB1 = false, inB2 = false;
	for (int i = 0; i < this->stlsID.size(); ++i)
	{
		stls[stlsID[i]].getPoint(p1, p2, p3);
		inB1 = bbox1.IsPointInBBox(p1) || bbox1.IsPointInBBox(p2) || bbox1.IsPointInBBox(p3);
		inB2 = bbox2.IsPointInBBox(p1) || bbox2.IsPointInBBox(p2) || bbox2.IsPointInBBox(p3);
		if (inB1) {
			newSTLID1.push_back(stlsID[i]);
		}
		if (inB2) {
			newSTLID2.push_back(stlsID[i]);
		}
		if (!inB1 && !inB2) {
			newSTLID1.push_back(stlsID[i]);
			newSTLID2.push_back(stlsID[i]);
		}
	}
	bbox1.setSTLID(newSTLID1);
	bbox2.setSTLID(newSTLID2);
}

void AABB_BOX::subBBox_V2(AABB_BOX& bbox1, AABB_BOX& bbox2, const std::vector<STL>& stls) const
{
	double minX = m_Min.getX(), minY = m_Min.getY(), minZ = m_Min.getZ();
	double maxX = m_Max.getX(), maxY = m_Max.getY(), maxZ = m_Max.getZ();
	double subX = maxX - minX, subY = maxY - minY, subZ = maxZ - minZ;
	int newSplit = subZ > subY ? subZ > subX ? 2 : 0 : subY > subX ? 1 : 0;
	bbox1.setSplit(newSplit);
	bbox2.setSplit(newSplit);
	double mid = 0;
	switch (split)
	{
	case 0:
		mid = minX + subX / 2;
		bbox1.setBounds(
			Point_3D(minX, minY, minZ),
			Point_3D(mid, maxY, maxZ));
		bbox2.setBounds(
			Point_3D(mid, minY, minZ),
			Point_3D(maxX, maxY, maxZ));
		break;
	case 1:
		mid = minY + subY / 2;
		bbox1.setBounds(
			Point_3D(minX, minY, minZ),
			Point_3D(maxX, mid, maxZ));
		bbox2.setBounds(
			Point_3D(minX, mid, minZ),
			Point_3D(maxX, maxY, maxZ));
		break;
	case 2:
		mid = minZ + subZ / 2;
		bbox1.setBounds(
			Point_3D(minX, minY, minZ),
			Point_3D(maxX, maxY, mid));
		bbox2.setBounds(
			Point_3D(minX, minY, mid),
			Point_3D(maxX, maxY, maxZ));
		break;
	default:
		mid = minX + subX / 2;
		bbox1.setBounds(
			Point_3D(minX, minY, minZ),
			Point_3D(maxX, maxY, maxZ));
		bbox2.setBounds(
			Point_3D(minX, minY, minZ),
			Point_3D(maxX, maxY, maxZ));
		break;
	}
	// 先分子包围盒，然后根据三角面片属于哪个子包围盒来进行划分
	Point_3D p1, p2, p3;
	std::vector<int> newSTLID1, newSTLID2;
	bool inB1 = false, inB2 = false;
	for (int i = 0; i < this->stlsID.size(); ++i)
	{
		stls[stlsID[i]].getPoint(p1, p2, p3);
		inB1 = bbox1.IsPointInBBox(p1) || bbox1.IsPointInBBox(p2) || bbox1.IsPointInBBox(p3);
		inB2 = bbox2.IsPointInBBox(p1) || bbox2.IsPointInBBox(p2) || bbox2.IsPointInBBox(p3);
		if (inB1) {
			newSTLID1.push_back(stlsID[i]);
		}
		if (inB2) {
			newSTLID2.push_back(stlsID[i]);
		}
		if (!inB1 && !inB2) {
			newSTLID1.push_back(stlsID[i]);
			newSTLID2.push_back(stlsID[i]);
		}
	}
	bbox1.setSTLID(newSTLID1);
	bbox2.setSTLID(newSTLID2);
}

void AABB_BOX::print() const
{
	std::cout
		<< this->split << " " << this->stlsID.size() << " ["
		<< this->m_Min.getX() << " " << this->m_Min.getY() << " " << this->m_Min.getZ() << "], ["
		<< this->m_Max.getX() << " " << this->m_Max.getY() << " " << this->m_Max.getZ() << "]" << std::endl;
}
