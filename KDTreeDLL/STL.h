#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include "Point_3D.h"

class STL
{
public:
	STL();
	~STL();
	void setPoint(const Point_3D& point1, const Point_3D& point2, const Point_3D& point3);
	void setPoint1(const Point_3D& point);
	void setPoint2(const Point_3D& point);
	void setPoint3(const Point_3D& point);
	void getPoint(Point_3D& point1, Point_3D& point2, Point_3D& point3) const;
	STL& operator=(const STL& stl)
	{
		if (this != &stl) {
			this->point1 = stl.point1;
			this->point2 = stl.point2;
			this->point3 = stl.point3;
		}
		return *this;
	}
	void print() const;

private:
	Point_3D point1, point2, point3;
};

inline STL::STL()
{
}

inline STL::~STL()
{
}

inline void STL::setPoint(const Point_3D& _point1, const Point_3D& _point2, const Point_3D& _point3)
{
	this->point1 = _point1;
	this->point2 = _point2;
	this->point3 = _point3;
}

inline void STL::setPoint1(const Point_3D& point)
{
	this->point1 = point;
}

inline void STL::setPoint2(const Point_3D& point)
{
	this->point2 = point;
}

inline void STL::setPoint3(const Point_3D& point)
{
	this->point3 = point;
}

inline void STL::getPoint(Point_3D& _point1, Point_3D& _point2, Point_3D& _point3) const
{
	_point1 = this->point1;
	_point2 = this->point2;
	_point3 = this->point3;
}

inline void STL::print() const
{
	std::cout << "[[" << this->point1.getX() << " " << this->point1.getY() << " " << this->point1.getZ() << "],["
		<< this->point2.getX() << " " << this->point2.getY() << " " << this->point2.getZ() << "],[" << this->point3.getX() << " " << this->point3.getY() << " " << this->point3.getZ() << "]]" << std::endl;
}
