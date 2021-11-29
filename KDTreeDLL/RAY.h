#pragma once
#include <iostream>
#include "Point_3D.h"

class RAY
{
public:
	RAY();
	RAY(const Point_3D& _ori, const Point_3D& _dir);
	void setOriDir(const Point_3D& _ori, const Point_3D& _dir);
	void getOriDir(Point_3D& _ori, Point_3D& _dir) const;
	RAY& operator=(const RAY& ray)
	{
		if (this != &ray) {
			this->ori = ray.ori;
			this->dir = ray.dir;
		}
		return *this;
	}
	void print() const;

private:
	Point_3D ori, dir;
};

inline RAY::RAY()
{
}

inline RAY::RAY(const Point_3D & _ori, const Point_3D & _dir)
{
	this->ori = _ori;
	this->dir = _dir;
}

inline void RAY::setOriDir(const Point_3D & _ori, const Point_3D & _dir)
{
	this->ori = _ori;
	this->dir = _dir;
}

inline void RAY::getOriDir(Point_3D & _ori, Point_3D & _dir) const
{
	_ori = this->ori;
	_dir = this->dir;
}

inline void RAY::print() const
{
	std::cout << "[[" << this->ori.getX() << " " << this->ori.getY() << " " << this->ori.getZ() << "], ["
		<< this->dir.getX() << " " << this->dir.getY() << " " << this->dir.getZ() << "]]" << std::endl;
}
