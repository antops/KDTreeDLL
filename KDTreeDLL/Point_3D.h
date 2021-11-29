#pragma once

#include <cmath>
#include <iostream>

class Point_3D
{
public:
	Point_3D();
	Point_3D(double x, double y, double z);
	~Point_3D();
	void set(double x, double y, double z);
	void setX(double x);
	void setY(double y);
	void setZ(double z);
	void get(double& x, double& y, double& z) const;
	double getX() const;
	double getY() const;
	double getZ() const;
	Point_3D& operator =(const Point_3D& p)
	{
		if (this != &p)
		{
			this->x = p.x;
			this->y = p.y;
			this->z = p.z;
		}
		return *this;
	}
	Point_3D& operator+(const Point_3D& point);
	Point_3D& operator-(const Point_3D& point);
	Point_3D& operator*(const float& num);
	double Dot(const Point_3D& v) const;
	Point_3D Cross(const Point_3D& v) const;
	void Normalization();
	double Length() const;
	double Area() const;
	void print();

private:
	double x, y, z;
};



Point_3D::Point_3D()
{
	this->x = 0.0f;
	this->y = 0.0f;
	this->z = 0.0f;
}

Point_3D::Point_3D(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Point_3D::~Point_3D()
{
}

void Point_3D::set(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void Point_3D::setX(double x)
{
	this->x = x;
}

void Point_3D::setY(double y)
{
	this->y = y;
}

void Point_3D::setZ(double z)
{
	this->z = z;
}

void Point_3D::get(double& x, double& y, double& z) const
{
	x = this->x;
	y = this->y;
	z = this->z;
}

double Point_3D::getX() const
{
	return this->x;
}

double Point_3D::getY() const
{
	return this->y;
}

double Point_3D::getZ() const
{
	return this->z;
}

Point_3D& Point_3D::operator+(const Point_3D& point)
{
	Point_3D p(x + point.x, y + point.y, z + point.z);
	return p;
}

Point_3D& Point_3D::operator-(const Point_3D& point)
{
	Point_3D p(x - point.x, y - point.y, z - point.z);
	return p;
}

Point_3D& Point_3D::operator*(const float& num)
{
	Point_3D p(x * num, y * num, z * num);
	return p;
}

double Point_3D::Dot(const Point_3D& v) const
{
	return ((double)x * (double)v.x + (double)y * (double)v.y + (double)z * (double)v.z);
}

Point_3D Point_3D::Cross(const Point_3D& v) const
{
	return Point_3D(
		y * v.z - z * v.y,
		z * v.x - x * v.z,
		x * v.y - y * v.x);
}

void Point_3D::Normalization()
{
	double temp = std::pow((x * x + y * y + z * z), 0.5);
	x /= temp;
	y /= temp;
	z /= temp;
}

double Point_3D::Length() const
{
	return std::pow((x * x + y * y + z * z), 0.5);
}

double Point_3D::Area() const
{
	return ((double)x * (double)x + (double)y * (double)y + (double)z * (double)z);
}

void Point_3D::print()
{
	std::cout << "[" << this->x << " " << this->y << " " << this->z << "]" << std::endl;
}
