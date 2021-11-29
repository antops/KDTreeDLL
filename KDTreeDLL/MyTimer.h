#pragma once

// windows 计时器API所需头文件
#include <ctime>

class MyTimer
{
public:
	MyTimer();
	~MyTimer();
	void startTimer();
	void endTimer();
	long getTime() const;

private:
	clock_t start, end;
	long time;
};


MyTimer::MyTimer()
{
	this->start = 0;
	this->end = 0;
	this->time = 0;
}

MyTimer::~MyTimer()
{
}

void MyTimer::startTimer()
{
	this->start = clock();
}

void MyTimer::endTimer()
{
	this->end = clock();
	this->time = this->end - this->start;
}

long MyTimer::getTime() const
{
	return this->time;
}
