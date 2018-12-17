#include "mbed.h"

#include <stdlib.h> 
#include <algorithm> 
#include <stdio.h> 
#include <iostream>


class Task {
public:
	string type;
	float x;
	float y;
	float angle;
	Task(string m_type,float m_x,float m_y,float m_angle);

};