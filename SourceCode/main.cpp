#include "mbed.h"
#include "rtos.h"
#include "QEI.h"
#include "motionCtrl.h"
#include <vector>






int main(){
	Serial pc(SERIAL_TX,SERIAL_RX,115200);
	std::vector<Task> v;
	Task s1("MOVE_POS",500,0.0,0.0);
	Task s2("MOVE_ANG",0.0,0.0,M_PI/2);
	//Task s3("MOVE_ANG",0.0,0.0,-M_PI/2);
	v.push_back(s1);
	v.push_back(s2);
	//v.push_back(s3);
	//Task Table[2]= {new Task("MOVE_POS",500,0,0),new Task("MOVE_POS",0,0,0)};
	/*Task t1,t2;
	t1=new Task("MOVE_POS",500,0,0);
	t2= new Task("MOVE_POS",0,0,0)
	Table[0]=t1;
	Table[1]=t2;*/
float a=0.0;
motionCtrl test=motionCtrl(0.0,0.0,0.0,v);

while(1)
{
	test.s1.update();
	a=test.s1.get_val();
	pc.printf("x=%f  y=%f angle=%f Pwml= %f Pwmr= %f\n",test.Posx,test.Posy,test.Angle,test.sPwm_L,test.sPwm_R);
	wait(1);
}

}