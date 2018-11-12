#include "mbed.h"
#include "rtos.h"
#include "QEI.h"

#include "motionCtrl.h"






int main(){
	Serial pc(SERIAL_TX,SERIAL_RX,115200);
motionCtrl test=motionCtrl(0.0,0.0,0.0,50.0,0.0,0.0);
while(1)
{
	wait(1);
	pc.printf("x=%f  y=%f angle=%f Pwml= %f Pwmr= %f\n",test.Posx,test.Posy,test.Angle,test.sPwm_L,test.sPwm_R);
	
}

}