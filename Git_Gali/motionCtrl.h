#include "mbed.h"
#include "rtos.h"
#include "QEI.h"
#include "motor.h"
#include "PID.h"
#define ENC_RADIUS          20.0                    // one encoder radius
#define ENC_PERIMETER       (2*M_PI*ENC_RADIUS)     // one encoder perimeter
#define ENC_POS_RADIUS      87                      // distance from one encoder to the center of the robot
#define TICKS_PER_MM_L      96.0
#define TICKS_PER_MM_R      96.0
#define MM_TO_TICKS_L(val)  ((val)*TICKS_PER_MM_L)
#define TICKS_TO_MM_L(val)  ((val)/TICKS_PER_MM_L)
#define MM_TO_TICKS_R(val)  ((val)*TICKS_PER_MM_R)
#define TICKS_TO_MM_R(val)  ((val)/TICKS_PER_MM_R)
#define ENC_L_DATA1 D11 
#define ENC_L_DATA2 D3
#define ENC_R_DATA1 A0
#define ENC_R_DATA2 A1
#define min_diff_enc 0.5
#define perim_wheel 128.74
#define reso_encoder 2050.0
#define coef_corr 1
#define entraxe 310.0
#define MAX_DECEL 2000
#define MAX_V 250
#define MAX_A 600
#define MAX_DECEL_A 4
#define MAX_V_A 1
#define MAX_A_A 3

#define PID_DIST_MAX_OUPUT  0.8
#define PID_ANGLE_MAX_OUPUT 1 

// PID settings

#define PID_DIST_P          6.0
#define PID_DIST_I          0
#define PID_DIST_D          (0.5/1000)

#define PID_ANGLE_P         3.0
#define PID_ANGLE_I         0
#define PID_ANGLE_D         (1.0/1000)





class motionCtrl {
public:
	motionCtrl(float m_Posx,float m_Posy,float m_Angle,float m_x_goal,float m_y_goal,float m_angle_goal);
	void asserv();
	void update_Pos();
	void Compute_PID();
	float update_Motor(float sPwm, char cote);
	void fetchEncodersValue();
	float  Dist_Consigne();
	float  Ang_Consigne();
	 void pidDistSetGoal(float goal);
    void pidAngleSetGoal(float goal);
	int32_t enc_l_val,enc_l_last,enc_r_val,enc_r_last;
	

	PID pid_dist_, pid_angle_;
       // tmp variable used as a working var
                                            //   use this instead of the raw value from the QEI objects.
                                            //   unit: encoder ticks
    float pid_dist_goal_, pid_angle_goal_;  // units: mm and rad
    float pid_dist_out_, pid_angle_out_;


float x_goal,y_goal;
	//Motor Motor_l, Motor_r;
	float last_Pwm_l,last_Pwm_r;
	float sPwm_L,sPwm_R;
	float Dist_last,angl_goal;
	float Dist;
	float Cap,Cap_last;
	float Posx,Posy,Angle;
	float Speed;
	Timer timer;
	Ticker *asserv_ticker_;
	QEI enc_l;
	QEI enc_r;
	//Motor Motor_l, Motor_r;
	

};