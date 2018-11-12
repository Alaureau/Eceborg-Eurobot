
#include "motionCtrl.h"
#include "config.h"
#include "QEI.h"
#include <stdlib.h> 
#include <stdio.h> 
DigitalOut MOTOR_L_DIR_1(D10);//MOTEUR 1 roue gauche
DigitalOut MOTOR_L_DIR_2(D8);
PwmOut MOTOR_L_PWM(D9);

DigitalOut MOTOR_R_DIR_1(D7);
DigitalOut MOTOR_R_DIR_2(D4);
PwmOut MOTOR_R_PWM(D5);//MOTEUR roue droite


motionCtrl::motionCtrl(float m_Posx,float m_Posy,float m_Angle,float m_x_goal,float m_y_goal,float m_angle_goal) 
    
   
    //Motor_l(MOTOR_L_PWM, MOTOR_L_DIR, MOTOR_DIR_LEFT_FORWARD),
   // Motor_r(MOTOR_R_PWM, MOTOR_R_DIR, MOTOR_DIR_RIGHT_FORWARD),
    {
        MOTOR_R_PWM=0.3;
        MOTOR_L_PWM=0.3;

        ///avancer
        MOTOR_R_DIR_2= 0;
        MOTOR_R_DIR_1= 1;
        MOTOR_L_DIR_2= 1;
        MOTOR_L_DIR_1= 0;
        //this->enc_l = QEI(ENC_L_DATA1,ENC_L_DATA2,NC,200);
        //this->enc_r = QEI(ENC_R_DATA1,ENC_R_DATA2,NC,200);
    	enc_l_val=0;
        enc_r_val=0;
        //enc_l_last=0;
        //enc_r_last=0;
        sPwm_L=0.0;
        sPwm_R=0.0;
        last_Pwm_l=0.0;
        last_Pwm_r=0.0;
        Dist_last=0.0;
        angl_goal=0.0;
        Dist=0.0;
        Cap=0.0;
        Cap_last=0.0;
        x_goal=m_x_goal;
        y_goal=m_y_goal;
        angl_goal=m_angle_goal;
    	Posx=m_Posx;
    	Posy=m_Posy;
    	Angle=m_Angle;
        //pc.printf("salut");
    	timer.start();
    	asserv_ticker_ = new Ticker;
    	asserv_ticker_->attach(callback(this, &motionCtrl::asserv), ASSERV_DELAY);
    }


void motionCtrl::fetchEncodersValue() {
    enc_l_last = enc_l_val;
    enc_r_last= enc_r_val;

    enc_l_val = enc_l.getPulses();
    enc_r_val = enc_r.getPulses();
}

void motionCtrl::update_Pos()
{
	float cur_x=0.0,cur_y=0.0,cur_angle=0.0;
	float last_x=0.0,last_y=0.0,last_angla=0.0;
	float Speed=0.0;
	float Speed_angle=0.0;
	float S_left=0.0;
	float S_right=0.0;


	S_left=(perim_wheel/reso_encoder)*(enc_l_val- enc_l_last);
	S_right=((perim_wheel*coef_corr)/reso_encoder)*(enc_r_val- enc_r_last);
	Speed=(S_right+S_left)/2;
	Speed_angle=((S_left-S_right)/entraxe);
	cur_angle=(Speed_angle+Angle);
	cur_x=Speed*cosf(cur_angle)+Posx;
	cur_y=Speed*sinf(cur_angle)+Posy;
    Posx=cur_x;
    Posy=cur_y;
    Angle=cur_angle;

	

}
float recalib(float Cap)
{
     if(Cap>M_PI)Cap-=2*M_PI;
    if(Cap<(-M_PI))Cap+=2*M_PI;
    return Cap;
}
float  motionCtrl::Dist_Consigne()
{
    float Dist;
    float VRobot=(Dist_last-Dist)/ASSERV_DELAY;
    float Dfrein=(VRobot*VRobot)/(2*MAX_DECEL);
    float Vconsigne=0;
    if(Dist<Dfrein)
    {
        Vconsigne=VRobot-(MAX_DECEL*ASSERV_DELAY);
    }
    else if(VRobot<MAX_V)
    {
        Vconsigne=VRobot+(MAX_A*ASSERV_DELAY);
    }
    else
    {
        Vconsigne=MAX_V;
    }
    return Vconsigne;
}
float  motionCtrl::Ang_Consigne()
{
    float VRobot=(Cap_last-Cap)/ASSERV_DELAY;
    float Dfrein=(VRobot*VRobot)/(2*MAX_DECEL_A);
    float Vconsigne=0;
    if(Cap<Dfrein)
    {
        Vconsigne=VRobot-(MAX_DECEL_A*ASSERV_DELAY);
    }
    else if(VRobot<MAX_V_A)
    {
        Vconsigne=VRobot+(MAX_A_A*ASSERV_DELAY);
    }
    else
    {
        Vconsigne=MAX_V_A;
    }
    return Vconsigne;
}
float constrain(float val, float min, float max)
{
    if(val<min)return min;
    else if(val >max)return max;
    else return val;
}
float  motionCtrl::update_Motor(float sPwm, float last_sPwm_)
{
    if (abs(sPwm) < PWM_IS_ALMOST_ZERO)
    {
        sPwm = 0;
        last_sPwm_ = 0;
    }
    else
    {
        float current = last_sPwm_;

        sPwm = constrain(sPwm, -1, 1);

        // step the raw value
        if (abs(sPwm - current) > PWM_STEP)
        {
            if (sPwm > current)
                sPwm = current + PWM_STEP;
            else
                sPwm = current - PWM_STEP;
        }

        last_sPwm_ = sPwm;

        // map for applying the pwm
        //sPwm = SIGN(sPwm) * map(abs(sPwm), 0, 1, PWM_MIN, 1);

        // cap for boundary checking
        sPwm = constrain(sPwm, -PWM_MAX, PWM_MAX);
    }

    return sPwm;
}

void motionCtrl::Compute_PID()
{
    float Xerr=x_goal-Posx;
    float Yerr=y_goal-Posy;
    float Aerr=angl_goal-Angle;
    Dist=sqrt((Xerr*Xerr)+(Yerr*Yerr));
    float Cap=atan2(Yerr,Xerr)-Angle;
    Cap=recalib(Cap);
    if(Cap>(M_PI/2)|| Cap<(M_PI/2))
    {
        Dist=-Dist;
        Cap+=M_PI;
        Cap=recalib(Cap);
    }

    float VCap=Ang_Consigne();
    float VDist=Dist_Consigne();


    float TDist= (PID_DIST_D*(Dist-Dist_last))+((Dist_last+Dist)*PID_DIST_I)+(Dist*PID_DIST_P);
    float TAngle= (PID_ANGLE_D*(Cap-Cap_last))+((Cap_last+Cap)*PID_ANGLE_I)+(Cap*PID_ANGLE_P);
    



        float mot_l_val = TDist - TAngle;
        float mot_r_val = TDist + TAngle;

        // if the magnitude of one of the two is > 1, divide by the bigest of these
        // two two magnitudes (in order to keep the scale)
        if ((abs(mot_l_val) > 1) || (abs(mot_r_val) > 1))
        {
            float m = max(abs(mot_l_val), abs(mot_r_val));
            mot_l_val /= m;
            mot_r_val /= m;
        }

        float Pwm_L=update_Motor(mot_l_val,last_Pwm_l);
        float Pwm_R=update_Motor(mot_r_val,last_Pwm_r);
       sPwm_L=Pwm_L;
        sPwm_R=Pwm_R;
       MOTOR_L_PWM=Pwm_L;
       MOTOR_R_PWM=Pwm_R;


}
 void motionCtrl::asserv()
 {

 	this->fetchEncodersValue();
 	this->update_Pos();
    this->Compute_PID();
 }