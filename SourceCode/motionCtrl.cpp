
#include "motionCtrl.h"
#include "config.h"
#include "QEI.h"

#include "utils.h"

DigitalOut MOTOR_L_DIR_1(D10);//MOTEUR 1 roue gauche
DigitalOut MOTOR_L_DIR_2(D8);
PwmOut MOTOR_L_PWM(D9);


DigitalOut MOTOR_R_DIR_1(D7);
DigitalOut MOTOR_R_DIR_2(D4);
PwmOut MOTOR_R_PWM(D5);//MOTEUR roue droite

#define SHARP1 A5
#define SHARP2 A4

motionCtrl::motionCtrl(float m_Posx,float m_Posy,float m_Angle,std::vector<Task> m_Liste) :
    
   
    //Motor_l(MOTOR_L_PWM, MOTOR_L_DIR, MOTOR_DIR_LEFT_FORWARD),
   // Motor_r(MOTOR_R_PWM, MOTOR_R_DIR, MOTOR_DIR_RIGHT_FORWARD),
    s1(SHARP1),
    s2(SHARP2),
    Liste(m_Liste),
    pid_dist_(PID_DIST_P, PID_DIST_I, PID_DIST_D, ASSERV_DELAY),
    pid_angle_(PID_ANGLE_P, PID_ANGLE_I, PID_ANGLE_D, ASSERV_DELAY),
    enc_l(ENC_L_DATA1,ENC_L_DATA2,NC,200),
    enc_r(ENC_R_DATA1,ENC_R_DATA2,NC,200)
    {
       /* MOTOR_R_PWM.period_ms(10);
        MOTOR_L_PWM.period_ms(10);
        MOTOR_R_PWM.pulsewidth_ms(1);
        MOTOR_L_PWM.pulsewidth_ms(1);*/
         pid_dist_.setInputLimits(-3*1000, 3*1000);  // dist (mm)
    pid_dist_.setOutputLimits(-PID_DIST_MAX_OUPUT, PID_DIST_MAX_OUPUT);  // motor speed (~pwm)
    pid_dist_.setMode(AUTO_MODE);  // AUTO_MODE or MANUAL_MODE
    pid_dist_.setBias(0); // magic *side* effect needed for the pid to work, don't comment this
    pid_dist_.setInterval(ASSERV_DELAY);
    pid_dist_.setSetPoint(0);
    this->pidDistSetGoal(0);  // pid's error

    pid_angle_.setInputLimits(-M_PI, M_PI);  // angle (rad). 0 toward, -pi on right, +pi on left
    pid_angle_.setOutputLimits(-PID_ANGLE_MAX_OUPUT, PID_ANGLE_MAX_OUPUT);  // motor speed (~pwm). -1 right, +1 left, 0 nothing
    pid_angle_.setMode(AUTO_MODE);  // AUTO_MODE or MANUAL_MODE
    pid_angle_.setBias(0); // magic *side* effect needed for the pid to work, don't comment this
    pid_angle_.setSetPoint(0);
    pid_dist_.setInterval(ASSERV_DELAY);
    this->pidAngleSetGoal(0);  // pid's error

        MOTOR_R_PWM=0.0;
        MOTOR_L_PWM=0.0;
        isFinished=false;
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
        x_goal=Liste.front().x;
        y_goal=Liste.front().y;
        angl_goal=Liste.front().angle;
    	Posx=m_Posx;
    	Posy=m_Posy;
    	Angle=m_Angle;
        //pc.printf("salut");
    	timer.start();
    	asserv_ticker_ = new Ticker;
    	asserv_ticker_->attach(callback(this, &motionCtrl::asserv), ASSERV_DELAY);
    }
    float recalib(float Cap)
{
     if(Cap>M_PI)Cap-=2*M_PI;
    if(Cap<(-M_PI))Cap+=2*M_PI;
    return Cap;
}
void motionCtrl::DefineDistCap()
{
     float Xerr=x_goal-Posx;
    float Yerr=y_goal-Posy;
    float Aerr=angl_goal-Angle;
    if(Liste.front().type=="MOVE_POS")
    {
        Dist=sqrt((Xerr*Xerr)+(Yerr*Yerr));
        Cap= (atan2(Yerr,Xerr)-Angle);
        Cap=recalib(Cap);
        if(Cap>(M_PI/2)|| Cap<(-M_PI/2))
        {
            Dist=-Dist;
            Cap+=M_PI;
            Cap=recalib(Cap);
        }
        if ((ABS(Dist) < MC_TARGET_TOLERANCE_DIST) )//&& (ABS(cur_speed) < MC_TARGET_TOLERANCE_SPEED))
            {
                
                isFinished = true;
            }
    }
    else if(Liste.front().type=="MOVE_ANG")
    {
        Cap = std_rad_angle(Aerr);
        Dist = Dist=sqrt((Xerr*Xerr)+(Yerr*Yerr)) * cos(atan2(Yerr,Xerr)-Angle);
        if ((ABS(Cap) < MC_TARGET_TOLERANCE_ANGLE) )//&& (ABS(cur_speed_ang) < MC_TARGET_TOLERANCE_ANG_SPEED))
                isFinished=true;

    }
    

    
}
void motionCtrl::updateTask()
{
   DefineDistCap();
   this->pidDistSetGoal(Dist);
    this->pidAngleSetGoal(Cap);

}
void motionCtrl::pidDistSetGoal(float goal) {
    pid_dist_goal_ = goal;
}

void motionCtrl::pidAngleSetGoal(float goal) {
    pid_angle_goal_ = goal;
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

float  motionCtrl::Dist_Consigne()
{
    //float Dist;
    float VRobot=((Dist_last-Dist)/ASSERV_DELAY);
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
    float VRobot=((Cap_last-Cap)/ASSERV_DELAY);
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
float  motionCtrl::update_Motor(float sPwm, char cote)
{

    if(cote=='r')
    {
        float last_sPwm_= last_Pwm_r;
        if (abs(sPwm) < PWM_IS_ALMOST_ZERO)
        {
            sPwm = 0.0;
            last_sPwm_ = 0.0;
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

            last_Pwm_r= sPwm;
             sPwm = SIGN(sPwm) * map(ABS(sPwm), 0, 1, PWM_MIN, 1);
            sPwm = SIGN(sPwm) * map(ABS(sPwm), 0, 1, PWM_MIN, 1);
            if(sPwm>0.0)
            {
                MOTOR_R_DIR_2= 0;
                MOTOR_R_DIR_1= 1;
            }
            else
            {   sPwm=-sPwm;
                MOTOR_R_DIR_2= 1;
                MOTOR_R_DIR_1= 0;
            }
        }
    }
     else 
    {
        float last_sPwm_= last_Pwm_l;
        if (abs(sPwm) < PWM_IS_ALMOST_ZERO)
        {
            sPwm = 0.0;
            last_sPwm_ = 0.0;
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
            
            last_Pwm_l= sPwm;
             sPwm = SIGN(sPwm) * map(ABS(sPwm), 0, 1, PWM_MIN, 1);
            sPwm = SIGN(sPwm) * map(ABS(sPwm), 0, 1, PWM_MIN, 1);
            if(sPwm>0.0)
            {
                MOTOR_L_DIR_2= 1;
                MOTOR_L_DIR_1= 0;
            }
            else
            {   sPwm=-sPwm;
                MOTOR_L_DIR_2= 0;
                MOTOR_L_DIR_1= 1;
            }
        }
    }

    return sPwm;
}

void motionCtrl::Compute_PID()
{
    /*float Xerr=x_goal-Posx;
    float Yerr=y_goal-Posy;
    float Aerr=angl_goal-Angle;
    Dist=sqrt((Xerr*Xerr)+(Yerr*Yerr));

    Cap= (atan2(Yerr,Xerr)-Angle);

    


    Cap=recalib(Cap);
    if(Cap>(M_PI/2)|| Cap<(-M_PI/2))
    {
        Dist=-Dist;
        Cap+=M_PI;
        Cap=recalib(Cap);
    }
    
    this->pidDistSetGoal(Dist);
    this->pidAngleSetGoal(Cap);*/
    if (isFinished)
    {
        pid_dist_out_ = 0;
        pid_angle_out_ = 0;
    }
    else
    {
        pid_dist_.setProcessValue(-pid_dist_goal_);
        pid_dist_out_ = pid_dist_.compute();

        pid_angle_.setProcessValue(pid_angle_goal_);
        pid_angle_out_ = pid_angle_.compute();
        sPwm_L=pid_dist_out_;
        sPwm_R=pid_angle_out_;
    }
        
    //pid_angle_out_=Ang_Consigne();
    //pid_dist_out_=Dist_Consigne();
    Dist_last=Dist;
    Cap_last=Cap;
     //sPwm_L=pid_angle_out_;
       //sPwm_R=pid_dist_out_;
    /*
    float TDist= (PID_DIST_D*(Dist-Dist_last))+((Dist_last+Dist)*PID_DIST_I)+(Dist*PID_DIST_P);
    float TAngle= (PID_ANGLE_D*(Cap-Cap_last))+((Cap_last+Cap)*PID_ANGLE_I)+(Cap*PID_ANGLE_P);
   
    Dist_last=TDist;
    Cap_last=TAngle;
    */

       float mot_l_val = pid_dist_out_ - pid_angle_out_;
       float mot_r_val = pid_dist_out_ + pid_angle_out_;

        // if the magnitude of one of the two is > 1, divide by the bigest of these
        // two two magnitudes (in order to keep the scale)
        if ((abs(mot_l_val) > 1) || (abs(mot_r_val) > 1))
        {
            float m = max(abs(mot_l_val), abs(mot_r_val));
            mot_l_val /= m;
            mot_r_val /= m;
        }
       //sPwm_L=mot_l_val/1.5;
        //sPwm_R=mot_r_val/1.5;

       
        
        //sPwm_L=pid_dist_out_;
        //sPwm_R=pid_angle_out_;
        //sPwm_L=update_Motor(mot_l_val,'l')/1.5;
        //sPwm_R=update_Motor(mot_r_val,'r')/1.5;
       //sPwm_R=Dist;
        //s1.update();
        //s2.update();
       MOTOR_L_PWM=update_Motor(mot_l_val,'l')/1.5;
       MOTOR_R_PWM=update_Motor(mot_r_val,'r')/1.5;
      //sPwm_L=s1.get_val();
       //sPwm_R=s2.get_val();

}
void motionCtrl::MAJTask()
{
    Liste.erase(Liste.begin(),Liste.begin());
    if(Liste.size()==0)
    {
        Posy=8888;
        asserv_ticker_->detach();
    }
    else
    {
        x_goal=Liste.front().x;
        y_goal=Liste.front().y;
        angl_goal=Liste.front().angle;
    }
}
 void motionCtrl::asserv()
 {
    bool ret=false;
    //Posx++;
 	this->fetchEncodersValue();
 	this->update_Pos();
    this->updateTask();
    this->Compute_PID();
     if (isFinished)
            {   
                this->MAJTask();
                isFinished=false;
            }

 }