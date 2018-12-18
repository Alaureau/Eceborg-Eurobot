/****************For motionCtrl************************/
DigitalOut MOTOR_R_DIR_2(D10);//MOTEUR 1 roue gauche
DigitalOut MOTOR_R_DIR_1(D8);
PwmOut MOTOR_R_PWM(D9);


DigitalOut MOTOR_L_DIR_2(D7);
DigitalOut MOTOR_L_DIR_1(D4);
PwmOut MOTOR_L_PWM(D5);//MOTEUR roue droite

/*****************************Sharps**************/
#define SHARP1 A5
#define SHARP2 A4


