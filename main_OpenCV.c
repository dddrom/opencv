#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include "../gpio_access.h"
#include <sys/mman.h>
#include <native/task.h>
#include <signal.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

RT_TASK pwm_task; // commande des deux moteurs
RT_TASK asserv_task; // asservissement moteurs

RT_TASK servo_task; // commande de deux servos
double duree1_servo;
double duree2_servo;
int angle1;
int angle2;

double duree1;
double duree2;
double cycl1;
double cycl2;
long rotations_command1;
long rotations_command2;
char *codeur1 = "/dev/encoder1";
char *codeur2 = "/dev/encoder2";
double k = 10000; // correcteur pour l'asservissement


void terminaison(){
	GPIO_CLR = (1<<24); // PWMA L
	GPIO_CLR = (1<<25); // PWMB L
    GPIO_CLR = (1<<21); // STBY L  
    GPIO_CLR = (1<<22); // SERVO1 L
	GPIO_CLR = (1<<23); // SERVO2 L
    GPIO_CLR = (1<<27); // AIN1 L
    GPIO_CLR = (1<<19); // AIN2 L
    GPIO_CLR = (1<<20); // BIN1 L
    GPIO_CLR = (1<<26); // BIN2 L
	exit(0);
}

void pwm(void *arg) { // {XENO} fonctionne en modulant duree1 et duree2 pour les deux moteurs
	while(1){
		GPIO_SET = (1<<24); // PWMA
		GPIO_SET = (1<<25); // PWMB
		
		if (duree1<=duree2){
			rt_task_sleep(duree1);
			GPIO_CLR = (1<<24);
			rt_task_sleep(duree2-duree1);
			GPIO_CLR = (1<<25);
			rt_task_sleep(2000000 - duree2); // 2ms
		}
		else {
			rt_task_sleep(duree2);
			GPIO_CLR = (1<<25);
			rt_task_sleep(duree1-duree2);
			GPIO_CLR = (1<<24);
			rt_task_sleep(2000000 - duree1);
		}
	}
}

void asserv(void *arg) { // {XENO} asservi le controle des moteurs en fonction de la commande utilisateur initiale
    int codeur_num1 = open(codeur1,O_RDONLY);
    int codeur_num2 = open(codeur2,O_RDONLY);
    long code1_1;
    long code1_2;
    long code2_1;
    long code2_2;
    long rotations1;
    long rotations2;
	while(1){
		read(codeur_num1,&code1_1,8);
        read(codeur_num2,&code2_1,8);
		rt_task_sleep(200000000); // 200ms
		read(codeur_num1,&code1_2,8);
        read(codeur_num2,&code2_2,8);
		rotations1 = -(code1_2 - code1_1);
        rotations2 = -(code2_2 - code2_1);
		long ecart1 = rotations1 - rotations_command1;
        long ecart2 = rotations2 - rotations_command2; // attention au signe de rotations2 ?
        //printf("ecart1 : %ld rotations1 : %ld rotations_command1 : %ld\n",ecart1,rotations1,rotations_command1);
        //printf("ecart2 : %ld rotations2 : %ld rotations_command2 : %ld\n",ecart2,rotations2,rotations_command2);
		if(ecart1 < 0)
			speed_up(1,ecart1);
		else
			speed_down(1,ecart1);
        if(ecart2 < 0) // signe de rotations2 ?
			speed_up(2,ecart2);
		else
			speed_down(2,ecart2);
	}
}

void speed_up(int motor, long ecart) { // augmente la vitesse de rotation de motor
	if(motor == 1) {
		duree1 = duree1 + k*(-ecart);
        //duree1 = 0.2*2000000.0;
        //printf("augmente duree1 : %lf \n",duree1);
	}
	if(motor == 2) {
		duree2 = duree2 + k*(-ecart);
	}
}

void speed_down(int motor, long ecart) { // baisse la vitesse de rotation de motor
	if(motor == 1) {
		duree1 = duree1 - k*ecart;
	}
	if(motor == 2) {
		duree2 = duree2 - k*ecart;
	}
}

void sense_direction(int motor, int sense) { // sense = 1 le robot avance, sense = -1 le robot recul (pour chaque moteur sans considération du sens de montage)
    //printf("Sens pour sens_direction : %i (pour motor %i)\n", sense,motor);
    if(motor == 1) {
		if(sense == 1) { // A -> CCW
            GPIO_CLR = (1<<27); // AIN1 L
            GPIO_SET = (1<<19); // AIN2 H
            printf("A -> CCW \n");
        }
        else { // A -> CW
            GPIO_SET = (1<<27); // AIN1 H
            GPIO_CLR = (1<<19); // AIN2 L 
            printf("A -> CW \n");
        }
	}
	if(motor == 2) {
		if(sense == 1) { // B -> CW
            GPIO_SET = (1<<20); // BIN1 H
            GPIO_CLR = (1<<26); // BIN2 L
            printf("B -> CW \n");
        }
        else {  // B -> CCW
            GPIO_CLR = (1<<20); // BIN1 L
            GPIO_SET = (1<<26); // BIN2 H
            printf("B -> CCW \n");
        }
	}
}

void set_motor(int motor, double cycl) { // MOTOR A = 1, MOTOR B = 2, -1.0 <= cycle <= 1.0 le rapport cyclique définissant aussi le sens de rotation
    int sense = 1;
    double cycle = cycl;
    
    if (cycle > 1.0) {
        printf("\n Le rapport %i est trop grand, il est ramené à 1.0", motor);
        cycle = 1.0;
    }
    else if (cycle < -1.0) {
        printf("\n Le rapport %i est trop bas, il est ramené à -1,0", motor);
        cycle = -1.0;
    }
    printf("Rapport %i choisi : %lf; ", motor, cycle);
    if(cycle < 0) {
        sense = -1;
        cycle = -cycle;
    }
    printf("Sens %i choisi : %i \n", motor, sense);
    sense_direction(motor,sense);
    
    if(motor == 1) {
        //duree1 = cycle*2000000.0; // >0
        rotations_command1 = cycle*316/5; // max rotations : 316/s, vérifier le signe pour l'asservissement
        printf("Duree1 choisi : %lf \n ", duree1);
    }
    if(motor == 2) {
        //duree2 = cycle*2000000.0; // >0
        rotations_command2 = cycle*316/5; // max rotations : 316/s
        printf("Duree2 choisi : %lf \n", duree2);
    }
}
    
void servo(void *arg) { // ccontrole en pwm des deux servomoteurs
	while(1){
		GPIO_SET = (1<<22);
		GPIO_SET = (1<<23);
		
		if (duree1_servo<=duree2_servo){
			rt_task_sleep(duree1_servo);
			GPIO_CLR = (1<<22);
			rt_task_sleep(duree2_servo-duree1_servo);
			GPIO_CLR = (1<<23);
			rt_task_sleep(20000000 - duree2_servo);
		}
		else {
			rt_task_sleep(duree2_servo);
			GPIO_CLR = (1<<23);
			rt_task_sleep(duree1_servo-duree2_servo);
			GPIO_CLR = (1<<22);
			rt_task_sleep(20000000 - duree1_servo);
		}
	}
}
    
void set_servo(int servo, int angl) { // définition de l'angle de servo (1 ou 2) de 0° à 180°
    int angle = angl;
    if(servo == 1) {
        if (angle > 180) {
		  printf("L'angle 1 est trop grand, il est ramené à 180°");
		  angle = 180;
        }
        else if (angle < 0) {
		  printf("L'angle 1 est negatif, il est ramené à 0°");
		  angle = 0;
        }
        printf("Angle 1 choisi : %d \n", angle);
        duree1_servo = (700.0 + ((double) angle + 90.0)*2000.0/180.0)*1000; //mauvaises valeur, le servo est défecteux ...
    }
    
    if(servo == 2) {
        if (angle > 180) {
		  printf("L'angle 2 est trop grand, il est ramené à 180°");
		  angle = 180;
        }
        else if (angle < 0) {
		  printf("L'angle 2 est negatif, il est ramené à 0°");
		  angle = 0;
        }
        printf("Angle 2 choisi : %d \n", angle);
        //duree2_servo = (700.0 + ((double) angle + 90.0)*2000.0/180.0)*1000;
        duree2_servo = ( 0.6 + ( ((double) angle)*1.9/180 ) )*1000000;
    }
}

void straight() {
    set_motor(1,1.0);
    set_motor(2,1.0);
    //printf("Tout droit ! \n");
}

void left() {
    set_motor(1,0.8);
    set_motor(2,0.2);
    //printf("A gauche ! \n");
}

void right() {
    set_motor(1,0.2);
    set_motor(2,0.8);
    //printf("A droite ! \n");
}

int main (int argc, char *argv[]){

	signal(SIGINT, terminaison);
	
	int err1;
	int err2;
    int err3;

	mlockall(MCL_CURRENT|MCL_FUTURE);

	setup_gpio_address();
    INP_GPIO(24); // PWMA
	INP_GPIO(25); // PWMB
    INP_GPIO(27); // AIN1 ancien 16
	INP_GPIO(19); // AIN2
    INP_GPIO(20); // BIN1
	INP_GPIO(26); // BIN2
    INP_GPIO(21); // STBY
    INP_GPIO(22); // SERVO 1
	INP_GPIO(23); // SERVO 2
	OUT_GPIO(24); // PWMA
	OUT_GPIO(25); // PWMB
    OUT_GPIO(27); // AIN1
	OUT_GPIO(19); // AIN2
    OUT_GPIO(20); // BIN1
	OUT_GPIO(26); // BIN2
    OUT_GPIO(21); // STBY
    OUT_GPIO(22); // SERVO 1
	OUT_GPIO(23); // SERVO 2
	
	duree1 = 0.0; //moteurs A et B à l'arrêt initiallement
	duree2 = 0.0;
    
    cycl1 = 0.0;
    cycl2 = 0.0;
    
    rotations_command1 = 0;
    rotations_command2 = 0;
    
    GPIO_SET = (1<<21); // STBY H
    
    GPIO_CLR = (1<<27); // Low motor sens
    GPIO_CLR = (1<<19);
    GPIO_CLR = (1<<20);
    GPIO_CLR = (1<<26);
    
    // servo 1 et 2 à 90° initiallement (Bon pour servo 2, servo 1 défecteux)
    duree1_servo = 1600000;
	duree2_servo = 1600000;

	err1 = rt_task_create(&pwm_task, "pwm_motor", 0, 99, 0);
	if (!err1)
		rt_task_start(&pwm_task, &pwm, NULL);

	err2 = rt_task_create(&asserv_task, "asserv_motor", 0, 99, 0);
	if (!err2)
		rt_task_start(&asserv_task, &asserv, NULL);
    
    err3 = rt_task_create(&servo_task, "pwm_servo", 0, 99, 0);
	if (!err3)
		rt_task_start(&servo_task, &servo, NULL);
	
    // ----------------------- OpenCV ------------------------------
    // -------------------------------------------------------------
    float bmin = 260.0;
    float bmax = 380.0;
    float bmin2 = 310.0;
    float bmax2 = 330.0;
    bool showimage = false;
    VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Frame size : " << dWidth << " x " << dHeight << endl; //usually 640x480

    if(showimage){
        //namedWindow("grey",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
        //namedWindow("bw",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
        namedWindow("final",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    }

    waitKey(100);

    //donne l'etat du mouvement du robot
    //0 pour tout droit, 1 pour tourner a gauche, 2 pour tourner a droite;
    int state =0;
    
    //GPIO_CLR = (1<<16); // AIN1 L
    set_motor(1,0.4);
    set_motor(2,0.4);
    set_servo(1,-45);
    set_servo(2,-45);
              
	while(1) {
        clock_t start = clock();
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        

        Mat roi = frame(Rect (0,190,640,100));
        Mat bwroi;// = roi > 128;
        cvtColor(roi,bwroi,CV_RGB2GRAY);
        //if (showimage) imshow("grey", bwroi);
        bwroi=bwroi>160;
        blur( bwroi, bwroi, Size(3,3));
        //if (showimage) imshow("bw", bwroi);


        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        float xcenter = -1.0f;

        findContours(bwroi, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, Point(0,0));
        //cout<<"nombre de contours : "<<contours.size()<<endl;
        for (size_t i = 0; i < contours.size(); i++) {

            float area = contourArea(contours[i]);
            //cout<<"area "<<i<<": "<<area<<endl;
            if (area > 2000) {
                Moments mu;
                mu = moments(contours[i], false);
                Point2f center(mu.m10 / mu.m00, 50); // point in center (x only)
                if (showimage) circle(roi, center, 5, Scalar(0, 255, 0), -1, 8, 0);
                cout<<"centre : "<<center.x<<endl; 
                xcenter =center.x;
                break;  
            }
        }
        cout<<"centre : "<<xcenter<<endl;  
        if (xcenter!=-1){
            //cout<<"test"<<endl;
            //Le centre de la ligne est dans la zone centrale
            if (xcenter>bmin2 && xcenter<bmax2){
                cout<<"cas1"<<endl;
                if(state!=0){
                    //setGpio pour aller tout droit
                    straight();
                    cout<<"on va tout droit"<<endl;
                    state=0;
                }
            }
            else if (xcenter<bmin){
                cout<<"cas2"<<endl;
                if (state !=1){
                    //setGpio pour tourner a gauche
                    left();
                    cout<<"on tourne a gauche"<<endl;
                    state=1;
                }
            }
            else if (xcenter>bmax){
                cout<<"cas3"<<endl;
                if (state !=2){
                    //set_Gpio pour tourner a droite
                    right();
                    cout<<"on tourne a droite"<<endl;
                    state=2;
                }
            }
            
        }


        if(showimage) {
            Point2f p1(bmin, 0.0f);
            Point2f p2(bmin,100.0f);
            line(roi, p1, p2, Scalar(255,0,0), 1,8,0);
            p1= Point2f(bmax, 0.0f);
            p2=Point2f(bmax, 100.0f);
            line(roi, p1, p2, Scalar(255,0,0), 1,8,0);
            p1= Point2f(bmin2, 0.0f);
            p2=Point2f(bmin2, 100.0f);
            line(roi, p1, p2, Scalar(0,255,0), 1,8,0);
            p1= Point2f(bmax2, 0.0f);
            p2=Point2f(bmax2, 100.0f);
            line(roi, p1, p2, Scalar(0,255,0), 1,8,0);

            imshow("final", roi); //show the frame in "MyVideo" window
        }


        clock_t end = clock();
        double cpu_time = static_cast<double> (end - start)/CLOCKS_PER_SEC;
        //cout<<"fps : "<<1.0/cpu_time<<endl;
        
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
	}
	 
}

void cleanup (void) {
	rt_task_delete(&pwm_task);
	rt_task_delete(&asserv_task);
    rt_task_delete(&servo_task);
}
