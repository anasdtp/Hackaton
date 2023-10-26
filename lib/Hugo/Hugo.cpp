#include <Hugo.h>
#include <RPLidar.h>
#include <Servo.h>
#include <AX12A.h>
#include <CRAC_utility.h>
RPLidar lidar;
// Servo drapeau;


// AVANT DE LANCER LE ROBOT:
// 1. DEFINIR PINS
// 2. DEFINIR COORDONNEES POUR CHAQUE ETAPE
// 3. DEFINIR ANGLES ET AUTRES PARAMETRES

//---------PINS A DEFINIR---------

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.

#define trigPin 9

#define echoPin 8

#define PIN_SERVO_DRAPEAU 13 // Pin pour le servo du drapeau

#define MAGNET_PIN 4 // PIN AIMANT

//---- PINS DEJA DEFINIS ------

#define pinStart 5


//----------------------- STRATEGIE -------------------------
#define SIZE_ACTION 17



// Chelou : 
//POSITIONS A ATTEINDRE EN (x,y) -->>> A EDITER POUR LA STRATEGIE FINALE
const int POS_0[] = {280, 280, 0};
const int POS_1[] = {1130,280, 0};
const int POS_2[] = {1445,985, 0};
const int POS_3[] = {565,985, 0};
const int POS_4[] = {755,380, 0};
const int POS_5[] = {1210, 380, 0};
const int POS_6[] = {225, 1780, 0};
const int POS_7[] = {330, 1610, 0};
const int POS_8[] = {210,1510, 0};
const int POS_9[] = {980, 1800, 0};

//reculs en cm
const int RECUL_APRES_PLANTES = -7;
const int RECUL_APRES_POT = -7;

//COORDONNEES DE LA DEPOSE ET ORIENTATION (x,y,theta)
const int POS_DEPOSE[] = {200, 200, 2700};

//COORDONNEES JUSTE DEVANT LA DEPOSE POUR QUE LE ROBOT RECULE
const int POS_JUSTE_DEVANT_DEPOSE[] = {150, 200, 0};

//COORDONNEES DU POT
const int POS_POT[] = {-300, 0, 0};

//COORDONNEES PAMI
const int POS_PAMI[] = {0, 0, 0};

//COORDONNEES ZONE FINALE
const int POS_FINALE[] = {20, 50, 0};
// Chelou : 





//--------- PARAMETRES DE CONTROLE -------
unsigned long startTime;

int etape_actuelle = 0;
//ULTRASON
float duration, distance, marge_mtplant;
//SERVO PELLE
float pos_servo_shovel = ANGLE_INITIAL_PELLE;
float current_shovel_angle = 0;

//Controle general du mouvement
float precision_position = 20; //precison en mm
float precision_angle = PI / 16; //precision en radian

bool movement_started = false;
unsigned long start_time = 0;

int n;                    //Compteur pour le Lidar
int detection;


AX12A Pince;
void Hugo_setup(){
    //pelle
  Pince.begin(BaudRate, DirectionPin, &Serial3);
  PINCE();

  //drapeau
  // drapeau.attach(PIN_SERVO_DRAPEAU);
  // drapeau.write(ANGLE_INITIAL_DRAPEAU);
  lidar.begin(Serial2);                           //utiliser TX2 et RX2 pour le Lidar
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 255);
  delay(100);
  lidar.startScan();
}

void PINCE(bool relacher){
  if(relacher){
    Pince.moveSpeed(ID, ANGLE_FINAL_PELLE, 800);
  }else{
    Pince.moveSpeed(ID, ANGLE_INITIAL_PELLE, 800);
  }
}


void Hugo_loop(){
    if (detection==true){

    }
}

bool DANGER = false;
void lidar_loop(){
  if (IS_OK(lidar.waitPoint())) {
  float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
  float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
  
  bool condition_sens = = (angle>=180-25 && angle<=180+25);
 
  // if(target_sens<0){
  //   condition_sens = (angle>=360-25 && angle<=25); // Si on part en arriere
  // }

  if(distance>DistanceMin && distance<DistanceMax && condition_sens){ detection+=1;}

  n+=1;
  if (n>1500){ n=0; detection=0; DANGER = false;}
  if (detection>10){DANGER = true;}
  }
  else{DANGER = false;}

}

bool lidarStatus(){
  return DANGER;
}