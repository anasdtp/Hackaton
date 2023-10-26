#include <Arduino.h>
#include <AX12A.h>

#define DirectionPin  (10u)
#define BaudRate      (1000000ul)
#define ID            1


//ETAPES
#define ATTEINDRE_POSITION_1  0
#define ATTEINDRE_POSITION_2  1
#define ATTEINDRE_POSITION_3  2
#define ATTEINDRE_POSITION_4  3
#define ATTEINDRE_POSITION_5  4
#define ATTEINDRE_POSITION_6  5
#define TOUCHER_PLANTES  6
#define RECUPERER_PLANTES  7
#define AMENER_PLANTES_A_LA_DEPOSE  8
#define LACHER_PLANTES  9
#define RECULER  10
#define LEVER_DRAPEAU  11
#define ALLER_DEVANT_POT  12
#define PRENDRE_POT  13
#define ATTEINDRE_ZONE_PAMI  14
#define POSER_POT  15
#define ATTEINDRE_ZONE_FINALE  16

//DEBUG
#define DEBUG_STRAIGHT_LINE  205


//------ DISTANCE POUR LE CAPTEUR ULTRASON -------
#define distance_ultrason 3.


//---- ANGLES DU SERVO PELLE ---
#define ANGLE_INITIAL_PELLE  910
#define ANGLE_FINAL_PELLE  500


//Temps pour refermer la pelle pour ramasser les plantes en douceur
#define delta  1
#define vitess_descente_pelle  50

//ANGLE POUR BRANDIR LE DRAPEAU (EN DEGRE) ET TEMPS (en ms)
#define ANGLE_INITIAL_DRAPEAU  0
#define ANGLE_FINAL_DRAPEAU  90


//Lidar 
#define ETAT_GAME_MVT_DANGER 0x01
#define ETAT_GAME_PAS_DANGER 0x02

#define DistanceMin 120. //Distance minimale prise en compte par le lidar, en cm
#define DistanceMax 550.

extern AX12A Pince;

void Hugo_setup();

void PINCE(bool relacher = false);

void Hugo_loop();

void lidar_loop();

bool lidarStatus();

void strat_loop();