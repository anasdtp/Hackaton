#include <Arduino.h>
#include <CRAC_utility.h>
#include <buffer_circulaire.h>//inutile
#include "math.h"
#include <mouvement.h>
#include <timerAsserBas.h>
#include <moteur.h>
#include <Hugo.h>

#define PIN_JACK 5
bool WaitForJack = false;

#define SIZE_ACTION 6 + 1 + 20       +1 //Depart + Jack + jeu   +1 fin de match

//Phase d'initialisation, de recalage : 
#define EN_BAS_A_DROITE 0 //L'origine donc x = x y = y theta = theta //Zone Bleu
#define EN_BAS_A_GAUCHE 1 //donc y = 3000 - y et theta = -theta //Zone JAUNE
//On ne peut pas partir d'en HAUT (pour le moment...); EN BAS c'est coté mat

#define RECALAGE_DEPART EN_BAS_A_GAUCHE //Donc en zone JAUNE
#define MOITIEE_ROBOT 94

//Phase de jeu : 
//Pour la phase du match choisir une couleur, couleur de base BLEU, mettre les coordonnées comme si on etait en BLEU meme si on est JAUNE
#define BLEU 0
#define JAUNE 1

#define COULEUR_STRAT JAUNE //Bleu Ou JAUNE, de base il faut faire la strat comme si on etait en bleu et sa s'inverse tout seul
// IMPORTANT FAIRE LA STRAT EN BLEU



typedef struct Etape
{
  /* data */
  int ACTION;
  int variable;
  uint16_t x,y,theta; signed char sens;//Asservissement XYT

  int16_t angle; //rotation

  int16_t distance; uint8_t mode; int16_t val_recalage = MOITIEE_ROBOT; //Ligne droite ou recalage, Si mode<0 recalage sur y sinon sur x. Si mode = 0 alors ligne droite

  int16_t position_servo; //Si à 1 relacher sinon lever
}Etape;

Etape strategie_hackaton[SIZE_ACTION];
bool next_action = true;
//Lidar : -----------------------------------------------------------------------------
char type_Evitement = ETAT_GAME_PAS_DANGER; //Mouvement ETAT_GAME_MVT_DANGER evitement sinon non 
unsigned short target_x,target_y,target_theta;signed char target_sens;//Pour aller là où on devait aller apres que le robot soit passé
//Fin init var Lidar ------------------------------------------------------------------

//Construction liste strategie : ------------------------------------------------------

/* [TYPE] [ARG1] [ARG2] [ARG3] [ARG4]*/

/*  J : Jack ;;; Wait for jack
    X : XYT [x];[y];[t];
    L : Ligne droite [d];;;
    R : Rotation [a];;;
    O : Recalage [d];[x ou y?]si x mettre 1, si y -1;[Val recalage] valeur que prendra l'encodeur à la fin;

    A : Action [Type] si Type à 1 c'est servomoteur 9G, 2 Servo pince ; [Position servo] 0 : retracter, 1 : Lacher ;;
*/

int16_t tab_action_brut[SIZE_ACTION][5]={
    //Phase d'initialisation (pendant les 3 min) :
    {'O', 1000,1,MOITIEE_ROBOT,0},//On avance doucement jusqu'a se plaquer contre le mur sur l'axe des x
    {'L', -100,0,0,0},//On recule
    {'R', 900,0,0,0},//On tourne
    {'O', 1000,-1,MOITIEE_ROBOT,0},//On avance doucement jusqu'a se plaquer contre le mur sur l'axe des y
    {'L', -100,0,0,0},//On recule

    {'X', 250,250,900,0},//On part à la position initiale

    {'J', 0,0,0,0},//Attente du jack
    //Debut du match :
    {'X', 700,780,900,0},//On se place devant le premier tas
    {'L', 150,0,0,0},//On avance face à eux
    {'A', 2,1,0,0},//On les attrape avec la pince
    {'L', 50,0,0,0},//On s'avance un peu pour mieux les attraper

    {'X', 2000-300,350,-900,0},//On part dans une assiette
    {'A', 2,0,0,0},//On lache les pots
    {'L', 50,0,0,0},//On les pousse un peu
    {'L', -200,0,0,0},//On a terminer on recule

    {'X', 2000 - 700,780,900,0},//On se place devant le second tas
    {'L', 150,0,0,0},//On avance face à eux
    {'A', 2,1,0,0},//On les attrape avec la pince
    {'L', 50,0,0,0},//On s'avance un peu pour mieux les attraper

    {'X', 300,350, -900,0},//On part dans une assiette
    {'A', 2,0,0,0},//On lache les pots
    {'L', 50,0,0,0},//On les pousse un peu

    {'L', -300,0,0,0},//On a terminer on recule

    {'R', -900,0,0,0},//On se tourne face au mur
    {'O', 1000,1,MOITIEE_ROBOT,0},//On fait un recalage, car on a forcement perdu de la precision, contre l'axe des x
    {'L', -150,0,0,0},//On recule

    {'X', 2000-250,300,0,0},//On vas se garer

    {'0', 0,0,0,0}//Fin de match
};
//Fin Construction liste strategie : --------------------------------------------------
//----------------------------------------------------------------------Variables
volatile uint16_t          mscount = 0,      // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                           mscount1 = 0,     // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                           mscount2 = 0,     // Compteur utilisé pour envoyer la trame CAN d'odométrie
                           mscount_lidar = 0;//Attendre que le robot passe

extern double Odo_val_pos_D, Odo_val_pos_G, Odo_last_val_pos_D, Odo_last_val_pos_G;

int nbValeurs = 0;

double Kp =2. , Ki =0.005, Kd =0.1;//double Kp =8    , Ki =0.4, Kd =10.0;
                
unsigned short cpt = 0; int cpt_ordre = 0; //Va savoir à quoi ça sert        

int set = false;


/****************************************************************************************/
//Sinon, détails : les angles sont exprimés en dixièmes de degrés quand il faut faire des calculs, ou quand ils faut les transmettre en CAN
//les distances, pareil, c'est de millimètres
//Par contre, pour les fonctions Ligne_droite, Rotation, et asser_pos c'est exprimé en ticks d'encodeur
/****************************************************************************************/
                             
/****************************************************************************************/
/*                               Deplacement a effectuer                                */
/*                Liste des ordres de déplacement à effectuer à la suite                */
/*                    (PAS IMPLEMENTE, OU ALORS CA MARCHE TRES MAL)                     */
/****************************************************************************************/ 
struct Ordre_deplacement liste;

//----------------------------------------------------------------------prototypes fonctions 
void TempsEchantionnage(uint16_t TIME);
//Fonctions principales : 
void calcul(void);
void CANloop();
void Odometrie(void);
void remplirStructStrat();
bool jack();
void setup() {
  Serial.begin(115200);
    init_coef();

  // Encodeur_Init(); Serial.println("fin encodeur init");
  Moteur_Init(); Serial.println("fin moteur init");
  
  AsserInitCoefs(Kp, Ki, Kd);
  Asser_Init();
  //Interruption : le programme est cadence grâce a l'interrutpion du Timer
  init_Timer(); Serial.println("fin init Timer");
  
  Hugo_setup();

  Serial.println("fin setup");
  
  remplirStructStrat(); 

   
  Odo_x = 450; Odo_y = 450; Odo_theta = 0;
    delay(500);
  mscount = 0;
}

void loop() {

    calcul();
    Odometrie();
    CANloop();

    
    
    TempsEchantionnage(TE_100US);  
}

void TempsEchantionnage(uint16_t TIME){
    if (mscount >= TIME) 
  {   
    Serial.println("erreur temp calcul");
    Serial.println(mscount);
    //remplirStruct(DATArobot,ERREUR_TEMP_CALCUL,2, mscount,TE_100US,0,0,0,0,0,0);
    //writeStructInCAN(DATArobot);
  }
  else 
  {
    while (mscount<(TIME)){
        //Lidar
        lidar_loop();
    }
  }
  //digitalWrite(27, set);//pour mesurer le temps de boucle avec l'oscilloscope
  //set = !set; temps de boucle = 1/(freq/2)
  mscount = 0; 
}
//----------------------------------------------------------------------fonctions
/***************************************************************************************
 NOM : calcul                                                              
 ARGUMENT : aucun                                        
 RETOUR : rien                                                                        
 DESCRIPTIF :   Fonction principale appelé periodiquement, qui gère l'ensemble
                Se base sur : struct Ordre_deplacement liste;
                liste d'ordre remplie par l'ISR CAN
                struct Ordre_deplacement
                {
                    char type, enchainement;
                    short vmax, amax;
                    long distance, recalage, val_recalage;
                    long angle;
                    short x, y, theta;
                    signed char sens;
                    short rayon, vit_ray, theta_ray;
                };La fonction est la boucle principale qui gère la robotique de l'application. 
                Elle est appelée périodiquement et s'assure que la liste des ordres est remplie par l'ISR CAN. 
                Cette liste est basée sur la structure "Ordre_deplacement". 
                La fonction détermine le prochain mouvement à effectuer à partir de la liste des ordres, 
                et appelle la fonction correspondante pour l'exécuter.
                Le mouvement à effectuer est basé sur le type de mouvement indiqué dans la structure "Ordre_deplacement". 
                Le mouvement peut être une ligne droite, une rotation, un mouvement sur les axes x, y et theta, ou un mouvement 
                sur un rayon de courbure. La fonction utilise également une série de commutateurs pour exécuter les mouvements 
                et les contrôler en temps réel. Le premier commutateur est destiné à l'état précédent et compare l'état actuel 
                à l'état précédent pour déterminer si un changement d'état s'est produit. Le deuxième commutateur est destiné à 
                l'automate de déplacement précédent et utilise un état pour savoir si le robot est en mouvement ou non. 
                La fonction arrête le robot lorsque le mouvement suivant est "TYPE_END_GAME", lorsque le robot n'est pas asservi 
                ou lorsque le match est terminé. En outre, la fonction envoie des messages de débogage à l'aide de //CANenvoiMsg1Byte(), 
                qui envoie un message CAN d'une longueur de 1 octet. Les messages de débogage sont destinés à fournir des informations 
                de débogage pour le débogage en temps réel de l'application. (Merci ChatGPT)
***************************************************************************************/
void calcul(void){//fait!!
    static int cpt_stop = 0;
    static char etat_prec = TYPE_DEPLACEMENT_IMMOBILE, etat_automate_depl_prec = INITIALISATION;
    if(stop_receive!=1)
    {
        #if F_DBUG_ETAT
        if(etat_prec != liste.type)
        {
            etat_prec = liste.type;
            
            //Serial.println("ID_DBUG_ETAT");
            //remplirStruct(DATArobot,ID_DBUG_ETAT, 1, etat_prec, 0,0,0,0,0,0,0);
            //writeStructInCAN(DATArobot);                             //CAN
            #if F_DBUG_ETAT_DPL
            //remplirStruct(DATArobot,ID_DBUG_ETAT_DPL, 1, etat_automate_depl, 0,0,0,0,0,0,0);
            //writeStructInCAN(DATArobot);                             //CAN
            #endif
        }
        #endif
        
        #if F_DBUG_ETAT_DPL
        if(etat_automate_depl_prec != etat_automate_depl && etat_automate_depl != 8)
        {
            etat_automate_depl_prec = etat_automate_depl;
            ////Serial.println("ID_DBUG_ETAT_DPL");
            //remplirStruct(DATArobot,ID_DBUG_ETAT_DPL, 1, etat_automate_depl_prec, 0,0,0,0,0,0,0);
            //writeStructInCAN(DATArobot);                             //CAN           
        }
       #endif
       
        if (Fin_Match){
            liste.type = (TYPE_END_GAME);
            ////Serial.println("INSTRUCTION_END_MOTEUR");
            //On prévient qu'on s'est arrêté
            // le dlc original était de 2 avec un 0 en second octet
            //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 1, 0x04, 0,0,0,0,0,0,0);
            //writeStructInCAN(DATArobot);                             //CAN
            Fin_Match = 0;
        }
        else if (!asser_actif)
        {
            liste.type = (TYPE_ASSERVISSEMENT_DESACTIVE);
        }
          
        double cmdD, cmdG;
        switch(liste.type)
        {
            case (TYPE_END_GAME) :{
            Arret();
            break;
            }
            case (TYPE_DEPLACEMENT_IMMOBILE):{
            cmdD = Asser_Pos_MotD(roue_drt_init);
            cmdG = Asser_Pos_MotG(roue_gch_init);
            write_PWMD(cmdD);
            write_PWMG(cmdG);  
            
            break;
            }            
            case (TYPE_DEPLACEMENT_LIGNE_DROITE):{
            Message_Fin_Mouvement = ASSERVISSEMENT_RECALAGE;

            Mouvement_Elementaire(liste.distance, VMAX, AMAX, DMAX, MOUVEMENT_LIGNE_DROITE); 
            if (finMvtElem)
            {
                Serial.println("Fin mouvement");
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finMvtElem = 0;
                next_action = true;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);

            }
/*            else if (stop_receive)
            {
                liste.type = TYPE_INIT_STOP;
    //            write_PWMD(0);
    //            write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_ROTATION):{
            Message_Fin_Mouvement = ASSERVISSEMENT_ROTATION;
            Mouvement_Elementaire(liste.angle, VMAX/3, AMAX, DMAX, MOUVEMENT_ROTATION);
            if (finMvtElem){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finMvtElem = 0;
                next_action = true;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);
            }
            /*else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                write_PWMD(0);
                write_PWMG(0);
            }*/
            break;
            }
            case (TYPE_DEPLACEMENT_X_Y_THETA):{
            Message_Fin_Mouvement = ASSERVISSEMENT_XYT;
            X_Y_Theta(liste.x, liste.y, liste.theta, liste.sens, VMAX, AMAX); 
            if (finXYT){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finXYT = 0;
                next_action = true;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);

            }    
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //Arret_Brutal();
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_RAYON_COURBURE) :{
            Message_Fin_Mouvement = ASSERVISSEMENT_COURBURE;
            Rayon_De_Courbure(liste.rayon, liste.theta_ray, VMAX, AMAX, liste.sens, DMAX);
            if (finRayonCourbure){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finRayonCourbure = 0;
                next_action = true;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_TRAIT_ENCH_RCV):{
                
                while(liste.enchainement !=2);
                
                #if (F_DBUG_TRAIT_ETAT)
                ////Serial.println("ID_TRAIT");
                //remplirStruct(DATArobot,ID_TRAIT, 1, 0x01, 0,0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);                             //CAN
                ////CANenvoiMsg1Byte(ID_TRAIT, 1);
                #endif
                
                int compteurMvnt = 0;
                for(compteurMvnt = 0; compteurMvnt<nb_ordres; compteurMvnt++)
                {
                    #if (F_DBUG_TRAIT_ETAT)
                    ////Serial.println("ID_TRAIT");
                    //remplirStruct(DATArobot,ID_TRAIT, 2, 0x03, compteurMvnt,0,0,0,0,0,0);
                    //writeStructInCAN(DATArobot);                             //CAN
                    //////CANenvoiMsg2x1Byte(ID_TRAIT, 3, compteurMvnt);
                    #endif
                    
                    switch(liste.type)
                    {
                        case (TYPE_TRAIT_ENCH_RCV):
                        case (TYPE_DEPLACEMENT_LIGNE_DROITE_EN):
                            trait_Mouvement_Elementaire_Gene(&liste);
                            liste.vinit = liste.vfin;
                            break;
                        case (TYPE_DEPLACEMENT_RAYON_COURBURE_CLOTHOIDE):
                            trait_Rayon_De_Courbure_Clotho(&liste);
                            liste.vinit = liste.vinit;
                            break;
                        
                    }
                    
                }
                
                
                liste.type = (TYPE_DEPLACEMENT_LIGNE_DROITE_EN);
                etat_automate_depl = (INITIALISATION);
                
                #if (F_DBUG_TRAIT_ETAT)
                ////Serial.println("ID_TRAIT");
                //remplirStruct(DATArobot,ID_TRAIT, 1, 0x02, 0,0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);                             //CAN
                ////CANenvoiMsg1Byte(ID_TRAIT, 2);
                #endif
                    
            break;
            }            
            case (TYPE_DEPLACEMENT_LIGNE_DROITE_EN):{
            Message_Fin_Mouvement = (ASSERVISSEMENT_RECALAGE);
            Mouvement_Elementaire_Gene(liste);
            if (finMvtElem){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finMvtElem = 0;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_RAYON_COURBURE_CLOTHOIDE) :{
            Message_Fin_Mouvement = (ASSERVISSEMENT_COURBURE);
            Rayon_De_Courbure_Clotho(liste);
            if (finRayonCourbureClo){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finRayonCourbureClo = 0;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_RECALAGE) :{
            Message_Fin_Mouvement = (ASSERVISSEMENT_RECALAGE);
            Recalage(liste.distance, 3, 25, liste.recalage, liste.val_recalage); // ancienne Amax : 500
            if (finRecalage){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finRecalage = 0;
                next_action = true;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, ASSERVISSEMENT_RECALAGE, 0,0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);
            }
            /*else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                write_PWMD(0);
                write_PWMG(0);
            }*/
            break;
            }
            case (TYPE_ASSERVISSEMENT_DESACTIVE) :{
            write_PWMD(0);
            write_PWMG(0); 
            if (asser_actif == 1){
                Asser_Init();
                liste.type = (TYPE_DEPLACEMENT_IMMOBILE);
            }
            break;
            }
            case (TYPE_DEPLACEMENT_BEZIER):{
             //Première valeur de la courbe
            if(flagDebutBezier == 0)
            {
                Message_Fin_Mouvement = (ASSERVISSEMENT_BEZIER);
                //Recuperation des valeurs dans le buffer
                buf_circ_pop(&buffer_distanceD, &distanceD);
                buf_circ_pop(&buffer_distanceG, &distanceG);
                flagDebutBezier = 1;
                nbValeurs = 0;
            }
            
            
            //Ne récupère pas de valeurs tant qu'on a pas utilisé autant de valeurs que FACTEUR_DIVISION
            if(nbValeurs >= (FACTEUR_DIVISION))
            {
                //Recuperation des valeurs dans le buffer
                buf_circ_pop(&buffer_distanceD, &distanceD);
                buf_circ_pop(&buffer_distanceG, &distanceG);
                nbValeurs = 0;
            }
            
            //Il y a de la place dans les buffer, demande nouvelles valeurs
            if(buf_circ_free_space(&buffer_distanceG) > 0)
            {
                //L'envoi d'un ack provoque l'envoi d'une nouvelle valeur
                ////Serial.println("ACKNOWLEDGE_BEZIER");
                //remplirStruct(DATArobot,ACKNOWLEDGE_BEZIER,0,0,0,0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);
                ////CANenvoiMsg(ACKNOWLEDGE_BEZIER);
            }
            
            nbValeurs ++; //Permet de savoir le nombre de valeurs utilisées
            if(Courbe_bezier(distanceG/(FACTEUR_DIVISION), distanceD/(FACTEUR_DIVISION))) //Vrai si fin du mouvement
            {
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                flagDebutBezier = 0;
                //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                //writeStructInCAN(DATArobot);
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
                flagDebutBezier = 0;
            }
*/            break;
              }
            
                
         /*   case TYPE_MOUVEMENT_SUIVANT :{
            //Remise a zero des variables
            Asser_Init();
            
            //Reinitialisation etat automate des mouvements
            etat_automate_depl = INITIALISATION;
            
            //On conserve la position du robot pour la prochaine action
            roue_drt_init = lireCodeurD();
            roue_gch_init = lireCodeurG();
            
            ////CANenvoiMsg2x1Byte(INSTRUCTION_END_MOTEUR, Message_Fin_Mouvement, 0);
            
            //On vide le buffer de mouvements et on prévoit de s'asservir sur la position atteinte
            for(i = 0;i<nb_ordres;i++)  liste = (struct Ordre_deplacement){0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            nb_ordres = 0;
            cpt_ordre = 0;
            cpt = 0;
            break;
            }*/
            
            case TYPE_INIT_STOP :{
            // Arret_Brutal();
            //cpt_stop = 0;
            //etat_automate_depl = DECELERATION_TRAPEZE;
            //cpt=global_ta_stop;
            Message_Fin_Mouvement = ASSERVISSEMENT_STOP;
            liste.type = TYPE_STOP;
            //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
            //writeStructInCAN(DATArobot);
            break;
            }
            case TYPE_STOP :{
        /*    cpt_stop ++;
            if ((cpt_stop > 25)))
            {
                if(stop_receive==0)
                    liste.type = TYPE_MOUVEMENT_SUIVANT;
            }
       */     break;
            }
            default :{
              cmdD = Asser_Pos_MotD(roue_drt_init);
              cmdG = Asser_Pos_MotG(roue_gch_init);
              write_PWMG(cmdG);   
              write_PWMD(cmdD);
            break;
            }
        }
        
        if(liste.type == TYPE_MOUVEMENT_SUIVANT)
        {
        #if F_DBUG_ETAT && F_DBUG_ETAT_MS
            ////CANenvoiMsg1Byte(ID_DBUG_ETAT, liste.type); a faire
        #endif
        
            
            ////CANenvoiMsg4x1Byte(INSTRUCTION_END_MOTEUR, Message_Fin_Mouvement, 0, cpt_ordre, liste.enchainement); a faire
            ////CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16_t)Odo_theta) % 3600); a faire       
           
            if( liste.enchainement == 1)
            {
                //Remise a zero des variables
                //Asser_Init();
                //Reinitialisation etat automate des mouvements
                etat_automate_depl = INITIALISATION;
                
                //On conserve la position du robot pour la prochaine action
                
                consigne_pos = 0;
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                
                cpt_ordre ++;
                cpt = 0;
                
            }
            else
            {
                //Remise a zero des variables
                Asser_Init();
                consigne_vit = 0;
                consigne_pos = 0;
                //Reinitialisation etat automate des mouvements
                etat_automate_depl = INITIALISATION;
                
                //On conserve la position du robot pour la prochaine action
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                
                //On vide le buffer de mouvements et on prévoit de s'asservir sur la position atteinte
                liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                nb_ordres = 0;
                cpt_ordre = 0;
                cpt = 0;
            }
        }
    }
    else // on a recu un stop
    {
                Arret_Brutal();//Remise a zero des variables
                Asser_Init();
                consigne_vit = 0;
                consigne_pos = 0;
                //Reinitialisation etat automate des mouvements
                etat_automate_depl = INITIALISATION;
                etat_automate_xytheta = INIT_X_Y_THETA;
                //On conserve la position du robot pour la prochaine action
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                //On vide le buffer de mouvements et on prévoit de s'asservir sur la position atteinte
                liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                nb_ordres = 0;
                cpt_ordre = 0;
                cpt = 0;
    }
    
    //On calcule l'odométrie à chaque tour de boucle
    //Odometrie();   
}


/***************************************************************************************
 NOM : CANloop                                                              
 ARGUMENT : aucun                                        
 RETOUR : rien                                                                        
 DESCRIPTIF :   Fonction principale appelé periodiquement, qui gère la communication avec le CAN et le Bluetooth
                Chaque id de message reçu, une tâche associé 
***************************************************************************************/

void CANloop(){
    static int FIFO_lecture=0,FIFO_occupation=0,FIFO_max_occupation=0, etat_evitement = 3;

    if(WaitForJack){//On attend alors le debut du match
        if(jack()){
            Serial.println("Game start");
            WaitForJack = false;
            next_action = true;
        }
        else{
            return;
        }
    }

    FIFO_occupation=SIZE_ACTION-FIFO_lecture;
    if(FIFO_occupation<0){FIFO_occupation=FIFO_occupation+SIZE_ACTION;}
    if(FIFO_max_occupation<FIFO_occupation){FIFO_max_occupation=FIFO_occupation;}//Ajouter des conditions : attendre que l'action est été faite

    

    lidar_loop();

    switch (etat_evitement)
    {
    case 0:{
        if(lidarStatus()){//Alors s'arreter)   //&& (type_Evitement != ETAT_GAME_PAS_DANGER) //Enlever car bug des fois
            Serial.println("Lidar detection");
            stop_receive = 1; //Arret brutal
            etat_evitement = 1;
            mscount_lidar = 0;
            return;
        }
    }
        break;
    case 1:{
        if(mscount_lidar>10000){//Attendre une seconde
            etat_evitement = 2;
        }
        return;
    }
        break;
    case 2:{
        if(lidarStatus()){//Alors s'arreter
            stop_receive = 1; //Arret brutal
            etat_evitement = 1;
            mscount_lidar = 0;
            return;
        }else{//On reprend
                etat_evitement = 0;
            // `#START MESSAGE_X_Y_Theta_RECEIVED` 
                    stop_receive = 0;

                    //On vide le buffer de mouvements
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;//ne sert à rien mais au cas où, au futur...

                    liste.x = target_x;
                    liste.y = target_y;

                    liste.theta = target_theta;
                    liste.sens =  target_sens;
                    liste.type = TYPE_DEPLACEMENT_X_Y_THETA;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;   
                    type_Evitement = ETAT_GAME_MVT_DANGER;
                    return;    
        }
    }
        break;
    case 3 :
    //Ne rien faire, Lidar desactivé pendant la phase d'initialisation et s'active dés quon tire le Jack
    break;
    default:
        break;
    }

    
    if(!next_action){return;}
    next_action = false;
    Serial.print(FIFO_lecture);  Serial.print(" - Action : ");  Serial.print(strategie_hackaton[FIFO_lecture].ACTION); 
   //if(canAvailable || BtAvailable){
    // if(canAvailable){canReadExtRtr();}//On le me ici pour ne pas surcharger l'interruption CAN.onRecveive
    // canAvailable = false; BtAvailable = false;
    ////Serial.println("CAN received");
    if(!FIFO_occupation){return;}
    switch (strategie_hackaton[FIFO_lecture].ACTION)
    {
            case WAIT_FOR_JACK ://Debut du Match dés qu'on tire le Jack
                WaitForJack = true;
                etat_evitement = 0;//On active le lidar
            break;

            case ASSERVISSEMENT_ENABLE :
                asser_actif = strategie_hackaton[FIFO_lecture].variable; //Si 1 actif sinon pas
                if(asser_actif == 1)
                {
                    roue_drt_init = lireCodeurD();
                    roue_gch_init = lireCodeurG();
                }
                //Serial.println("ASSERVISSEMENT_ENABLE");
            break;

            case ASSERVISSEMENT_XYT :
            {
                    // `#START MESSAGE_X_Y_Theta_RECEIVED` 
                    stop_receive = 0;

                    //On vide le buffer de mouvements
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;//ne sert à rien mais au cas où, au futur...

                    liste.x = strategie_hackaton[FIFO_lecture].x;
                    liste.y = strategie_hackaton[FIFO_lecture].y;

                    liste.theta = strategie_hackaton[FIFO_lecture].theta;
                    liste.sens =  strategie_hackaton[FIFO_lecture].sens;
                    liste.type = TYPE_DEPLACEMENT_X_Y_THETA;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;       
                    
                    target_x       =  strategie_hackaton[FIFO_lecture].x;
                    target_y       =  strategie_hackaton[FIFO_lecture].y;
                    target_theta   =  strategie_hackaton[FIFO_lecture].theta;
                    target_sens    =  strategie_hackaton[FIFO_lecture].sens;
                    type_Evitement = ETAT_GAME_MVT_DANGER;
            }
            break;
        
            case ASSERVISSEMENT_ROTATION:
            {
                // `#START MESSAGE_Rotation_RECEIVED`
                stop_receive = 0;

                liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                nb_ordres = 0;
                cpt_ordre = 0;

                int16_t angle = strategie_hackaton[FIFO_lecture].angle;
        
                liste.type = TYPE_DEPLACEMENT_ROTATION;
                liste.angle = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * angle / (3600 * PERIMETRE_ROUE_CODEUSE);
                liste.vmax = VMAX;
                liste.amax = AMAX;

                target_x       = (uint16_t)Odo_x;
                target_y       = (uint16_t)Odo_y;
                target_theta   = (int16_t)Odo_theta + angle;
                target_sens    = 0;
                type_Evitement = ETAT_GAME_PAS_DANGER;

            }
            break;  
            case ASSERVISSEMENT_RECALAGE:
            {
                stop_receive = 0;

                int16_t distance = strategie_hackaton[FIFO_lecture].distance;
                uint8_t mode = strategie_hackaton[FIFO_lecture].mode;
                int16_t valRecalage = strategie_hackaton[FIFO_lecture].val_recalage; //Valeur que devra prendre l'encodeur sur l'axe designé
                
                target_x       = (uint16_t)Odo_x;
                target_y       = (uint16_t)Odo_y;
                target_theta   = (int16_t)Odo_theta;
                target_sens    = (distance >=0 ? -1 : 1);
                type_Evitement = ETAT_GAME_PAS_DANGER;
                 
                Serial.println("ASSERVISSEMENT_RECALAGE");
                //Recalage
                if (mode)
                {
                    Serial.println("Recalage en cours ...");
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;
                    liste.type = TYPE_DEPLACEMENT_RECALAGE;
                    liste.distance = ((distance * RESOLUTION_ROUE_CODEUSE) / PERIMETRE_ROUE_CODEUSE); //-
                    liste.vmax = 10;
                    liste.amax = 100;
                    liste.enchainement = 0;
                    liste.val_recalage = valRecalage;
                    liste.recalage = mode;

                }
                //Ligne droite
                else
                {          
                    //On vide le buffer de mouvements
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;
                    liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE;
                    liste.distance = (distance * RESOLUTION_ROUE_CODEUSE) / PERIMETRE_ROUE_CODEUSE;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;
                    //liste.enchainement = enchainement;
                    target_x       = (uint16_t)Odo_x + distance * cos(Odo_theta * M_PI / 1800.0);
                    target_y       = (uint16_t)Odo_y + distance * sin(Odo_theta * M_PI / 1800.0);
                    target_theta   = (uint16_t)Odo_theta;
                    target_sens    = (distance >=0 ? 1 : -1);
                    type_Evitement = ETAT_GAME_MVT_DANGER;
                }
            }
            break;
              
            // case ODOMETRIE_SMALL_POSITION:
            // {
            //     Odo_x = (strategie_hackaton[FIFO_lecture].dt[1] << 8) | strategie_hackaton[FIFO_lecture].dt[0];
            //     Odo_y = (strategie_hackaton[FIFO_lecture].dt[3] << 8) | strategie_hackaton[FIFO_lecture].dt[2];
            //     Odo_theta = (strategie_hackaton[FIFO_lecture].dt[5] << 8) | strategie_hackaton[FIFO_lecture].dt[4];
            // }
            // break;
           
            case GLOBAL_GAME_END:
            {
                 // `#START MESSAGE_End_Game_RECEIVED` 
                Fin_Match = 1;
                type_Evitement = ETAT_GAME_PAS_DANGER;
            }
            break;
            case ASSERVISSEMENT_STOP:
            {
                //`#START MESSAGE_Stop_RECEIVED` 
                stop_receive = 1;
            }
            break;  
            case CHECK_MOTEUR:
            {
                //`#START MESSAGE_Check_RECEIVED` 
                attente = 1;
            }
            break;

            case SERVOMOTEUR_9G:{

            }break;

            case SERVOMOTEUR_PINCE:{
                bool pos_servo = (strategie_hackaton[FIFO_lecture].position_servo == 1 ? true : false);
                PINCE(pos_servo);
                next_action = true;
                type_Evitement = ETAT_GAME_PAS_DANGER;
            }break;
            
            default :
            break;
    }
    FIFO_lecture ++;
    // FIFO_lecture %= SIZE_ACTION;
    // FIFO_lecture=(FIFO_lecture+1)%SIZE_ACTION;
    Serial.println(FIFO_lecture);
}   

//*/
/***************************************************************************************
 NOM : Odometrie                                                                      
 ARGUMENT : rien                                                                      
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction qui calcul la position et l'orientation du robot               
***************************************************************************************/
void Odometrie(void)//fait
{
    
    //Declaration des variables
    double dist, ang;   //distance, angle
    
    //Recuperation des valeurs des compteurs des encodeurs  
    Odo_val_pos_D = lireCodeurD();
    Odo_val_pos_G = lireCodeurG();
    double erreur = Odo_val_pos_D - Odo_val_pos_G;
    //Calcul de la distance parcourue
    dist = 0.5*((Odo_val_pos_D - Odo_last_val_pos_D) + (Odo_val_pos_G - Odo_last_val_pos_G));

    //Serial.print("Odo_val_pos_G :"); Serial.print(Odo_val_pos_G); Serial.print("  Odo_val_pos_D :");  Serial.println(Odo_val_pos_D); 
      
    //Calcul de la valeur de l'angle parcouru
    ang = (((Odo_val_pos_D - Odo_last_val_pos_D) -(Odo_val_pos_G - Odo_last_val_pos_G))*1800.0*PERIMETRE_ROUE_CODEUSE/(LARGEUR_ROBOT*M_PI*RESOLUTION_ROUE_CODEUSE));
    
    //Determination de la position sur le terrain en X, Y, Theta
    Odo_theta +=  ang;
    Odo_x += dist*cos((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    Odo_y += dist*sin((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    

    
    //Stockage de la derniere valeur de l'odometrie
    Odo_last_val_pos_D = Odo_val_pos_D;
    Odo_last_val_pos_G = Odo_val_pos_G;
    
    /**/
    
    /**/
    mscount1 ++;    
    //Condition d'envoi des informations de l'odometrie par CAN 
    if(mscount1 >= (500/TE_100US))//toutes les 50ms
    {
        // digitalWrite(27, set);//pour mesurer le temps de mscount1 avec l'oscilloscope
        // set = !set;
        mscount1 = 0;
        // //Serial.println();
         //Serial.printf("Odo_val_pos_D : %lf ; Odo_val_pos_G : %lf ; Odo_val_pos_D - Odo_val_pos_G : %lf\n", Odo_val_pos_D, Odo_val_pos_G, erreur);
        // Serial.print    ("Odo_x : "); Serial.print(     Odo_x); Serial.print(", ");
        // Serial.print    ("Odo_y : "); Serial.print(     Odo_y); Serial.print(", ");
        // Serial.print("Odo_theta : "); Serial.print( Odo_theta); Serial.println();
        // //CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16_t)Odo_theta) % 3600);
        ////CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16)Odo_theta) % 3600);
        ////CANenvoiMsg3x2Bytes(DEBUG_ASSERV, QuadDec_D_GetCounter(), QuadDec_G_GetCounter(), consigne_pos);

    }
    
        
    
}  

/* [TYPE] [ARG1] [ARG2] [ARG3] [ARG4]*/
/*  X : XYT [x];[y];[t];
    L : Ligne droite [d];;;
    R : Rotation [a];;;
    O : Recalage [d];[x ou y?] si x mettre 1, si y -1 ;;

    A : Action [Type] si Type à 1 c'est servomoteur 9G, 2 Servo pince ; [Position servo] 0 : retracter, 1 : Lacher ;;
*/
#define TYPE 0
#define ARG1 1
#define ARG2 2
#define ARG3 3
#define ARG4 4
void remplirStructStrat(){
    Serial.println ("remplirStructStrat... ***********************************");
    for(int act = 0; act< SIZE_ACTION; act++){
        switch (tab_action_brut[act][TYPE])
        {
        case 'J'://Jack
        {
            Serial.print (act); Serial.print (" - Jack ");
            //mettre WaitForJack a true
            strategie_hackaton[act].ACTION = WAIT_FOR_JACK;
            Serial.println (WAIT_FOR_JACK);
        }
        break;
        case 'L'://Ligne droite
        {
            Serial.print (act); Serial.print (" - Ligne droite  ");
            strategie_hackaton[act].ACTION = ASSERVISSEMENT_RECALAGE;
            strategie_hackaton[act].distance = tab_action_brut[act][ARG1];
            strategie_hackaton[act].mode = 0;

            
            Serial.print (ASSERVISSEMENT_RECALAGE); Serial.print (" - "); Serial.println (strategie_hackaton[act].distance);
        }
            break;
        case 'R'://rotation
        {
            Serial.print (act); Serial.print (" - rotation  ");
            strategie_hackaton[act].ACTION = ASSERVISSEMENT_ROTATION;
            if(COULEUR_STRAT == JAUNE){//Alors on inverse
                strategie_hackaton[act].angle = -1 *tab_action_brut[act][ARG1];
            }
            else{//Couleur de base BLEU
                strategie_hackaton[act].angle = tab_action_brut[act][ARG1];
            }

            Serial.print (ASSERVISSEMENT_ROTATION); Serial.print (" - "); Serial.println (strategie_hackaton[act].angle);
        }
            break;
        case 'X'://XYT
        {
            Serial.print (act); Serial.print (" - XYT  ");
            strategie_hackaton[act].ACTION = ASSERVISSEMENT_XYT;
            uint16_t x,y,theta;
            x = tab_action_brut[act][ARG1];
            y = tab_action_brut[act][ARG2];
            theta = tab_action_brut[act][ARG3];
            strategie_hackaton[act].sens = tab_action_brut[act][ARG4];

            if(COULEUR_STRAT == JAUNE){
                x     = x;
                y     = 3000 - y;
                theta = - theta;
                
            }

            strategie_hackaton[act].x     = x;
            strategie_hackaton[act].y     = y;
            strategie_hackaton[act].theta = theta; 

            Serial.print (ASSERVISSEMENT_XYT); Serial.print (" - ");
            Serial.print(strategie_hackaton[act].x); Serial.print(" "); 
            Serial.print(strategie_hackaton[act].y); Serial.print(" "); 
            Serial.println(strategie_hackaton[act].theta);
        }
            break;
        case 'O'://Recalage
        {
            Serial.print (act);  Serial.print (" - Recalage  ");
            strategie_hackaton[act].ACTION = ASSERVISSEMENT_RECALAGE;
            strategie_hackaton[act].distance = tab_action_brut[act][ARG1];
            strategie_hackaton[act].mode = tab_action_brut[act][ARG2];

            strategie_hackaton[act].val_recalage = tab_action_brut[act][ARG3];//Bas droit sur l'assiette bleu, de base
            //Puis
            //bas Gauche sur l'assiette jaune 
            if((RECALAGE_DEPART == EN_BAS_A_GAUCHE) && tab_action_brut[act][ARG2] <0){//Aussi si on se recale sur l'axe y
                strategie_hackaton[act].val_recalage = 3000 - tab_action_brut[act][ARG3];
            }
            //Ajouter les cas en haut à droite et en haut à gauche
            
            

            Serial.print (ASSERVISSEMENT_RECALAGE); Serial.print (" - "); 
            Serial.print (strategie_hackaton[act].distance); Serial.print (" - "); 
            Serial.print (strategie_hackaton[act].mode); Serial.print (" - "); 
            Serial.println (strategie_hackaton[act].val_recalage);
        }
        break;
        case 'A'://Action servo
        {
            Serial.print (act); Serial.print (" - Action servo  ");
            int ACTION = SERVOMOTEUR_9G;
            if(tab_action_brut[act][ARG1]== 2){
                ACTION = SERVOMOTEUR_PINCE;
            }

            strategie_hackaton[act].ACTION = ACTION;
            strategie_hackaton[act].position_servo = tab_action_brut[act][ARG2];

            Serial.print (ACTION); Serial.print (" - "); Serial.println (strategie_hackaton[act].position_servo);
        }
            break;
        default:
            break;
        }
    }
    Serial.println ("*************************************************");
}



void test_accel(void)//fonctionne
{
    static int etat_test_accel = 0;
    static double posG, posD, lposG, lposD;
    double accel_test;
    static int cpt_test_accel = 0;
    switch(etat_test_accel)
    {
    case 0:
        write_PWMD(8*250/10);
        write_PWMG(8*250/10);
        etat_test_accel = 1;
        break;
        
    case 1:
        posD = lireCodeurD();      //Recuperation de la valeur du compteur incrémental
        posG = lireCodeurG();      //Recuperation de la valeur du compteur incrémental
        
        accel_test = posG - lposG;
        //Serial.printf("accel test : %lf\n", accel_test);
        // DATArobot.ID = ID_TEST_VITESSE;
        // DATArobot.dt[0] = accel_test;
        // DATArobot.ln = 1;
       // //writeStructInCAN(DATArobot); 
        
        
        lposD = posD;
        lposG = posG;
        
        cpt_test_accel++;
        if(cpt_test_accel > 160){
        etat_test_accel = 2;
        }
        break;
        
    case 2:
        write_PWMD(8*250/10);
        write_PWMG(8*250/10);
        etat_test_accel = 3;
        break;
        
    case 3:
        posD = lireCodeurD();      //Recuperation de la valeur du compteur incrémental
        posG = lireCodeurG();      //Recuperation de la valeur du compteur incrémental
        
        accel_test = posG - lposG;
        //Serial.printf("accel test : %lf\n", accel_test);
        // DATArobot.ID = ID_TEST_VITESSE;
        // DATArobot.dt[0] = accel_test;
        // DATArobot.ln = 1;
       // //writeStructInCAN(DATArobot); 

        
        lposD = posD;
        lposG = posG;
        
        break;
    }
}


bool jack(){
    return(!digitalRead(PIN_JACK));
}