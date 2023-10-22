#include <Arduino.h>
#include <CRAC_utility.h>
#include <buffer_circulaire.h>//inutile
#include "math.h"
#include <mouvement.h>
#include <timerAsserBas.h>
#include <moteur.h>



//----------------------------------------------------------------------Variables
volatile uint16_t          mscount = 0,      // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                         mscount1 = 0,     // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                         mscount2 = 0;     // Compteur utilisé pour envoyer la trame CAN d'odométrie

extern double Odo_val_pos_D, Odo_val_pos_G, Odo_last_val_pos_D, Odo_last_val_pos_G;

int nbValeurs = 0;

double Kp =0.2   , Ki =0, Kd =0.5;//double Kp =8    , Ki =0.4, Kd =10.0;
                
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
void TempsEchantionnage(int TIME);
//Fonctions principales : 
void calcul(void);
// void CANloop();
void Odometrie(void);

void setup() {
  Serial.begin(921600);
  init_coef();

  // Encodeur_Init(); Serial.println("fin encodeur init");

  
  AsserInitCoefs(Kp, Ki, Kd);
  Asser_Init();
  //Interruption : le programme est cadence grâce a l'interrutpion du Timer
  init_Timer(); Serial.println("fin init Timer");
  
  Serial.println("fin setup");

    Moteur_Init(); Serial.println("fin moteur init");
  liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE;//test
  liste.distance = 1000;
  mscount = 0;

}

void loop() {

//   Serial.print(cmdD); Serial.print(" - ");  Serial.print(Odo_val_pos_D); Serial.print(" - "); Serial.println(Odo_val_pos_G); 
  calcul();
  Odometrie();

  TempsEchantionnage(TE_100US);  
}

void TempsEchantionnage(int TIME){
    if (mscount >= TIME) 
  {   
    Serial.println("erreur temp calcul");
    Serial.println(mscount);
    //remplirStruct(DATArobot,ERREUR_TEMP_CALCUL,2, mscount,TE_100US,0,0,0,0,0,0);
    //writeStructInCAN(DATArobot);
  }
  else 
  {
    while (mscount<(TIME));
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
                ou lorsque le match est terminé. En outre, la fonction envoie des messages de débogage à l'aide de CANenvoiMsg1Byte(), 
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
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finMvtElem = 0;
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
                //CANenvoiMsg1Byte(ID_TRAIT, 1);
                #endif
                
                int compteurMvnt = 0;
                for(compteurMvnt = 0; compteurMvnt<nb_ordres; compteurMvnt++)
                {
                    #if (F_DBUG_TRAIT_ETAT)
                    ////Serial.println("ID_TRAIT");
                    //remplirStruct(DATArobot,ID_TRAIT, 2, 0x03, compteurMvnt,0,0,0,0,0,0);
                    //writeStructInCAN(DATArobot);                             //CAN
                    ////CANenvoiMsg2x1Byte(ID_TRAIT, 3, compteurMvnt);
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
                //CANenvoiMsg1Byte(ID_TRAIT, 2);
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
                //CANenvoiMsg(ACKNOWLEDGE_BEZIER);
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
            
            //CANenvoiMsg2x1Byte(INSTRUCTION_END_MOTEUR, Message_Fin_Mouvement, 0);
            
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
            //CANenvoiMsg1Byte(ID_DBUG_ETAT, liste.type); a faire
        #endif
        
            
            //CANenvoiMsg4x1Byte(INSTRUCTION_END_MOTEUR, Message_Fin_Mouvement, 0, cpt_ordre, liste.enchainement); a faire
            //CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16_t)Odo_theta) % 3600); a faire       
           
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

    
      
    //Calcul de la valeur de l'angle parcouru
    ang = (((Odo_val_pos_D - Odo_last_val_pos_D) -(Odo_val_pos_G - Odo_last_val_pos_G))*1800.0*PERIMETRE_ROUE_CODEUSE/(LARGEUR_ROBOT*M_PI*RESOLUTION_ROUE_CODEUSE));
    
    //Determination de la position sur le terrain en X, Y, Theta
    Odo_theta +=  ang;
    Odo_x += dist*cos((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    Odo_y += dist*sin((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    //Serial.print(Odo_val_pos_G); Serial.print("  ");  Serial.println(Odo_val_pos_D); 

    
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
         //Serial.println("Odo_val_pos_D : %lf ; Odo_val_pos_G : %lf ; Odo_val_pos_D - Odo_val_pos_G : %lf\n", Odo_val_pos_D, Odo_val_pos_G, erreur);

        // CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16_t)Odo_theta) % 3600);
        //CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16)Odo_theta) % 3600);
        //CANenvoiMsg3x2Bytes(DEBUG_ASSERV, QuadDec_D_GetCounter(), QuadDec_G_GetCounter(), consigne_pos);

    }
    
        
    
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
