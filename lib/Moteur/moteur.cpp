#include "moteur.h"
#include <CRAC_utility.h>





void Moteur_Init(){
  //init pins carte de puiss
  pinMode(inApin_MOTG, OUTPUT);
  pinMode(inApin_MOTD, OUTPUT);
  pinMode(inBpin_MOTG, OUTPUT);
  pinMode(inBpin_MOTD, OUTPUT);
  pinMode(PWM_MOTG, OUTPUT);
  pinMode(PWM_MOTD, OUTPUT);

  //mise à l'arret
  digitalWrite(inApin_MOTG, LOW);
  digitalWrite(inApin_MOTD, LOW);
  digitalWrite(inBpin_MOTG, LOW);
  digitalWrite(inBpin_MOTD, LOW);
  //init pwm
  setupPWM(PWM_MOTD, 60000);
  setupPWM(PWM_MOTG, 60000);
}


//----------------------------------------------------------------------autres fonctions
/****************************************************************************************/
/* NOM : setupPWM                                                                       */
/* ARGUMENT : Pins de sortie et channel des PWM                                         */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet de setup les canaux de PWM et de les faire sortir sur les pins   */
/****************************************************************************************/
void setupPWM(int PWMpin, int frequency)
{
  // Vérifiez si la broche est compatible PWM sur Arduino Mega
  if (PWMpin == 2 || PWMpin == 3 || PWMpin == 5 || PWMpin == 6 || PWMpin == 7 || PWMpin == 8 || PWMpin == 9 || PWMpin == 10 || PWMpin == 11)
  {
    // Calculer la valeur de comparaison (OCRxx) en fonction de la fréquence
    long pwmFrequency = 16000000L / 8; // Fréquence d'horloge / 8 (en raison de la division du timer)
    pwmFrequency /= frequency;
    pwmFrequency -= 1;
    
    // Configurer le mode PWM sur la broche spécifiée
    pinMode(PWMpin, OUTPUT);
    
    // Configurer le canal PWM (utilisation du timer 1 pour les broches 9 et 10, et du timer 3 pour les broches 2, 3, 5, 6, 11, et 12)
    if (PWMpin == 9 || PWMpin == 10)
    {
      TCCR1A |= (1 << COM1A1) | (1 << WGM11);
      TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
      OCR1A = pwmFrequency;
    }
    else if (PWMpin == 2 || PWMpin == 3 || PWMpin == 5 || PWMpin == 6 || PWMpin == 11 || PWMpin == 12)
    {
      TCCR3A |= (1 << COM3A1) | (1 << WGM31);
      TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31);
      OCR3A = pwmFrequency;
    }
  }
  else
  {
    // La broche spécifiée n'est pas compatible PWM sur Arduino Mega
    Serial.println("Erreur : Cette broche n'est pas compatible PWM sur Arduino Mega");
  }
}
