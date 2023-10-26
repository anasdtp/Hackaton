#include <Arduino.h>

extern char vitesse_danger, Stop_Danger, asser_actif, attente, mode_xyt,
                finMvtElem, finRecalage, finRayonCourbure,finRayonCourbureClo, finXYT,  Fin_Match, Message_Fin_Mouvement, explosionErreur; 
extern int nb_ordres;
// Tout plein de flags
extern short           etat_automate, etat_automate_depl, new_message,
                xytheta_sens, next_move_xyt, next_move, i, stop, stop_receive, param_xytheta[3],
                etat_automate_xytheta, ralentare;

                
//Fonctions principales :
void Mouvement_Elementaire(long pcons, short vmax, short amax, short dmax, char mvt);
void Rayon_De_Courbure(short rayon, short theta, short vmax, short amax, short sens, short dmax);
void X_Y_Theta(long px, long py, long ptheta, long sens, short vmax, short amax);
void Recalage(long pcons, short vmax, short amax, short dir, short nv_val);

void init_coef();

//Fonctions jamais testées et que je ne comprends pas en vrai :
extern float distanceG, distanceD;
void trait_Mouvement_Elementaire_Gene(struct Ordre_deplacement* monDpl);
void trait_Rayon_De_Courbure_Clotho(struct Ordre_deplacement* monDpl);
void Mouvement_Elementaire_Gene(struct Ordre_deplacement monDpl);
void Rayon_De_Courbure_Clotho(struct Ordre_deplacement monDpl);
int Courbe_bezier(double distanceG, double distanceD);