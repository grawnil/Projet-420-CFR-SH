/**
 * @file   		    joystick_emulator.h
 * @author 	      Jacob Deschamps
 * @date          Mars 2022
 * @brief         Fichier contenant la définition des touches sous forme de macros, la définition de la classe "Joy_emulator", etc.
 * @version 1.0 : Première version (1.0)
 *
 * Environnement :
 * 	Développement : Éditeur de code/texte sous Linux
 * 	Compilateur : gcc (ou g++) et catkin_make
 *  Simulateur : ROS (stage)
 */

#ifndef joystick_emulator_H
#define joystick_emulator_H

#include "ros/ros.h" // Permet d'inclure la plupart des fichiers d'entêtes nécessaires à l'utilisation des fonctionnalités de ROS.
#include <termios.h> // Pour appliquer certains configurations au terminal via lequel le noeud ROS sera lancé.
#include "boost/thread/mutex.hpp" /* Pour pouvoir utiliser des threads et des mutex dans ce programme. On peut donc rouler des
                                     processus en parallèle (ex : détecter en console la touche que l'utilisateur aura enfoncée
                                     et vérifier en parallèle si une touche est maintenue enfoncé si si, encore, aucune touche
                                     n'a de nouveau été enfoncé dans un certain intervalle de temps).*/

// Codes hexadécimaux reçus lorsque les touches correspondantes sont enfoncées sur le clavier.
#define LETTER_E 0x65
#define LETTER_S 0x73
#define LETTER_X 0x78
#define ARROW_UP 0x41
#define ARROW_DOWN 0x42
#define ARROW_RIGHT 0x43
#define ARROW_LEFT 0x44
#define SIGN_PLUS 0x2B // Touche [+] sur le clavier numérique ou touches [SHIFT] et [=] sur le clavier alphanumérique.
#define SIGN_MINUS 0x2D

// Classe de la manette de jeu ("joystick").
class Joy_emulator {
  private:
    float linear_, angular_; // Indique si un déplacement linéaire ou angulaire doit être effectué.
    int enable_, spin_, accel_, avoid_; /* Sert à activer ou à désactiver la transmission d'informations de la manette
                                           (pour la variable "enable_"). Permet de faire tourner le robot sur lui-même
                                           (pour la variable "spin_"). Indique si la vitesse de déplacement du robot
                                           doit être accrue ou diminuée (pour la variable "accel_"). Permet d'activer
                                           ou de désactiver le mode d'évitement des collisions (pour la variable
                                           "avoid_"). */
    ros::NodeHandle nh_;
    ros::Time first_publish_;
    ros::Time last_publish_;
    ros::Publisher joy_pub_;
    boost::mutex publish_mutex_;
    
    void publish(float, float, int); // Fonction permettant de publier les touches enfoncées
  
  public:
    Joy_emulator(); // Constructeur par défaut de la classe "Joy_emulator".
    void keyLoop(); // Fonction permettant de détecter les touches enfoncées en console pendant l'exécution du programme.
    void watchdog(); /* Fonction analysant l'intervalle de temps entre deux touches enfoncées consécutivement afin de
                        déterminer s'il s'agit de la même touche maintenue enfoncée ou si, encore, aucune touche n'a été
                        enfoncée dans cet intervalle de temps. */
};

// Les variables globales ci-dessous servent à gérer le terminal à partir duquel a été exécuté le programme courant.
int kfd = 0;
struct termios cooked, raw;

// Prototype de la fonction quit(). Les autres fonctions ont leur prototype déjà déclaré dans la classe "Joy_emulator".
void quit(int);

#endif // joystick_emulator_H