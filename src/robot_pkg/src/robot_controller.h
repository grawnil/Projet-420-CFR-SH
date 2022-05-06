/**
 * @file   		    robot_controller.cpp
 * @author 	      Jacob Deschamps
 * @date          Mars 2022
 * @brief         Fichier contenant la définition de la classe "Robot_motion", les prototypes de fonctions et les macros.
 * @version 1.0 : Première version (1.0)
 *
 * Environnement :
 * 	Développement : Éditeur de code/texte sous Linux
 * 	Compilateur : gcc (ou g++) et catkin_make
 *  Simulateur : ROS (stage)
 */

#ifndef robot_controller_H
#define robot_controller_H

#include "ros/ros.h" // Permet d'inclure la plupart des fichiers d'entêtes nécessaires à l'utilisation des fonctionnalités de ROS.
#include "sensor_msgs/Joy.h" // Permet d'utiliser des messages ayant pour type "sensor_msgs/Joy".
#include "sensor_msgs/LaserScan.h" // Permet d'utiliser des messages ayant pour type "sensor_msgs/LaserScan".

#define ROBOT_LINEAR_MIN_SPEED 0.2f // La vitesse linéaire minimale pour le robot est de 0.2 m/s.
#define ROBOT_LINEAR_MAX_SPEED (ROBOT_LINEAR_MIN_SPEED + 2.0f) // La vitesse linéaire maximale pour le robot est de 2.0 m/s.
#define ROBOT_ANGULAR_MIN_SPEED 0.1f // La vitesse linéaire minimale pour le robot est de 0.1 rad/s (360 degrés = 2*PI radians).
#define ROBOT_ANGULAR_MAX_SPEED (ROBOT_ANGULAR_MIN_SPEED + 1.0f) // La vitesse angulaire maximale pour le robot est de 1.0 rad/s.
#define ROBOT_SPEED_STEP 0.2f // L'augmentation ou la diminution de vitesse se fait par intervalle de 0.2 m/s ou rad/s.

#define JOY_MIN_CMD -1 // La plus petite commande reçue de la manette est le chiffre -1.
#define JOY_MAX_CMD 1 // La plus grande commande reçue de la manette  est le chiffre 1.

#define SCAN_MIN_INDEX 70
/* Le robot voit jusqu'à 20 degrés sur sa gauche (90 - 20 = 70). Cela permet de restreindre le champ de vision du laser qui scanne sur
   un rayon de 180 degrés devant le robot pour détecter les obstacles. */

#define SCAN_MAX_INDEX 110 // Le robot voit jusqu'à 20 degrés sur sa droite (90 + 20 = 110).
#define SCAN_DISTANCE_STOP 0.4f // Distance (en m) entre le robot et un obstacle à laquelle le robot doit s'arrêter d'avancer.
#define SCAN_DISTANCE_SLOW (SCAN_DISTANCE_STOP + 0.6f) // Distance (en m) entre le robot et un obstacle à laquelle il faut ralentir.

// Classe pour contrôler le robot en simulation.
class Robot_control {
  private:
    float linear_vel_; // Vitesse linéaire du robot devant se trouver entre "ROBOT_LINEAR_MIN_SPEED" et "ROBOT_LINEAR_MAX_SPEED".
    float angular_vel_; // Vitesse angulaire du robot devant se trouver entre "ROBOT_ANGULAR_MIN_SPEED" et "ROBOT_ANGULAR_MAX_SPEED".
    ros::NodeHandle nh_; // Objet permettant  d'avertir "roscore" que les vitesses du robot seront éventuellement publiées.
    ros::Publisher vel_pub_; // Objet permettannt de publier les nouvelles vitesses du robot.

  public:
    // Énumération indiquant les états possibles du robot.
    enum Robot_state {
      good, // Le robot progresse sans problème (rien à signaler).
      slow, /* Le robot a détecté un obstacle se situant entre "SCAN_DISTANCE_STOP" et "SCAN_DISTANCE_SLOW". Comme il n'est pas
               encore trop proche de cet obstacle (il n'est pas à "SCAN_DISTANCE_STOP"), il doit commencer à ralentir. */
      stop // Le robot a atteint la distance "SCAN_DISTANCE_STOP" entre lui et l'obstacle détecté. Il doit arrêter d'avancer.
    };

    // Structure qui contiendra les données reçues de la manette de jeu.
    struct Joystick_data {
      float linear_, angular_; // Indique si un déplacement linéaire (1) ou angulaire (-1) doit être effectué.
      int accel_, spin_, avoid_; /* Indique si les vitesses (autant linéaire qu'angulaire) doivent être accrues ou diminuées
                                    (dans le cas de la variable "accel_". Indique si le robot doit entrer dans le mode "spin"
                                    (mode dans lequel le robot se met à tourner sur lui-même (peu importe le sens de rotation)
                                    et ce, dans le cas de la variable "spin_"). Indique si le mode "évitement des collisions"
                                    doit être activé ou non (ce mode permet au robot de ne jamais entrer en contact avec un
                                    obstacle se trouvant devant lui et, donc, de ne jamais rester coincé). */
    };

    // Structure qui contiendra les données reçues du laser.
    struct Laser_data {
      float max_range_, ranges_[SCAN_MAX_INDEX - SCAN_MIN_INDEX + 1];
      /* La variable "max_range_" contient la distance maximale que le laser peut percevoir (limite de son rayon de lecture) et
      	 le vecteur "ranges_[]" contient toutes les lectures du laser entre les angles "SCAN_MIN_INDEX" et "SCAN_MAX_INDEX". */
    };

    Robot_state robot_state_; // Variable contenant l'état du robot selon l'énumération "Robot_state".
    struct Joystick_data joy_data_; // Variable permettant d'accéder à la structure "Joystick_data".
    struct Laser_data scan_data_; // Variable permettant d'accéder à la structure "Laser_data".
    
    /* Les "getters" et les "setters" suivants permettent d'accéder aux vitesses linéaire et anguaire du robot et de changer
       leur valeur respective. Ces fonctions sont définies à même la classe "Robot_control", car elles ne comprennent chacune
       qu'une seule ligne de code et elles s'inscrivent donc dans la catégorie des "inline functions". Il est plus optimal,
       lors de la compilation du programme (et de son exécution subséquente), d'implémenter des fonctions aussi courtes dans
       le corps de la classe plutôt qu'à l'extérieur de celle-ci. Cependant, toutes les autres fonctions de cette classe sont
       définies dans le fichier C++, car elles présenteront de plus longs codes et il ne sera alors pas optimal de les écrire
       à même la classe "Robot_control". */
    float getLinearVel() const {
      return linear_vel_;
    }

    float getAngularVel() const {
      return angular_vel_;
    }

    void setLinearVel(float linear) {
      linear_vel_ = linear;
    }

    void setAngularVel(float angular) {
      angular_vel_ = angular;
    }

    Robot_control(); // Constructeur par défaut de la classe "Robot_control".

    /* Fonctions de type "callback" permettant de réagir aux messages reçus sur les différents "topics" auxquels aura souscrit le
       programme (ces souscriptions sont présentes dans le "main"). */
    void joyCallback(const sensor_msgs::Joy::ConstPtr&);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
    void publish(float, float); // Fonction permettant de publier les commandes de vélocité du robot.
};

#endif // robot_controller_H
