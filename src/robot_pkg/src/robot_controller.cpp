/**
 * @file          robot_controller.cpp
 * @author 	      Jacob Deschamps
 * @date          Mars 2022
 * @brief         Fichier permettant de contrôler le robot de la simulation "stage"
 * @version 1.0 : Première version (1.0)
 */

// Librairies ROS et personnelle
#include "robot_controller.h"
#include "geometry_msgs/Twist.h" // Permet d'utiliser des messages ayant pour type "geometry_msgs/Twist".

void moveRobot(Robot_control&);
void changeRobotSpeed(Robot_control&);
void changeBothSpeedCmd(Robot_control&, float&, float&);
float adjustSpeedValue(int, float, float, float);

// Fonction principale du programme.
int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_controller"); /* On crée un noeud appelé "robot_controller" et on renseigne "roscore" quant à son
                                                existance. */
  
  Robot_control robot_teleop; // On déclare un objet de la classe "Robot_control".
  ros::NodeHandle n; // On déclare un noeud ROS (objet de la classe "NodeHandle").

  /* Le noeud ROS précédemment déclaré souscrit aux "topics" "joy" (pour écouter ce que la manette de jeu envoie comme informations)
     et "base_scan" (pour écouter ce que le laser du robot envoie comme informations). Les fonctions de type "callback" qui réagiront
     à toute information perçue sur ces "topics" sont déclarées dans la classe "Robot_control". Ce faisant, ces fonctions de même
     qu'un objet de cette classe (en l'occurrence, l'objet "robot_teleop") doivent être passés par référence dans la fonction
     "subscribe()". */
  ros::Subscriber subJoy = n.subscribe("joy", 10, &Robot_control::joyCallback, &robot_teleop);
  ros::Subscriber subLaser = n.subscribe("base_scan", 10, &Robot_control::scanCallback, &robot_teleop);
  
  ros::Rate loop_rate(20); // Fréquence (en Hz) à laquelle tournera la boucle "while(ros::ok())".

  while (ros::ok()) {
    moveRobot(robot_teleop); /* Les commandes de la manette de jeu sont gérées dans cette fonction et on modifie le contrôle
                                du robot au besoin si les commandes ne concordent pas avec l'état dans lequel le robot se trouve. */

    ros::spinOnce(); /* Permet aux fonctions de type "callback" (fonctions associées aux noeuds qui souscrivent aux "topics") d'être
                        appelées. */

    loop_rate.sleep(); // Permet à la boucle de respecter la fréquence à laquelle elle doit tourner et qui a été établie plus tôt.
  }

  return 0;
}

// Constructeur par défaut de la classe "Robot_control".
Robot_control::Robot_control() {
  linear_vel_ = ROBOT_LINEAR_MAX_SPEED; // La vitesse linéraire du robot est initialisée à sa valeur maximale.
  angular_vel_ = ROBOT_ANGULAR_MAX_SPEED; // La vitesse angulaire du robot est initialisée à sa valeur maximale.
  
  /* Au cas où la manette de jeu ne serait pas encore ouverte au lancement de ce programme, les données normalement reçues de
     cette manette sont toutes initialisées à 0 afin d'être certain que le robot démarre dans un état connu. */
  joy_data_.linear_ = joy_data_.angular_ = 0.0;
  joy_data_.accel_ = joy_data_.spin_ = joy_data_.avoid_ = 0;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10); /* On avertit "roscore" qu'un message de type "geometry_msgs/Twist"
                                                                    sera éventuellement publié sur le "topic" "cmd_vel". */
}

// Fonction permettant de recevoir les commandes envoyées par la manette de jeu ("joystick").
void Robot_control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {  
  joy_data_.linear_ = joy_msg->axes[0];
  joy_data_.angular_ = joy_msg->axes[1];
  joy_data_.accel_ = joy_msg->buttons[0];
  joy_data_.spin_ = joy_msg->buttons[1];
  joy_data_.avoid_ = joy_msg->buttons[2];
}

// Fonction permettant de détecter la distance entre le robot et de quelconques obstacles.
void Robot_control::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) { }

// Fonction permettant d'envoyer les commandes de vélocité au robot.
void Robot_control::publish(float dir_linear, float dir_angular) {
  geometry_msgs::Twist vel_msg;

  vel_msg.linear.x = linear_vel_*dir_linear;
  vel_msg.angular.z = angular_vel_*dir_angular;

  vel_pub_.publish(vel_msg);
}

void moveRobot(Robot_control& robot_teleop) {
  float dir_linear, dir_angular;

  changeRobotSpeed(robot_teleop);
  changeBothSpeedCmd(robot_teleop, dir_linear, dir_angular);
  
  robot_teleop.publish(dir_linear, dir_angular);
  
  robot_teleop.joy_data_.linear_ = robot_teleop.joy_data_.angular_ = 0.0;
  robot_teleop.joy_data_.accel_ = 0;
}

void changeRobotSpeed(Robot_control& robot_teleop) {
  float linear_vel, angular_vel;
  
  linear_vel = robot_teleop.getLinearVel();
  angular_vel = robot_teleop.getAngularVel();
  
  linear_vel = adjustSpeedValue(robot_teleop.joy_data_.accel_, linear_vel, ROBOT_LINEAR_MIN_SPEED, ROBOT_LINEAR_MAX_SPEED);
  angular_vel = adjustSpeedValue(robot_teleop.joy_data_.accel_, angular_vel, ROBOT_ANGULAR_MIN_SPEED, ROBOT_ANGULAR_MAX_SPEED);
  
  robot_teleop.setLinearVel(linear_vel);
  robot_teleop.setAngularVel(angular_vel);
}

void changeBothSpeedCmd(Robot_control& robot_teleop, float& dir_linear, float& dir_angular) {
  dir_linear = robot_teleop.joy_data_.linear_;
  dir_angular = robot_teleop.joy_data_.angular_;
  
  if (robot_teleop.joy_data_.spin_) {
    dir_linear = dir_angular = JOY_MAX_CMD;
  }
}

float adjustSpeedValue(int dir_vel, float current_vel, float min_vel, float max_vel) {
  switch (dir_vel) {
    case JOY_MIN_CMD:
      current_vel -= ROBOT_SPEED_STEP;

      current_vel = (current_vel < min_vel) ? min_vel : current_vel;
      break;
    case JOY_MAX_CMD:
      current_vel += ROBOT_SPEED_STEP;

      current_vel = (current_vel > max_vel) ? max_vel : current_vel;
      break;
  }

  return current_vel;
}