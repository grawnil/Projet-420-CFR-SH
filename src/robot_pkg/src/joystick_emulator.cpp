/**
 * @file   		    joystick_emulator.cpp
 * @author 	      Willow Garage, Inc. (révision Jacob Deschamps)
 * @date          2011 (révision mars 2022)
 * @brief         Fichier permettant d'émuler une manette de jeu avec ROS
 * @version 1.1 : Première version (1.0) révisée (1.1)
 *
 * Environnement :
 * 	Développement : Éditeur de code/texte sous Linux
 * 	Compilateur : gcc (ou g++) et catkin_make
 *  Simulateur : ROS (stage)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Librairies ROS et personnelle
#include "sensor_msgs/Joy.h" // Permet d'utiliser des messages ayant pour type "sensor_msgs/Joy".
#include "joystick_emulator.h" // Contient notamment les macros et la classe "Joy_emulator" utilisés dans ce code C++

// Librairies Linux et C/C++
#include <signal.h> // Pour gérer les différents signaux pouvant survenir pendant l'exécution du programme (ex : CTRL-C).
#include <stdio.h> // Pour afficher des messages en console notamment via les fonctions printf() et puts().
#include "boost/thread/thread.hpp" /* Pour pouvoir utiliser des threads et des mutex dans ce programme. On peut donc rouler des
                                      processus en parallèle (ex : détecter en console la touche que l'utilisateur aura enfoncée
                                      et vérifier en parallèle si une touche est maintenue enfoncé si si, encore, aucune touche
                                      n'a de nouveau été enfoncé dans un certain intervalle de temps).*/

using namespace boost; /* On utilise l'espace de nommage "boost" ce qui nous permet d'éviter d'écrire la mention "boost::" devant
                          chaque fonction provenant de la libraire définissant les threads et les mutex. On pourrait le faire pour
                          "ros" à la place, mais ce serait peut-être déconcertant de ne pas pouvoir remarquer d'un simple coup
                          d'oeil si les fonctions tels que init(), spin(), ok(), etc. viennent de l'environnement ROS. */

// Fonction principale du programme.
int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_emulator"); /* On crée un noeud appelé "joystick_emulator" et on renseigne "roscore" quant à
                                                 son existance. */
  
  // Les variables utilisées dans le main() ne peuvent pas être déclarées avant l'appel de la fonction init() de ROS.
  Joy_emulator joystick_teleop; // On déclare un objet de la classe "Joy_emulator".
  ros::NodeHandle n; // On déclare un noeud ROS (objet de la classe "NodeHandle").

  signal(SIGINT, quit); // Permet d'appeler la fonction quit() si  un signal d'interruption est envoyé (ex : CTRL-C).

  /* Appel du constructeur de la classe "thread" qui permettra de créer un thread pour la fonction keyLoop(). Cette fonction
     pourra donc rouler en parallèle d'autres processus puisqu'elle s'occuppe de détecter les touches que l'utilisateur tappe
     en console pendant l'exécution du programme, */
  thread keyLoop_thread(bind(&Joy_emulator::keyLoop, &joystick_teleop));
  ros::Timer watchdog_timer = n.createTimer(ros::Duration(0.1), bind(&Joy_emulator::watchdog, &joystick_teleop));

  ros::spin(); /* Permet aux fonctions de type "callback" (fonctions associées aux noeuds qui souscrivent aux "topics") d'être
                  appelées. Ce n'est pas pertinent, ici, puisqu'il n'y a pas de fonctions de type "callback" dans ce fichier C++,
                  mais on peut tout de même laisser la fonction "spin()" au cas où, un jour, une fonction de type "callback"
                  serait ajoutée. */

  keyLoop_thread.interrupt(); /* Permet d'interrompre le thread appliqué sur la fonction keyLoop() afin qu'il puisse redémarré
                                 par apprès lors de l'appel de la fonction join() ci-dessous. */
  keyLoop_thread.join(); /* L'appel à la fonction join() permet au thread appliqué sur la fonction keyLoop() de s'exécuter. On ne
                            quitte pas la fonction join() tant que le thread ne s'est pas exécuté complètement. La fonction join()
                            retourne toutefois immédiatement si le thread n'a pas été préalablement démarré (ce qui n'est pas le
                            cas ici). */
  return 0;
}

// Constructeur par défaut de la classe "Joy_emulator".
Joy_emulator::Joy_emulator() {
	linear_ = angular_ = 0.0;
  enable_ = spin_ = accel_ = avoid_ = 0;
  joy_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1); /* On avertit "roscore" qu'un message de type "sensor_msgs/Joy" sera
                                                           éventuellement publié sur le "topic" "joy". */
}

/* Fonction qui vérifie si une touche est maintenue enfoncée ou s'il n'y en a aucune d'enfoncée. Si aucune touche n'est enfoncée
   ou maintenue enfoncée dans un certain intervalle de temps, des zéros sont publiés. */
void Joy_emulator::watchdog() {
  mutex::scoped_lock lock(publish_mutex_); /* Puisqu'on s'apprête possiblemment à publier des zéros, il faut s'assurer que le thread
                                              appliqué sur la fonction keyLoop() n'est pas lui-même déjà en train d'utiliser les
                                              ressources de publication (on s'assure de sécuritairement partager les ressources entre
                                              les threads). */

  /* Si aucune touche n'a été enfoncée dernièrement ou n'est maintenue enfoncée, on publie des zéros pour indiquer à tous les noeuds
     qui écouteraient les messages publiés qu'aucune commande précédemment entamée ne doit être poursuivie. */
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0.0, 0.0, 0);
}

// Fonction qui détecte les touches enfoncées sur le clavier. Cette fonction roule en parallèle d'autres fonctions grâce à un thread.
void Joy_emulator::keyLoop() {
  char key; // Touche enfoncée sur le clavier.

  /* On configure la console via laquelle a été lancé le programme pour qu'elle soit en mode "raw" (ex : pas d'encodage de caractères
     particulier). */
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  // On ajoute une nouvelle ligne à la console après le lancement du programme, puis on termine avec EOF (pour "End Of File").
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  // On affiche ces instructions en console.
  puts("Lecture des touches enfoncees sur le clavier.");
  puts("---------------------------------------------");
  puts("Utilisez les fleches pour controler le robot.");

  /* Tant que l'utilistaeur n'appuie pas sur les touches [CTRL] et [C], on reste coincé dans cette boucle (à l'exception de
     l'exécution des threads (ex : la fonction watchdog())). */
  while (ros::ok()) {
    // On détecte la touche qui vient d'être enfoncée et on vérifie qu'il n'y a pas d'erreur de lecture (autrement, on quitte).
    if(read(kfd, &key, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    linear_ = angular_ = 0.0;
    accel_ = 0;

    ROS_DEBUG("value: 0x%02X\n", key); /* On affiche sous forme de "debug" (donc, ça n'apparaît pas officiellement en console)
                                          le code hexadécimal de chaque touche enfoncée au clavier. */

    // On détecte quelle touche a été enfoncée et on entreprend l'action appropriée en fonction de cette touche.
    switch(key) {
      case LETTER_E:
        ROS_DEBUG("E");
        enable_ ^= 1; // On inverse la valeur de la variable "enable_" à chaque fois qu'on appuie sur a touche [E].
        break;
      case LETTER_S:
        ROS_DEBUG("S");
        spin_ ^= 1; // On inverse la valeur de la variable "spin_" à chaque fois qu'on appuie sur a touche [S].
        break;
      case LETTER_X:
        ROS_DEBUG("X");
        avoid_ ^= 1; // On inverse la valeur de la variable "avoid_" à chaque fois qu'on appuie sur a touche [X].
        break;
      
      /* Les prochaines valeurs de touches sont multipliées par la variable "enable_" afin d'annuler leur effet si
         la mentte  n'est pass activée (si la variable "enable_" vaut 0). */
      case ARROW_UP:
        ROS_DEBUG("UP");
        linear_ = 1.0*enable_;
        break;
      case ARROW_DOWN:
        ROS_DEBUG("DOWN");
        linear_ = -1.0*enable_;
        break;
      case ARROW_LEFT:
        ROS_DEBUG("LEFT");
        angular_ = 1.0*enable_;
        break;
      case ARROW_RIGHT:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0*enable_;
        break;
      case SIGN_PLUS:
        ROS_DEBUG("+");
        accel_ = 1*enable_;
        break;
      case SIGN_MINUS:
        ROS_DEBUG("-");
        accel_ = -1*enable_;
        break;
    }

    mutex::scoped_lock lock(publish_mutex_); /* Puisqu'on s'apprête à modifier les valeurs des variables "last_publish_" et
                                                "first_publish_" et qu'elles sont utilisées par la fonction watchdog() qui roule en
                                                parallèle de celle-ci, il faut s'assurer que les threads ne tomberont pas en conflit
                                                en essayant d'utiliser les mêmes ressources. C'est pourquoi ce mutex est important
                                                ici. */

    /* On modifie les valeurs des variables "last_publish_" et "first_publish_", car elles sont utilisées dans la fonction watchdog()
       pour vérifier si aucune touche n'a été enfoncée depuis un certain intervalle de temps. */
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
      first_publish_ = ros::Time::now();

    last_publish_ = ros::Time::now();
    
    /* On multiplie les variables "spin_" et "avoid_" par la variable "enable_" pour prendre en compte que le fait que la manette de
       jeu puisse être désactivée. Dans un tel cas, les variables "spin_" et "avoid_"doivent valoir 0. */
    spin_ *= enable_;
    avoid_ *= enable_;
    
    publish(linear_, angular_, accel_); /* On publie les nouvelles informations qui ont été modifiées en fonction de la ou des
                                           touche(s) enfoncée(s). */
  }
}

// Fonction qui publie sur le "topic" "joy" les informations modifiées par les touches du clavier.
void Joy_emulator::publish(float linear, float angular, int accel) {
	sensor_msgs::Joy joy_msg;
	
  /* On redimensionne les vecteurs "axes" et "buttons". Chaque case du vecteur "buttons" doit contenir la valeur d'un des boutons
     de la manette de jeu et chaque case du vecteur "axes" doit contenir la valeur de déplacement (linéaire ou angulaire). */
	joy_msg.axes.resize(2);
  joy_msg.buttons.resize(3);
	
  // On assigne les valeurs des axes et des boutons aux bonnes cases des vecteurs "axes" et "buttons".
	joy_msg.axes[0] = linear;
	joy_msg.axes[1] = angular;
  joy_msg.buttons[0] = accel;
  joy_msg.buttons[1] = spin_;
  joy_msg.buttons[2] = avoid_;
	
	joy_pub_.publish(joy_msg); // On publie les nouvelles valeurs sur le "topic" "joy".
}

/* Fonction qui est appelée si l'utilisateur appuie sur les touches [CTRL] et [C] par exemple. Elle permet d'interrompre tous
   processus et de libérer la console pour d'autres commandes puiss y être tapées. */
void quit(int sig) {
  tcsetattr(kfd, TCSANOW, &cooked); // Configuration de la console pour prochainement quitter.

  ros::shutdown(); // On ferme tous le noeud participant à l'exécution de ce programme.

  exit(0); // On quitte le programme sans erreur.
}