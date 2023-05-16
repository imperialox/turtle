// LES FICHIERS EN-TETES
#include "pid.cpp"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <cmath>
#include <csignal>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tb3_projet_LAKHAL_MEZOUAR/LaserDirection.h>

#include <tf/tf.h>

#include <iostream>
#include <vector>

// LES NAMESPACES
using namespace ros;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace tb3_projet_LAKHAL_MEZOUAR;

// LES CONSTANTES
#define MODE_CIRCULAIRE 1
#define MODE_RECTANGULAIRE 2
#define LINEAR_SPEED 0.8
#define ANGULAR_SPEED 0.8
#define FREQ_CONTROL 5.0 // hz
#define PERIODE_DE_CONTROLE 1 / FREQ_CONTROL
#define MAX_LINEAR_SPEED 0.15
#define MAX_ANGULAR_SPEED 2.84

#define CMD_LMAX MAX_LINEAR_SPEED
#define CMD_LMIN -MAX_LINEAR_SPEED

#define CMD_AMAX MAX_ANGULAR_SPEED
#define CMD_AMIN -MAX_ANGULAR_SPEED

#define Kp_pos 1.0
#define Kd_pos 0.0
#define Ki_pos 0.0

#define Kp_angle 1.0
#define Kd_angle 0.0
#define Ki_angle 0.0

// LES VARIABLES GLOBALES

String message;
float x_objectif = 1.0;
float y_objectif = 1.0;
int k = 0;
int flag_obstacle = 0;
float distance_objectif;
LaserDirection var;
int mode = 1;
Twist msg;
Pose2D positions;

Int8 key_B;
double frequence = FREQ_CONTROL;
int obstacle_evite = 0;

LaserScan data;
String test;
int indice_case = 3;



std_msgs::Int32MultiArray map_py;
Publisher map_py_pub;
int map2[34][31], map_Score[41][41], init_map = 0, y_n = 0, x_n = 0, init90 = 0;

int tableau_x[100];
int tableau_y[100];

int temps_attente_case = 0;
int start_x = 20;
int start_y = 20;



// definitions de structures

struct noeud {
  float count_g, count_h, count_f; // point de depart au noeud consid?r?
                                   // noeud consid?r? au noeud destination
                                   // somme des 2 m?moris?s
  pair<int, int> parent;
};

struct point {
  int x, y, occupation;
};

noeud depart;

// D?clarations
int taille[2] = {34, 31};
const int taille_x = 34; // Nombre de cases dans chaque dimension
const int taille_y = 31;
list<point> chemin;

typedef map<pair<int, int>, noeud> l_noeud;

point arrivee = {25, 5, 0};
l_noeud liste_fermee;
l_noeud liste_ouverte;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Les publishers
// Publisher log_pub;
Publisher vel_pub;
Publisher position2D_pub;
Publisher python_pub;

// Les PID
PID *pidDistance =
    new PID(PERIODE_DE_CONTROLE, CMD_LMAX, CMD_LMIN, Kp_pos, Kd_pos, Ki_pos);

PID *pidAngle = new PID(PERIODE_DE_CONTROLE, CMD_AMAX, CMD_AMIN, Kp_angle,
                        Kd_angle, Ki_angle);

// LES PROTOTYPES DE FONCTIONS
void sigintHandler(int sig);

// LES FONCTIONS ET LES CALLBACKS

// void log_message(char *texte) {
// message.data = texte;
// ROS_INFO("%s", message.data.c_str());
// log_pub.publish(message);
//}

void avancer(double lin_velx) {
  msg.linear.x = lin_velx;
  msg.linear.y = 0;
  msg.angular.z = 0;
  vel_pub.publish(msg);
}

void stop() {
  msg.linear.x = 0;
  msg.angular.z = 0;
  vel_pub.publish(msg);
}

void reculer(float lin_vel) {
  msg.linear.x = -lin_vel;
  msg.angular.z = 0;
  vel_pub.publish(msg);
}

// void move(float lin_vel) {
// avancer(lin_vel);
// loop_rate.sleep();
// stop();
// loop_rate.sleep();
// reculer(lin_vel);
// loop_rate.sleep();
//}

void set_plus() {
  if (msg.linear.x < 2) {
    msg.linear.x += 0.05;
    vel_pub.publish(msg);
    key_B.data = 118;
  }
}

void set_moins() {
  if (msg.linear.x >= -2) {
    msg.linear.x -= 0.05;
    vel_pub.publish(msg);
    key_B.data = 118; // Ici on remet la valeur du bouton à une valeur
                      // inutilisée pour que ça n'accélère qu'une fois.
  }
}

void pivoter_droite(float ang_vel) {
  msg.angular.z = ang_vel;
  vel_pub.publish(msg);
  key_B.data =
      118; // Même chose ici, on augmente la vitesse de rotation qu'une fois.
}

void pivoter_gauche(float ang_vel) {
  msg.angular.z = -ang_vel;
  vel_pub.publish(msg);
  key_B.data = 118; // Réinitialisation à une touche du clavier inutilisée ici.
}

void pivoter(float ang_vel) {
  msg.angular.z = ang_vel;
  vel_pub.publish(msg);
  key_B.data = 118; // Réinitialisation à une touche du clavier inutilisée ici
}

float cal_delta_Theta(float _theta, float _theta0) {

  float theta, theta0, delta_theta;

  if (_theta < 0)
    theta = _theta + 2 * M_PI;
  else
    theta = _theta;

  if (_theta0 < 0)
    theta0 = _theta0 + 2 * M_PI;
  else
    theta0 = _theta0;

  delta_theta = theta - theta0;

  if (delta_theta < -M_PI) {
    delta_theta = delta_theta + 2 * M_PI;
  } else {
    if (delta_theta >= M_PI) {
      delta_theta = delta_theta - 2 * M_PI;
    }
  }
  return delta_theta;
}

void cmd_angle(double theta, double theta0) {
  float delta_angle =
      cal_delta_Theta(theta, theta0); // calcul de l'écart entre l'angle courant
                                      // et celui de destination.
  double cmd_angulaire = pidAngle->calculate(delta_angle);
  pivoter(cmd_angulaire);
}

void cmd_lin(double x1, double x0) {
  double cmd_lineaire = pidDistance->calculate(x1, x0);
  avancer(fabs(cmd_lineaire));
}

float quaternionToAngleEuler(Quaternion angle_quaternion) {
  float angle_Euler = tf::getYaw(angle_quaternion);
  return angle_Euler;
}

void drive(float lin_vel, float ang_vel) {}

void chatterCallback1(Int8 key_board) { key_B.data = key_board.data; }

void chatterCallback2(Odometry position) {
  positions.x = position.pose.pose.position.x;
  positions.y = position.pose.pose.position.y;
  positions.theta = quaternionToAngleEuler(position.pose.pose.orientation);
  position2D_pub.publish(positions);
}

void chatterCallback3(LaserDirection data) {
  var.FACE_GAUCHE = data.FACE_GAUCHE;
  var.FACE_DROITE = data.FACE_DROITE;
  var.GAUCHE_HAUT = data.GAUCHE_HAUT;
  var.GAUCHE_BAS = data.GAUCHE_BAS;
  var.ARRIERE_DROITE = data.ARRIERE_DROITE;
  var.ARRIERE_GAUCHE = data.ARRIERE_GAUCHE;
  var.DROITE_HAUT = data.DROITE_HAUT;
  var.DROITE_BAS = data.DROITE_BAS;
}

void chatterCallback4(const std_msgs::Int32::ConstPtr &msg) {
  // ROS_INFO("Received: %d", msg->data);
  panneau_detecte = msg->data;
  key_B.data = 118;
}

void chatterCallback5(const std_msgs::Int32::ConstPtr &msg) {
  // ROS_INFO("Received: %d", msg->data);
  panneau_detecte2 = msg->data;
  key_B.data = 118;
}

void chatterCallback6(const std_msgs::Int32::ConstPtr &msg) {
  // ROS_INFO("Received: %d", msg->data);
  panneau_detecte3 = msg->data;
  key_B.data = 118;
}

void objectif_position() {
  if (mode == 1) { // On vérifie que l'on se trouve bien dans le monde autonome
                   // du TurtleBot.
    k++;
    double angle =
        atan2((y_objectif - positions.y), (x_objectif - positions.x)); //
    ROS_INFO("angle = %.2f", angle);
    distance_objectif = sqrt(pow(x_objectif - positions.x, 2) +
                             pow(y_objectif - positions.y, 2));
    if (distance_objectif < 0.02) { // tolérance à l'objectif
      stop();
      ROS_INFO("JE SUIS LA");
    } else {
      if (flag_obstacle == 0 && var.FACE_DROITE != 0.0 &&
          var.FACE_DROITE > 0.35) {
        cmd_angle(angle, positions.theta);
        ROS_INFO("commande angulaire");

        if (fabs(positions.theta - angle) < 0.02) {
          stop();

          cmd_lin(x_objectif, positions.x);
          ROS_INFO("ANGLE OK");
        }
      }
    }

    if (((var.GAUCHE_HAUT <= 0.45 && var.GAUCHE_HAUT != 0) ||
         (var.DROITE_HAUT <= 0.45 && var.DROITE_HAUT != 0)) &&
        obstacle_evite == 0) {
      flag_obstacle = 1;
      if (var.FACE_DROITE <= var.FACE_GAUCHE &&
          var.FACE_DROITE != 0) { // On exclue le cas égale à 0 car au lancement
        // du robot ces valeurs sont à 0 et on rentre dans la
        // condition alors que nous ne devrions pas
        pivoter(1);
        ROS_INFO("PIVOTEMENT FACE_DROITE");
      }
      if (var.FACE_DROITE > var.FACE_GAUCHE &&
          var.FACE_GAUCHE != 0) { // On exclue le cas égale à 0 car au lancement
        // du robot ces valeurs sont à 0 et on rentre dans la
        // condition alors que nous ne devrions pas
        pivoter(-1);
        ROS_INFO("PIVOTEMENT FACE_GAUCHE");
      }

    } else {
      flag_obstacle = 0;
    }
    if (distance_objectif < 0.1) {
      obstacle_evite = 1;
    }
  }
}



/////////////////////Optimisation///////////////////////////////////////////////////////////////////////////////


/* calcule la distance entre les points (x1,y1) et (x2,y2) */
float distance(int x1, int y1, int x2, int y2) {
  /* distance euclidienne */
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

  /* carr? de la distance euclidienne */
  /* return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2); */
}

bool deja_present_dans_liste(pair<int, int> n, l_noeud &l) {
  l_noeud::iterator i = l.find(n);
  if (i == l.end())
    return false;
  else
    return true;
}

void ajouter_cases_adjacentes(std::pair<int, int> &n) {
  noeud tmp;
  /* on met tous les noeuds adjacents dans la liste ouverte (+v?rif) */
  for (int i = n.first - 1; i <= n.first + 1; i++) {
    if ((i < 0) || (i >= taille[0])) /* en dehors de l'image, on oublie */
      continue;
    for (int j = n.second - 1; j <= n.second + 1; j++) {
      if ((j < 0) || (j >= taille[1])) /* en dehors de l'image, on oublie */
        continue;
      if ((i == n.first) && (j == n.second)) /* case actuelle n, on oublie */
        continue;

      if (map2[i][j] == 1)
        /* obstacle, terrain non franchissable, on oublie */
        continue;

      std::pair<int, int> it(i, j);
      if (!deja_present_dans_liste(it, liste_fermee)) {
        /* le noeud n'est pas d?j? pr?sent dans la liste ferm?e */

        /* calcul du cout G du noeud en cours d'?tude : cout du parent +
         * distance jusqu'au parent */
        tmp.count_g =
            liste_fermee[n].count_g + distance(i, j, n.first, n.second);

        /* calcul du cout H du noeud ? la destination */
        tmp.count_h = distance(i, j, arrivee.x, arrivee.y);
        tmp.count_f = tmp.count_g + tmp.count_h;
        tmp.parent = n;

        if (deja_present_dans_liste(it, liste_ouverte)) {
          /* le noeud est d?j? pr?sent dans la liste ouverte, il faut comparer
           * les couts */
          if (tmp.count_f < liste_ouverte[it].count_f) {
            /* si le nouveau chemin est meilleur, on met ? jour */
            liste_ouverte[it] = tmp;
          }

          /* le noeud courant a un moins bon chemin, on ne change rien */

        } else {
          /* le noeud n'est pas pr?sent dans la liste ouverte, on l'y ajoute */
          liste_ouverte[std::pair<int, int>(i, j)] = tmp;
        }
      }
    }
  }
}

std::pair<int, int> meilleur_noeud(l_noeud &l) {
  float m_coutf = l.begin()->second.count_f;
  std::pair<int, int> m_noeud = l.begin()->first;

  for (l_noeud::iterator i = l.begin(); i != l.end(); i++)
    if (i->second.count_f < m_coutf) {
      m_coutf = i->second.count_f;
      m_noeud = i->first;
    }

  return m_noeud;
}

void ajouter_liste_fermee(std::pair<int, int> &p) {
  noeud n = liste_ouverte[p];
  liste_fermee[p] = n;

  /* il faut le supprimer de la liste ouverte, ce n'est plus une solution
   * explorable */
  if (liste_ouverte.erase(p) == 0)
    std::cerr << "Erreur, le noeud n'appara?t pas dans la liste ouverte, "
                 "impossible ? supprimer"
              << std::endl;
  return;
}

void retrouver_chemin() {
  /* l'arrivee est le dernier ?l?ment de la liste ferm?e */
  noeud &tmp = liste_fermee[std::pair<int, int>(arrivee.x, arrivee.y)];

  point n;
  std::pair<int, int> prec;
  n.x = arrivee.x;
  n.y = arrivee.y;
  prec.first = tmp.parent.first;
  prec.second = tmp.parent.second;
  chemin.push_front(n);

  while (prec !=
         std::pair<int, int>(depart.parent.first, depart.parent.first)) {
    n.x = prec.first;
    n.y = prec.second;
    chemin.push_front(n);

    tmp = liste_fermee[tmp.parent];
    prec.first = tmp.parent.first;
    prec.second = tmp.parent.second;
  }
}

void construire_tableau(list<point> road) {
  list<point>::iterator it; // déclaration de l'itérateur
  int  indice_tableau= 0;
  for (it = road.begin(); it != road.end(); ++it) {
    tableau_x[indice_tableau] = it->x;
    tableau_y[indice_tableau] = it->y; 
    indice_tableau += 1;
  }
}

void turtle_Astar() {

  float x = (tableau_x[indice_case] - start_x) / 20.0;
  float y = (tableau_y[indice_case] - start_y) / 20.0;

  double theta = atan2((y - positions.y), (x - positions.x)); 
  float dist = sqrt(pow(x - positions.x, 2) + pow(y - positions.y, 2));
  temps_attente_case += 1;
  if (temps_attente_case > 10 && indice_case < chemin.size() - 1) {
    indice_case += 1;
    temps_attente_case = 0;
  }
  if (dist < 0.02) {
    stop();

    if (indice_case < chemin.size() - 1) {
      indice_case += 1;
    }
  } else {
    cmd_angle(theta, positions.theta);
    if (fabs(positions.theta - theta) < 0.02) {
      stop();
      cmd_lin(x, positions.x);
    }
  }
}

// MAIN DE L'APPLICATION

int main(int argc, char **argv) {
  signal(SIGINT, sigintHandler);
  init(argc, argv, "controleur", ros::init_options::NoSigintHandler);
  NodeHandle nh;

  vel_pub = nh.advertise<Twist>("/cmd_vel", 10);
  position2D_pub = nh.advertise<Pose2D>("/topic_position", 10);
  // log_pub = nh.advertise<String>("/log", 10);
  python_pub = nh.advertise<String>("/stringpy", 10);

  Subscriber key_board_sub =
      nh.subscribe("/keyboard_input", 1000, chatterCallback1);
  Subscriber position_sub = nh.subscribe("/odom", 1000, chatterCallback2);
  Subscriber laser_sub = nh.subscribe("/obstacle", 1000, chatterCallback3);
  // Subscriber test_python = nh.subscribe("/camera_turtle", 1000,
  // chatterCallback4);
  Subscriber mapping = nh.subscribe("/scan", 1000, mappingcallback);
  map_py_pub = nh.advertise<std_msgs::Int32MultiArray>("map_matrix", 1000);

  Subscriber test_python =
      nh.subscribe("/camera_turtle", 1000, chatterCallback4);
  Subscriber panneau2 =
      nh.subscribe("/camera_turtle_droite", 1000, chatterCallback5);
  Subscriber panneau3 =
      nh.subscribe("/camera_turtle_feu", 1000, chatterCallback6);

  
  // Saisie de sparamètres

  Rate loop_rate(FREQ_CONTROL);
  stop();

  ros::Duration(2).sleep();

  while (ok()) {
    switch (key_B.data) {

    case (122):
      ROS_INFO("AVANCE");
      avancer(0.05);
      break;

    case (115):
      reculer(0.05);
      break;

    case (97):
      stop();
      break;

    case (112):
      set_plus();

      break;
    case (109):
      set_moins();
      break;

    case (100):
      pivoter_droite(0.05);
      break;

    case (113):
      pivoter_gauche(0.5);
      break;
    }

  
    //  ///////////  Chemin Optimal///////////////////
    depart.parent.first = 20;
    depart.parent.second = 20;
    std::pair<int, int> courant;
    /* d?roulement de l'algo A* */
    /* initialisation du noeud courant */
    courant.first = 20;
    courant.second = 20;
    /* ajout de courant dans la liste ouverte */
    liste_ouverte[courant] = depart;
    ajouter_liste_fermee(courant);
    ajouter_cases_adjacentes(courant);
    /* tant que la destination n'a pas ?t? atteinte et qu'il reste des
   noeuds
    ? explorer dans la liste ouverte */
    while (!((courant.first == arrivee.x) && (courant.second == arrivee.y)) &&
           (!liste_ouverte.empty())) {

      /* on cherche le meilleur noeud de la liste ouverte, on sait
      qu'elle n'est pas vide donc il existe */
      courant = meilleur_noeud(liste_ouverte);

      /* on le passe dans la liste ferm?e, il ne peut pas d?j? y ?tre
       */
      ajouter_liste_fermee(courant);

      /* on recommence la recherche des noeuds adjacents */
      ajouter_cases_adjacentes(courant);
    }

    /* si la destination est atteinte, on remonte le chemin */
    if ((courant.first == arrivee.x) && (courant.second == arrivee.y)) {
      retrouver_chemin();

    } else {
      /* pas de solution */
    }

    construire_tableau(chemin);
  ;
  
    turtle_Astar();


    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void sigintHandler(int sig) {
  // Log quit
  ROS_INFO("Exiting program gracefully ...");
  stop();
  // Kill all open subscriptions, publications, service calls, and service
  // servers
  shutdown();
}
