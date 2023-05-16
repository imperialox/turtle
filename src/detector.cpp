#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <string>
#include <tb3_projet_LAKHAL_MEZOUAR/LaserDirection.h>
#include <vector>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace tb3_projet_LAKHAL_MEZOUAR;

// VARIABLES GLOBALES
int NBR_REGION = 8;
int SCAN_SAMPLE = 360;
int ANGLE_REGION = 45;
std::vector<float> min_distances;
#define FREQ_CONTROL 5.0
int Indice[8];
Publisher obstacle_pub;
LaserDirection var;

// PROTOTYPE DES FONCTIONS
float get_minimum(vector<float> data, int debut, int fin, int *indiceMin);

// FONCTION CALLBACK
// A COMPLETER//
void scanCallback(LaserScan lidar) {
  min_distances.clear();
  int i;

  for (i = 0; i < NBR_REGION; i++) {
    float distance = get_minimum(lidar.ranges, i * ANGLE_REGION,
                                 (i + 1) * (ANGLE_REGION - 1), &Indice[i]);

    min_distances.push_back(distance);
  }
  var.GAUCHE_HAUT = min_distances[0];

  var.FACE_GAUCHE = min_distances[1];

  var.ARRIERE_GAUCHE = min_distances[2];

  var.GAUCHE_BAS = min_distances[3];

  var.DROITE_BAS = min_distances[4];

  var.ARRIERE_DROITE = min_distances[5];

  var.FACE_DROITE = min_distances[6];

  var.DROITE_HAUT = min_distances[7];

  obstacle_pub.publish(var);
}

int main(int argc, char **argv) {
  init(argc, argv, "detector");
  NodeHandle nh;

  Rate loop_rate(FREQ_CONTROL);

  // A COMPLETER
  Subscriber obstacle_sub = nh.subscribe("/scan", 100, scanCallback);
  obstacle_pub =
      nh.advertise<LaserDirection>("/obstacle", 10);
      
  // topic

  spin();

  return 0;
}

float get_minimum(vector<float> data, int debut, int fin, int *indice_min) {

  float min = FLT_MAX;

  for (int i = debut; i <= fin; i++) {
    if (data[i] < min && data[i] > 0.0) {
      min = data[i];
      //*indice_min = i;
    }
  }
  return std::min(10.0f, min);
}