//#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <utility>
#include <vector>

using namespace std;

struct noeud {
  float count_g, count_h, count_f; // point de depart au noeud consid�r�
                                   // noeud consid�r� au noeud destination
                                   // somme des 2 m�moris�s
  std::pair<int, int> parent;
};

struct point {
  int x, y, occupation;
};

noeud depart;

// D�clarations
int taille[2] = {10, 10};
const int nb_cases = 10;            // Nombre de cases dans chaque dimension
const int taille_case = 50;         // Taille en pixels d'une case
int grid[nb_cases][nb_cases] = {0}; // Initialise toutes les cases � z�ro

list<point> chemin;

typedef map<pair<int, int>, noeud> l_noeud;

point arrivee = {5, 5, 0};
l_noeud liste_fermee;
l_noeud liste_ouverte;

/* calcule la distance entre les points (x1,y1) et (x2,y2) */
float distance(int x1, int y1, int x2, int y2) {
  /* distance euclidienne */
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

  /* carr� de la distance euclidienne */
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
  /* on met tous les noeuds adjacents dans la liste ouverte (+v�rif) */
  for (int i = n.first - 1; i <= n.first + 1; i++) {
    if ((i < 0) || (i >= taille[0])) /* en dehors de l'image, on oublie */
      continue;
    for (int j = n.second - 1; j <= n.second + 1; j++) {
      if ((j < 0) || (j >= taille[1])) /* en dehors de l'image, on oublie */
        continue;
      if ((i == n.first) && (j == n.second)) /* case actuelle n, on oublie */
        continue;

      if (grid[i][j] == 1)
        /* obstacle, terrain non franchissable, on oublie */
        continue;

      std::pair<int, int> it(i, j);
      if (!deja_present_dans_liste(it, liste_fermee)) {
        /* le noeud n'est pas d�j� pr�sent dans la liste ferm�e */

        /* calcul du cout G du noeud en cours d'�tude : cout du parent +
         * distance jusqu'au parent */
        tmp.count_g =
            liste_fermee[n].count_g + distance(i, j, n.first, n.second);

        /* calcul du cout H du noeud � la destination */
        tmp.count_h = distance(i, j, arrivee.x, arrivee.y);
        tmp.count_f = tmp.count_g + tmp.count_h;
        tmp.parent = n;

        if (deja_present_dans_liste(it, liste_ouverte)) {
          /* le noeud est d�j� pr�sent dans la liste ouverte, il faut comparer
           * les couts */
          if (tmp.count_f < liste_ouverte[it].count_f) {
            /* si le nouveau chemin est meilleur, on met � jour */
            liste_ouverte[it] = tmp;
          }

          /* le noeud courant a un moins bon chemin, on ne change rien */

        } else {
          /* le noeud n'est pas pr�sent dans la liste ouverte, on l'y ajoute */
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
    std::cerr << "Erreur, le noeud n'appara�t pas dans la liste ouverte, "
                 "impossible � supprimer"
              << std::endl;
  return;
}

void retrouver_chemin() {
  /* l'arriv�e est le dernier �l�ment de la liste ferm�e */
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

int main() {
  grid[1][1] = 1;
  grid[4][1] = 1;
  grid[3][5] = 1;
  grid[3][6] = 1;
  grid[3][7] = 1;
  // arrivee.x = s->w - 1;
  // arrivee.y = s->h - 1;

  depart.parent.first = 0;
  depart.parent.second = 0;

  std::pair<int, int> courant;

  /* d�roulement de l'algo A* */

  /* initialisation du noeud courant */
  courant.first = 0;
  courant.second = 0;

  /* ajout de courant dans la liste ouverte */
  liste_ouverte[courant] = depart;
  ajouter_liste_fermee(courant);
  ajouter_cases_adjacentes(courant);

  /* tant que la destination n'a pas �t� atteinte et qu'il reste des noeuds �
   * explorer dans la liste ouverte */
  while (!((courant.first == arrivee.x) && (courant.second == arrivee.y)) &&
         (!liste_ouverte.empty())) {

    /* on cherche le meilleur noeud de la liste ouverte, on sait qu'elle n'est
     * pas vide donc il existe */
    courant = meilleur_noeud(liste_ouverte);

    /* on le passe dans la liste ferm�e, il ne peut pas d�j� y �tre */
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

  // Partie Visualisation
  sf::RenderWindow window(sf::VideoMode(500, 500),
                          "Cadrillage de cases avec obstacles");
  // Cr�ation des cases
  sf::RectangleShape cases[nb_cases][nb_cases];

  for (int i = 0; i < nb_cases; i++) {
    for (int j = 0; j < nb_cases; j++) {
      cases[i][j].setPosition(i * taille_case, j * taille_case);
      cases[i][j].setSize(sf::Vector2f(taille_case, taille_case));
      cases[i][j].setOutlineThickness(1);
      cases[i][j].setOutlineColor(sf::Color::Black);

      if (grid[i][j] == 1) {
        cases[i][j].setFillColor(sf::Color::Black);
      } else {
        cases[i][j].setFillColor(sf::Color::White);
      }
    }
  }

  for (const auto &p : chemin) {
    int x = p.x; // Access x field using dot notation
    int y = p.y; // Access y field using dot notation
    cases[x][y].setFillColor(sf::Color::Red);
  }

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    window.clear(sf::Color::Black);

    // Affichage des cases

    for (int i = 0; i < nb_cases; i++) {
      for (int j = 0; j < nb_cases; j++) {
        window.draw(cases[i][j]);
      }
    }

    window.display();
  }

  return 0;
}