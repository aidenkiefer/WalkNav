// application.cpp 
// Aiden Kiefer
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <algorithm>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

//
// FindNearestNode
//given a building, iterates through each node, finding the node
//with the minimum distance from the building to the node
//returns the long long identifying the nearest node

long long FindNearestNode(const BuildingInfo& building, const map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways) {
    long long nearest = -1;
    double minDist = 100000.00;

    for (const auto& footway : Footways) {
      for (const auto& node : footway.Nodes) {
        double dist = distBetween2Points(building.Coords.Lat, building.Coords.Lon, Nodes.at(node).Lat, Nodes.at(node).Lon);
        if (dist < minDist) {
            minDist = dist;
            nearest = node;
        }
      }
    }

    return nearest;
}

void Dijkstra(const graph<long long, double>& G, long long start, map<long long, double>& distance, map<long long, long long>& previous) {
  priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<pair<double, long long>>> pq;

  for (const auto& vertex : G.getVertices()) {
    distance[vertex] = 100000.00;  //set initial distances to large number
    previous[vertex] = -1;   //set initial previous node to -1
  }

  distance[start] = 0;  //set distance to the start node to 0
  pq.push({0, start});

  while (!pq.empty()) {
    auto [dist, current] = pq.top();
    pq.pop();

    for (const auto& neighbor : G.neighbors(current)) {
      double tempWeight = 0;
      if (G.getWeight(current, neighbor, tempWeight)) {
        double newDist = distance[current] + tempWeight;
        if (newDist < distance[neighbor]) {
          distance[neighbor] = newDist;
          previous[neighbor] = current;
          pq.push({newDist, neighbor});
        }
      }
    }
  }
}

vector<long long> retrievePath(long long dest, const map<long long, long long>& previous) {
  vector<long long> path;
  for (long long at = dest; at != -1; at = previous.at(at)) {
    path.push_back(at);
  }
  reverse(path.begin(), path.end());
  return path;
}

void outputPathsAndDistances(const vector<long long>& path1, double distance1, const vector<long long>& path2, double distance2) {
  int p1size = path1.size();
  int p2size = path2.size();
  
  cout << "Person 1's distance to dest: " << distance1 << " miles" << endl;
  cout << "Path: ";
  for (int i = 0; i < p1size; ++i) {
    cout << path1[i];
    if (i != p1size - 1) {
      cout << "->";
    }
  }

  cout << endl << endl;

  cout << "Person 2's distance to dest: " << distance2 << " miles" << endl;
  cout << "Path: ";
  for (int i = 0; i < p2size; ++i) {
    cout << path2[i];
    if (i != p2size - 1) {
      cout << "->";
    }
  }

  cout << endl << endl;
  
}

void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  
  
  //strings for user input
  string person1Building, person2Building;
  double min = 100000.00;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    //bools to check if user input is valid
    bool person1buildingfound  = false;
    bool person2buildingfound = false;

    //if input is valid, building info stored in each
    BuildingInfo start1;
    BuildingInfo start2;

    //initialize two long long identifiers
    long long sNode1 = -1;
    long long sNode2 = -1;

    for (const auto& building : Buildings) {
      if (building.Abbrev == person1Building) {
        person1buildingfound = true;
        start1 = building;
      }
      else if (building.Fullname.find(person1Building) != string::npos) {
        person1buildingfound = true;
        start1 = building;
      }
      if (building.Abbrev == person2Building) {
        person2buildingfound = true;
        start2 = building;
      }
      else if (building.Fullname.find(person2Building) != string::npos) {
        person2buildingfound = true;
        start2 = building;
      }
    }

    if (!person1buildingfound) {
      cout << "Person 1's building not found" << endl;
    }
    else if (!person2buildingfound) {
        cout << "Person 2's building not found" << endl;
    }
    else {

      //calculates midpoint between two starting buildings
      Coordinates midpoint = centerBetween2Points(start1.Coords.Lat, start1.Coords.Lon, start2.Coords.Lat, start2.Coords.Lon);

      //buildinginfo for center building (destination)
      BuildingInfo centerBuilding;

      for (const auto& building : Buildings) {
        double tempDist = distBetween2Points(midpoint.Lat, midpoint.Lon, building.Coords.Lat, building.Coords.Lon);
        if (tempDist < min) {
          min = tempDist;
          centerBuilding = building;
        }
      }

      sNode1 = FindNearestNode(start1, Nodes, Footways);
      sNode2 = FindNearestNode(start2, Nodes, Footways);

      long long destNode = FindNearestNode(centerBuilding, Nodes, Footways);
      cout << endl;
    
      cout << "Person 1's point:" << endl;
      cout << " " << start1.Fullname << endl;
      cout << " (" << start1.Coords.Lat << ", " << start1.Coords.Lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << start2.Fullname << endl;
      cout << " (" << start2.Coords.Lat << ", " << start2.Coords.Lon << ")" << endl;
      cout << "Destination Building:" << endl;
      cout << " " << centerBuilding.Fullname << endl;
      cout << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")" << endl << endl;

      cout << "Nearest P1 node:" << endl;
      cout << " " << sNode1 << endl;
      cout << " (" << Nodes[sNode1].Lat << ", " << Nodes[sNode1].Lon << ")" << endl;
      cout << "Nearest P2 node:" << endl;
      cout << " " << sNode2 << endl;
      cout << " (" << Nodes[sNode2].Lat << ", " << Nodes[sNode2].Lon << ")" << endl;
      cout << "Nearest destination node:" << endl;
      cout << " " << destNode << endl;
      cout << " (" << Nodes[destNode].Lat << ", " << Nodes[destNode].Lon << ")" << endl << endl;


      //run dijkstra's algorithm for each starting node
      map<long long, double> distance1, distance2;
      map<long long, long long> previous1, previous2;

      Dijkstra(G, sNode1, distance1, previous1);
      Dijkstra(G, sNode2, distance2, previous2);

      //retrieve + output paths
      vector<long long> path1 = retrievePath(destNode, previous1);
      vector<long long> path2 = retrievePath(destNode, previous2);

      if (distance1[destNode] == 100000.00 || distance2[destNode] == 100000.00) {
        cout << "Sorry, destination unreachable." << endl;
      }
      else {
        outputPathsAndDistances(path1, distance1[destNode], path2, distance2[destNode]);
      }
      cout << endl;
    }

    
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    
  }    
}

int main() {
  graph<long long, double> G;

  

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  G.BuildGraph(Nodes, Footways);


  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}

