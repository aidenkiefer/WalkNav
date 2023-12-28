// graph.h 
// Aiden Kiefer
//
// Basic graph class using adjacency list representation.  Currently
// unlimited to a graph with any number of vertices.
//
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <string>
#include <unordered_map>

#include "osm.h"
#include "dist.h"


using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:

  // unordered map implementation of an adjacency list to store vertices 
  //and their respective neighbors with weights (distances) between
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> AdjList;
  


 public:
  //
  // constructor:
  //
  // Constructs an empty graph where n is the max # of vertices
  // you expect the graph to contain.
  //
  graph() {}

  //
  // BuildGraph
  //
  // builds graph from footways

  void BuildGraph(const map<long long, Coordinates>& Nodes, const vector<FootwayInfo>& Footways) {
    //add nodes for vertices
    for (const auto& node : Nodes) {
        this->addVertex(node.first);
    }

    //add edges based on footways
    for (const auto& footway : Footways) {
      const vector<long long>& nodes = footway.Nodes;

      // iterate thru nodes of footway + add edges to the graph
      for (size_t i = 0; i < nodes.size() - 1; ++i) {
        long long node1 = nodes[i];
        long long node2 = nodes[i + 1];

        // calculate distance between nodes using distBetween2Points function
        WeightT distance = distBetween2Points(Nodes.at(node1).Lat, Nodes.at(node1).Lon, Nodes.at(node2).Lat, Nodes.at(node2).Lon);

        // add edge between node1 and node2 with the calculated distance
        this->addEdge(node1, node2, distance);
      }
    }
  }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return AdjList.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;

    //
    // loop through the adjacency list and count how many
    // edges currently exist:
    //
    for (const auto& it : AdjList){
      const auto& edges = it.second;
      count += edges.size();
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    //
    // is the vertex already in the graph?  If so, we do not
    // insert again otherwise Vertices may fill with duplicates:
    //
    if (AdjList.find(v) != AdjList.end()) {
      return false;
    }

    //
    // if we get here, vertex does not exist so insert new element to
    //map using the vertex as the key
    AdjList[v] = unordered_map<VertexT, WeightT>();

    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    //
    // we need to check if both vertices exist in the map
    //
    if (AdjList.find(from) == AdjList.end() || AdjList.find(to) == AdjList.end()) {
      return false;
    }

    //
    // the vertices exist and we have the row and col of the
    // adjacency matrix, so insert / update the edge:
    //
    AdjList[from][to] = weight;

    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    
    //
    // do the vertices exist,  does the edge exist?
    //
    if (AdjList.find(from) == AdjList.end() || AdjList.at(from).find(to) == AdjList.at(from).end()) {  // no:
      return false;
    }

    //
    // Okay, the edge exists, set weight
    // 
    weight = AdjList.at(from).at(to);

    return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;

    //
    // we need to search the Vertices and find the position
    // of v, that will be the row we need in the adjacency
    // matrix:
    //
    if (AdjList.find(v) == AdjList.end()) {
      return S;
    }

    for (const auto& entry : AdjList.at(v)) {
      S.insert(entry.first);
    }

    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;
    for (const auto& entry : AdjList) {
      vertices.push_back(entry.first);
    }
    return vertices;  // returns a copy:
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    for (const auto& entry : AdjList) {
      output << " " << entry.first << endl;
    }

    output << endl;
    output << "**Edges:" << endl;
    for (const auto& entry : AdjList) {
      output << " row " << entry.first << ": ";

      for (const auto& edge : entry.second) {
        output << "(" << edge.first << ", " << edge.second << ") ";
      }
      output << endl;
    }
    output << "**************************************************" << endl;
  }
};
