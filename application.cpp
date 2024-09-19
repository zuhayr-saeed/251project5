#include "application.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

double INF = numeric_limits<double>::max();

graph<long long, double> buildGraph(
    const map<long long, Coordinates>& Nodes,
    const vector<FootwayInfo>& Footways,
    const vector<BuildingInfo>& Buildings) {
    graph<long long, double> G;

    // Add each node as a vertex
    for (auto& node : Nodes) {
        G.addVertex(node.first);
    }

    // Add edges for each footway
    for (auto& footway : Footways) {
        for (size_t i = 0; i + 1 < footway.Nodes.size(); i++) {
            double weight = distBetween2Points(
                Nodes.at(footway.Nodes[i]).Lat, Nodes.at(footway.Nodes[i]).Lon,
                Nodes.at(footway.Nodes[i+1]).Lat, Nodes.at(footway.Nodes[i+1]).Lon);
            G.addEdge(footway.Nodes[i], footway.Nodes[i+1], weight);
            G.addEdge(footway.Nodes[i+1], footway.Nodes[i], weight); // If bidirectional
        }
    }

    for (const auto& building : Buildings) {
    if (!G.addVertex(building.Coords.ID)) {
        cerr << "Failed to add building vertex: " << building.Coords.ID << endl;
    }

    // Connect buildings to nearby nodes
    for (const auto& nodePair : Nodes) {
        if (nodePair.second.OnFootway) {
            double weight = distBetween2Points(building.Coords.Lat, building.Coords.Lon,
                                               nodePair.second.Lat, nodePair.second.Lon);
            if (weight <= 0.041) {
                G.addEdge(building.Coords.ID, nodePair.first, weight);
                G.addEdge(nodePair.first, building.Coords.ID, weight); // Symmetric edge
            }
        }
    }
}

    return G;
}


vector<long long> dijkstra(
    const graph<long long, double>& G,
    long long start,
    long long target,
    const set<long long>& ignoreNodes) {
    
    map<long long, double> distances;
    map<long long, long long> previous;
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<pair<double, long long>>> pq;

    // Initialize distances and add start node to the queue
    for (auto vertex : G.getVertices()) {
        distances[vertex] = numeric_limits<double>::infinity();
    }
    distances[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        long long currentNode = pq.top().second;
        pq.pop();
        
        if (currentNode == target) break;
        
        // Process each neighbor
        for (auto neighbor : G.neighbors(currentNode)) {
            if (ignoreNodes.find(neighbor) != ignoreNodes.end() && neighbor != target) continue;

            double weight;
            if (G.getWeight(currentNode, neighbor, weight)) {
                double tentativeDistance = distances[currentNode] + weight;
                if (tentativeDistance < distances[neighbor]) {
                    distances[neighbor] = tentativeDistance;
                    previous[neighbor] = currentNode;
                    pq.push({tentativeDistance, neighbor});
                }
            }
        }
    }

    // Reconstruct the shortest path
    vector<long long> path;
    if (distances[target] == numeric_limits<double>::infinity()) return {}; // Path not found

    for (long long at = target; at != start; at = previous[at]) {
        path.push_back(at);
        if (previous.find(at) == previous.end()) return {}; // break if no path
    }
    path.push_back(start);

    // Reverse of the path vector
    size_t n = path.size();
    for (size_t i = 0; i < n / 2; ++i) {
        swap(path[i], path[n - 1 - i]);
    }

    return path;
}


double pathLength(const graph<long long, double>& G, const vector<long long>& path) {
    double length = 0.0;
    double weight;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
        assert(res);
        length += weight;
    }
    return length;
}

void outputPath(const vector<long long>& path) {
    for (size_t i = 0; i < path.size(); i++) {
        cout << path.at(i);
        if (i != path.size() - 1) {
            cout << "->";
        }
    }
    cout << endl;
}

void application(
    const vector<BuildingInfo>& Buildings,
    const graph<long long, double>& G) {
    string person1Building, person2Building;

    set<long long> buildingNodes;
    for (const auto& building : Buildings) {
        buildingNodes.insert(building.Coords.ID);
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);

        //
        // find the building coordinates
        //
        bool foundP1 = false;
        bool foundP2 = false;
        Coordinates P1Coords, P2Coords;
        string P1Name, P2Name;

        for (const BuildingInfo& building : Buildings) {
            if (building.Abbrev == person1Building) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (building.Abbrev == person2Building) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        for (const BuildingInfo& building : Buildings) {
            if (!foundP1 &&
                building.Fullname.find(person1Building) != string::npos) {
                foundP1 = true;
                P1Name = building.Fullname;
                P1Coords = building.Coords;
            }
            if (!foundP2 && building.Fullname.find(person2Building) != string::npos) {
                foundP2 = true;
                P2Name = building.Fullname;
                P2Coords = building.Coords;
            }
        }

        if (!foundP1) {
            cout << "Person 1's building not found" << endl;
        } else if (!foundP2) {
            cout << "Person 2's building not found" << endl;
        } else {
            cout << endl;
            cout << "Person 1's point:" << endl;
            cout << " " << P1Name << endl;
            cout << " (" << P1Coords.Lat << ", " << P1Coords.Lon << ")" << endl;
            cout << "Person 2's point:" << endl;
            cout << " " << P2Name << endl;
            cout << " (" << P2Coords.Lat << ", " << P2Coords.Lon << ")" << endl;

            string destName;
            Coordinates destCoords;

            Coordinates centerCoords = centerBetween2Points(
                P1Coords.Lat, P1Coords.Lon, P2Coords.Lat, P2Coords.Lon);

            double minDestDist = numeric_limits<double>::max();

            for (const BuildingInfo& building : Buildings) {
                double dist = distBetween2Points(
                    centerCoords.Lat, centerCoords.Lon,
                    building.Coords.Lat, building.Coords.Lon);
                if (dist < minDestDist) {
                    minDestDist = dist;
                    destCoords = building.Coords;
                    destName = building.Fullname;
                }
            }

            cout << "Destination Building:" << endl;
            cout << " " << destName << endl;
            cout << " (" << destCoords.Lat << ", " << destCoords.Lon << ")" << endl;

            vector<long long> P1Path = dijkstra(G, P1Coords.ID, destCoords.ID, buildingNodes);
            vector<long long> P2Path = dijkstra(G, P2Coords.ID, destCoords.ID, buildingNodes);

            // This should NEVER happen with how the graph is built
            if (P1Path.empty() || P2Path.empty()) {
                cout << endl;
                cout << "At least one person was unable to reach the destination building. Is an edge missing?" << endl;
                cout << endl;
            } else {
                cout << endl;
                cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P1Path);
                cout << endl;
                cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
                cout << " miles" << endl;
                cout << "Path: ";
                outputPath(P2Path);
            }
        }

        //
        // another navigation?
        //
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
    }
}
