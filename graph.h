#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
   private:
   // Internal storage using an unordered_map, mapping each vertex to another map.
    // The nested map associates each adjacent vertex with a weight.
    unordered_map<VertexT, map<VertexT, WeightT>> adjacency_list;

   public:
    /// Default constructor
    graph() {}

    /// @brief Add the vertex `v` to the graph, must run in at most O(log |V|).
    /// @param v
    /// @return true if successfully added; false if it existed already
    bool addVertex(VertexT v) {
        if (adjacency_list.find(v) == adjacency_list.end()) {
            adjacency_list[v] = {};
            return true;
        }
        return false;
    }

    /// @brief Add or overwrite directed edge in the graph, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight edge weight / label
    /// @return true if successfully added or overwritten;
    ///         false if either vertices isn't in graph
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
        if (adjacency_list.find(from) != adjacency_list.end() && adjacency_list.find(to) != adjacency_list.end()) {
            adjacency_list[from][to] = weight;
            return true;
        }
        return false;
    }

    /// @brief Maybe get the weight associated with a given edge, must run in at most O(log |V|).
    /// @param from starting vertex
    /// @param to ending vertex
    /// @param weight output parameter
    /// @return true if the edge exists, and `weight` is set;
    ///         false if the edge does not exist
    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
        auto it = adjacency_list.find(from);
        if (it != adjacency_list.end()) {
            auto it2 = it->second.find(to);
            if (it2 != it->second.end()) {
                weight = it2->second;
                return true;
            }
        }
        return false;
    }

    /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
    /// @param v
    /// @return vertices that v has an edge to
    set<VertexT> neighbors(VertexT v) const {
        set<VertexT> S;
        auto it = adjacency_list.find(v);
        if (it != adjacency_list.end()) {
            for (const auto& pair : it->second) {
                S.insert(pair.first);
            }
        }
        return S;
    }

    /// @brief Return a vector containing all vertices in the graph
    vector<VertexT> getVertices() const {
        vector<VertexT> vertices;
        for (const auto& pair : adjacency_list) {
            vertices.push_back(pair.first);
        }
        return vertices;
    }

    /// @brief Get the number of vertices in the graph. Runs in O(1).
    size_t NumVertices() const {
        return adjacency_list.size();
    }

    /// @brief Get the number of directed edges in the graph. Runs in at most O(|V|).
    size_t NumEdges() const {
        size_t count = 0;
        for (const auto& pair : adjacency_list) {
            count += pair.second.size();
        }
        return count;
    }
};
