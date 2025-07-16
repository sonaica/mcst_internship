#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <unordered_map>
#include <vector>
#include <stack>
#include <limits>
#include <queue>
#include <algorithm>
#include <iostream>

class Graph {
public:
    struct Node {
        std::string id;
        std::vector<std::pair<std::string, int>> outgoing;
        std::vector<std::pair<std::string, int>> incoming;
    };

    bool hasNode(const std::string& nodeId) const;
    void addNode(const std::string& nodeId);
    void addEdge(const std::string& from, const std::string& to, int weight);
    void removeNode(const std::string& nodeId);
    void removeEdge(const std::string& from, const std::string& to);
    std::vector<std::string> rpoNumbering(const std::string& start);
    std::unordered_map<std::string, int> dijkstra(const std::string& start);
    int maxFlow(const std::string& source, const std::string& sink);
    std::vector<std::vector<std::string>> tarjan();

private:
    std::unordered_map<std::string, Node> nodes;
};

#endif
