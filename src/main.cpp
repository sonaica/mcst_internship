#include "graph.h"
#include <sstream>

void processing(Graph& graph, const std::string& command) {
    std::istringstream iss(command);
    std::string cmd;
    iss >> cmd;

    if (cmd == "NODE") {
        std::string nodeId;
        iss >> nodeId;
        graph.addNode(nodeId);
    }
    else if (cmd == "EDGE") {
        std::string from, to;
        int weight;
        iss >> from >> to >> weight;
        graph.addEdge(from, to, weight);
    }
    else if (cmd == "REMOVE") {
        std::string type, arg1;
        iss >> type;
        if (type == "NODE") {
            iss >> arg1;
            graph.removeNode(arg1);
        }
        else if (type == "EDGE") {
            std::string arg2;
            iss >> arg1 >> arg2;
            graph.removeEdge(arg1, arg2);
        }
    }
    else if (cmd == "RPO_NUMBERING") {
        std::string start;
        iss >> start;
        auto rpo = graph.rpoNumbering(start);
        for (const auto& node : rpo) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }
    else if (cmd == "DIJKSTRA") {
        std::string start;
        iss >> start;
        auto distances = graph.dijkstra(start);
        for (const auto& d : distances) {
            std::cout << d.first << " " << d.second << std::endl;
        }
    }
    else if (cmd == "MAX_FLOW") {
        std::string source, sink;
        iss >> source >> sink;
        std::cout << graph.maxFlow(source, sink) << std::endl;
    }
    else if (cmd == "TARJAN") {
        auto scc = graph.tarjan();
        for (const auto& comp : scc) {
            for (const auto& node : comp) {
                std::cout << node << " ";
            }
            std::cout << std::endl;
        }
    }
}

int main() {
    Graph graph;
    std::string line;
    
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        processing(graph, line);
    }
    
    return 0;
}