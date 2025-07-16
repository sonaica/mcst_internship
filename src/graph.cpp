#include "graph.h"
#include <functional>

bool Graph::hasNode(const std::string& nodeId) const {
    return nodes.count(nodeId) > 0;
}

void Graph::addNode(const std::string& nodeId) {
    if (!hasNode(nodeId)) {
        nodes[nodeId] = {nodeId, {}, {}};
    }
}

void Graph::addEdge(const std::string& from, const std::string& to, int weight) {
    if (!hasNode(from) && !hasNode(to)) {
        std::cout << "Unknown nodes " << from << " " << to << std::endl;
        return;
    }
    if (!hasNode(from)) {
        std::cout << "Unknown node " << from << std::endl;
        return;
    }
    if (!hasNode(to)) {
        std::cout << "Unknown node " << to << std::endl;
        return;
    }
    nodes[from].outgoing.push_back({to, weight});
    nodes[to].incoming.push_back({from, weight});
}

void Graph::removeNode(const std::string& nodeId) {
    if (!hasNode(nodeId)) {
        std::cout << "Unknown node " << nodeId << std::endl;
        return;
    }

    for (auto& edge : nodes[nodeId].outgoing) {
        auto& neighbors = nodes[edge.first].incoming;
        neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), 
                         std::make_pair(nodeId, edge.second)), neighbors.end());
    }

    for (auto& edge : nodes[nodeId].incoming) {
        auto& neighbors = nodes[edge.first].outgoing;
        neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), 
                         std::make_pair(nodeId, edge.second)), neighbors.end());
    }

    nodes.erase(nodeId);
}

void Graph::removeEdge(const std::string& from, const std::string& to) {
     if (!hasNode(from) && !hasNode(to)) {
        std::cout << "Unknown nodes " << from << " " << to << std::endl;
        return;
    }
    if (!hasNode(from)) {
        std::cout << "Unknown node " << from << std::endl;
        return;
    }
    if (!hasNode(to)) {
        std::cout << "Unknown node " << to << std::endl;
        return;
    }

    auto removePair = [](auto& vec, const auto& value) {
        vec.erase(std::remove(vec.begin(), vec.end(), value), vec.end());
    };

    for (auto it = nodes[from].outgoing.begin(); it != nodes[from].outgoing.end(); ++it) {
        if (it->first == to) {
            removePair(nodes[to].incoming, std::make_pair(from, it->second));
            nodes[from].outgoing.erase(it);
            break;
        }
    }
}

std::vector<std::string> Graph::rpoNumbering(const std::string& start) {
    std::vector<std::string> result;
    if (!hasNode(start)) return result;

    std::unordered_map<std::string, int> color;
    std::stack<std::string> stack;
    bool loopFound = false;

   std::function<void(const std::string&)> dfs = [&](const std::string& u) {
        color[u] = 1;
        for (const auto& edge : nodes[u].outgoing) {
            const std::string& v = edge.first;
            if (!color[v]) {
                dfs(v);
            } else if (color[v] == 1) {
                std::cout << "Found loop " << u << "->" << v << std::endl;
                loopFound = true;
            }
        }
        color[u] = 2;
        stack.push(u);
    };

    dfs(start);

    while (!stack.empty()) {
        result.push_back(stack.top());
        stack.pop();
    }

    return result;
}

std::unordered_map<std::string, int> Graph::dijkstra(const std::string& start) {
    std::unordered_map<std::string, int> distances;
    if (!hasNode(start)) return distances;

    for (const auto& node : nodes) {
        distances[node.first] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;

    auto cmp = [](const auto& a, const auto& b) { return a.second > b.second; };
    std::priority_queue<std::pair<std::string, int>, 
                       std::vector<std::pair<std::string, int>>,
                       decltype(cmp)> pq(cmp);
    pq.push({start, 0});

    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();

        if (current.second > distances[current.first]) continue;

        for (const auto& edge : nodes[current.first].outgoing) {
            int new_dist = current.second + edge.second;
            if (new_dist < distances[edge.first]) {
                distances[edge.first] = new_dist;
                pq.push({edge.first, new_dist});
            }
        }
    }

    return distances;
}

int Graph::maxFlow(const std::string& source, const std::string& sink) {
    if (!hasNode(source) || !hasNode(sink)) return 0;

    std::unordered_map<std::string, std::unordered_map<std::string, int>> flow;
    for (const auto& node : nodes) {
        for (const auto& edge : node.second.outgoing) {
            flow[node.first][edge.first] = edge.second;  
            flow[edge.first][node.first] = 0;            
        }
    }

    int total_flow = 0;
    std::unordered_map<std::string, std::string> parent;

    auto bfs = [&]() {
        parent.clear();
        std::queue<std::string> q;
        q.push(source);
        parent[source] = "";

        while (!q.empty()) {
            std::string u = q.front();
            q.pop();
            for (const auto& [v, capacity] : flow[u]) {
                if (parent.find(v) == parent.end() && capacity > 0) {
                    parent[v] = u;
                    if (v == sink) return true;
                    q.push(v);
                }
            }
        }
        return false;
    };

    while (bfs()) {
        int path_flow = std::numeric_limits<int>::max();
        for (std::string v = sink; v != source; v = parent[v]) {
            std::string u = parent[v];
            path_flow = std::min(path_flow, flow[u][v]);
        }

        for (std::string v = sink; v != source; v = parent[v]) {
            std::string u = parent[v];
            flow[u][v] -= path_flow;
            flow[v][u] += path_flow;
        }

        total_flow += path_flow;
    }

    return total_flow;
}

std::vector<std::vector<std::string>> Graph::tarjan() {
    std::vector<std::vector<std::string>> result;
    std::unordered_map<std::string, int> disc;
    std::unordered_map<std::string, int> low;
    std::unordered_map<std::string, bool> inStack;
    std::stack<std::string> stk;
    int time = 0;

    for (const auto& node : nodes) {
        disc[node.first] = -1;
        inStack[node.first] = false;
    }

    for (const auto& node : nodes) {
        if (disc[node.first] != -1) continue;

        std::stack<std::pair<std::string, bool>> dfsStack;
        dfsStack.push({node.first, false});

        while (!dfsStack.empty()) {
            auto [u, processed] = dfsStack.top();
            dfsStack.pop();

            if (!processed) {
                disc[u] = low[u] = ++time;
                stk.push(u);
                inStack[u] = true;
                dfsStack.push({u, true});

                for (const auto& edge : nodes[u].outgoing) {
                    const std::string& v = edge.first;
                    if (disc[v] == -1) {
                        dfsStack.push({v, false});
                    }
                }
            } else {
                for (const auto& edge : nodes[u].outgoing) {
                    const std::string& v = edge.first;
                    if (inStack[v]) {
                        low[u] = std::min(low[u], low[v]);
                    }
                }

                if (low[u] == disc[u]) {
                    std::vector<std::string> component;
                    while (stk.top() != u) {
                        component.push_back(stk.top());
                        inStack[stk.top()] = false;
                        stk.pop();
                    }
                    component.push_back(stk.top());
                    inStack[stk.top()] = false;
                    stk.pop();

                    if (component.size() > 1) {
                        result.push_back(component);
                    }
                }
            }
        }
    }

    result.erase(
        std::remove_if(result.begin(), result.end(),
            [](const auto& comp) { return comp.size() <= 1; }),
        result.end()
    );

    return result;
}