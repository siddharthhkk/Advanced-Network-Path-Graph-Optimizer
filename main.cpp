#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std;

const double INF = numeric_limits<double>::infinity();

class Edge {
private:
    int destination;
    double weight;

public:
    Edge(int dest, double w) : destination(dest), weight(w) {}

    int getDestination() const {
        return destination;
    }

    double getWeight() const {
        return weight;
    }
};

class Graph {
private:
    int vertices;
    vector<vector<Edge>> adjacencyList;

public:
    Graph(int v) : vertices(v), adjacencyList(v) {
        if (v <= 0) {
            throw invalid_argument("Graph must contain at least one vertex.");
        }
    }

    int getVertices() const {
        return vertices;
    }

    void addEdge(int source, int destination, double weight, bool bidirectional = true) {
        validateVertex(source);
        validateVertex(destination);

        if (weight < 0) {
            throw invalid_argument("Negative edge weights are not supported.");
        }

        adjacencyList[source].push_back(Edge(destination, weight));

        if (bidirectional) {
            adjacencyList[destination].push_back(Edge(source, weight));
        }
    }

    const vector<Edge>& getNeighbors(int vertex) const {
        validateVertex(vertex);
        return adjacencyList[vertex];
    }

    void displayGraph() const {
        cout << "\nNetwork Topology:\n";

        for (int i = 0; i < vertices; i++) {
            cout << "Router " << i << " -> ";

            for (const Edge& edge : adjacencyList[i]) {
                cout << "(" << edge.getDestination()
                     << ", cost=" << edge.getWeight() << ") ";
            }

            cout << endl;
        }
    }

private:
    void validateVertex(int vertex) const {
        if (vertex < 0 || vertex >= vertices) {
            throw out_of_range("Invalid router index.");
        }
    }
};

class PathResult {
public:
    vector<int> path;
    double totalCost;
    int nodesVisited;
    double executionTimeMs;

    PathResult() : totalCost(INF), nodesVisited(0), executionTimeMs(0.0) {}
};

class RouterOptimizer {
private:
    const Graph& graph;

public:
    RouterOptimizer(const Graph& g) : graph(g) {}

    PathResult dijkstra(int source, int destination) {
        auto startTime = chrono::high_resolution_clock::now();

        int n = graph.getVertices();
        vector<double> distance(n, INF);
        vector<int> parent(n, -1);
        vector<bool> visited(n, false);

        priority_queue<pair<double, int>,
                       vector<pair<double, int>>,
                       greater<pair<double, int>>> pq;

        PathResult result;

        distance[source] = 0;
        pq.push({0, source});

        while (!pq.empty()) {
            int current = pq.top().second;
            pq.pop();

            if (visited[current]) {
                continue;
            }

            visited[current] = true;
            result.nodesVisited++;

            if (current == destination) {
                break;
            }

            for (const Edge& edge : graph.getNeighbors(current)) {
                int neighbor = edge.getDestination();
                double newDistance = distance[current] + edge.getWeight();

                if (newDistance < distance[neighbor]) {
                    distance[neighbor] = newDistance;
                    parent[neighbor] = current;
                    pq.push({newDistance, neighbor});
                }
            }
        }

        result.totalCost = distance[destination];
        result.path = reconstructPath(parent, source, destination);

        auto endTime = chrono::high_resolution_clock::now();
        result.executionTimeMs =
            chrono::duration<double, milli>(endTime - startTime).count();

        return result;
    }

    PathResult aStar(int source, int destination) {
        auto startTime = chrono::high_resolution_clock::now();

        int n = graph.getVertices();

        vector<double> gScore(n, INF);
        vector<double> fScore(n, INF);
        vector<int> parent(n, -1);
        vector<bool> visited(n, false);

        priority_queue<pair<double, int>,
                       vector<pair<double, int>>,
                       greater<pair<double, int>>> openSet;

        PathResult result;

        gScore[source] = 0;
        fScore[source] = heuristic(source, destination);

        openSet.push({fScore[source], source});

        while (!openSet.empty()) {
            int current = openSet.top().second;
            openSet.pop();

            if (visited[current]) {
                continue;
            }

            visited[current] = true;
            result.nodesVisited++;

            if (current == destination) {
                break;
            }

            for (const Edge& edge : graph.getNeighbors(current)) {
                int neighbor = edge.getDestination();
                double tentativeGScore = gScore[current] + edge.getWeight();

                if (tentativeGScore < gScore[neighbor]) {
                    parent[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = tentativeGScore + heuristic(neighbor, destination);

                    openSet.push({fScore[neighbor], neighbor});
                }
            }
        }

        result.totalCost = gScore[destination];
        result.path = reconstructPath(parent, source, destination);

        auto endTime = chrono::high_resolution_clock::now();
        result.executionTimeMs =
            chrono::duration<double, milli>(endTime - startTime).count();

        return result;
    }

private:
    double heuristic(int current, int destination) {
        return abs(destination - current);
    }

    vector<int> reconstructPath(const vector<int>& parent, int source, int destination) {
        vector<int> path;

        if (source == destination) {
            path.push_back(source);
            return path;
        }

        if (parent[destination] == -1) {
            return path;
        }

        int current = destination;

        while (current != -1) {
            path.push_back(current);
            current = parent[current];
        }

        reverse(path.begin(), path.end());

        return path;
    }
};

class NetworkTestSuite {
public:
    static void runAllTests() {
        cout << "\nRunning Edge Case Tests...\n";

        testDisconnectedGraph();
        testSingleNodeGraph();
        testInvalidEdge();
        testStressGraph();

        cout << "All tests completed successfully.\n";
    }

private:
    static void testDisconnectedGraph() {
        Graph graph(4);
        graph.addEdge(0, 1, 10);
        graph.addEdge(2, 3, 5);

        RouterOptimizer optimizer(graph);
        PathResult result = optimizer.dijkstra(0, 3);

        if (!result.path.empty()) {
            throw runtime_error("Disconnected graph test failed.");
        }

        cout << "[PASS] Disconnected graph handled correctly.\n";
    }

    static void testSingleNodeGraph() {
        Graph graph(1);
        RouterOptimizer optimizer(graph);

        PathResult result = optimizer.dijkstra(0, 0);

        if (result.path.size() != 1 || result.totalCost != 0) {
            throw runtime_error("Single node graph test failed.");
        }

        cout << "[PASS] Single node graph handled correctly.\n";
    }

    static void testInvalidEdge() {
        try {
            Graph graph(3);
            graph.addEdge(0, 5, 10);
            throw runtime_error("Invalid edge test failed.");
        } catch (const out_of_range&) {
            cout << "[PASS] Invalid edge detected correctly.\n";
        }
    }

    static void testStressGraph() {
        const int nodes = 1000;
        Graph graph(nodes);

        for (int i = 0; i < nodes - 1; i++) {
            graph.addEdge(i, i + 1, 1);
        }

        for (int i = 0; i < nodes - 10; i += 10) {
            graph.addEdge(i, i + 10, 2);
        }

        RouterOptimizer optimizer(graph);

        PathResult result = optimizer.dijkstra(0, nodes - 1);

        if (result.path.empty()) {
            throw runtime_error("Stress graph test failed.");
        }

        cout << "[PASS] Stress graph with " << nodes << " routers processed.\n";
    }
};

void printResult(const string& algorithm, const PathResult& result) {
    cout << "\nAlgorithm: " << algorithm << endl;

    if (result.path.empty()) {
        cout << "No route found.\n";
        return;
    }

    cout << "Optimized Path: ";

    for (size_t i = 0; i < result.path.size(); i++) {
        cout << result.path[i];

        if (i != result.path.size() - 1) {
            cout << " -> ";
        }
    }

    cout << "\nTotal Cost: " << result.totalCost;
    cout << "\nNodes Visited: " << result.nodesVisited;
    cout << "\nExecution Time: " << result.executionTimeMs << " ms\n";
}

int main() {
    try {
        Graph network(8);

        network.addEdge(0, 1, 4);
        network.addEdge(0, 2, 2);
        network.addEdge(1, 2, 1);
        network.addEdge(1, 3, 5);
        network.addEdge(2, 3, 8);
        network.addEdge(2, 4, 10);
        network.addEdge(3, 5, 6);
        network.addEdge(4, 5, 3);
        network.addEdge(4, 6, 7);
        network.addEdge(5, 7, 2);
        network.addEdge(6, 7, 1);

        network.displayGraph();

        RouterOptimizer optimizer(network);

        int source = 0;
        int destination = 7;

        PathResult dijkstraResult = optimizer.dijkstra(source, destination);
        PathResult aStarResult = optimizer.aStar(source, destination);

        printResult("Dijkstra", dijkstraResult);
        printResult("A*", aStarResult);

        NetworkTestSuite::runAllTests();

    } catch (const exception& error) {
        cerr << "Error: " << error.what() << endl;
    }

    return 0;
}