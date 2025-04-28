#include <iostream>
#include <vector>
#include <queue>
#include <omp.h>
using namespace std;

// Define a graph structure where each node has an adjacency list
struct Node {
    vector<int> adj;
};

// Parallel BFS function
void parallelBFS(vector<Node>& graph, int startNode, int n) {
    vector<int> visited(n, 0);  // To track visited nodes
    queue<int> q;
    q.push(startNode);
    visited[startNode] = 1;

    cout << "BFS starting from node " << startNode << ": ";
    
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        cout << node << " ";

        // Parallelizing the exploration of neighbors
        #pragma omp parallel for
        for (int i = 0; i < graph[node].adj.size(); i++) {
            int neighbor = graph[node].adj[i];
            if (!visited[neighbor]) {
                #pragma omp critical
                {
                    visited[neighbor] = 1;
                    q.push(neighbor);
                }
            }
        }
    }
    cout << endl;
}

int main() {
    // Define a graph with 6 nodes
    int n = 7;
    vector<Node> graph(n);

    // Define adjacency list for the graph
    graph[0].adj = {1, 2};
    graph[1].adj = {0, 3};
    graph[2].adj = {0, 4, 6};
    graph[3].adj = {1, 5};
    graph[4].adj = {2};
    graph[5].adj = {3};  // No neighbors for node 5
    graph[6].adj = {2};

    // Perform BFS starting from node 0
    parallelBFS(graph, 0, n);

    return 0;
}
