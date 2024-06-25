#include <iostream>
#include <vector>
#include <cstring>
using namespace std;

#define INF 0x3f3f3f3f

class Graph {
    int V;
    vector<vector<int>> adjMatrix;

public:
    Graph(int V);
    void addEdge(int u, int v, int w);
    void BFS(int start);
    void DFSUtil(int v, vector<bool>& visited);
    void DFS(int start);
    void Dijkstra(int src);
    void FloydWarshall();
};

Graph::Graph(int V) {
    this->V = V;
    adjMatrix = vector<vector<int>>(V, vector<int>(V, INF));
    for (int i = 0; i < V; ++i)
        adjMatrix[i][i] = 0;
}

void Graph::addEdge(int u, int v, int w) {
    adjMatrix[u][v] = w;
    adjMatrix[v][u] = w; // Si es un grafo no dirigido
}

void Graph::BFS(int start) {
    vector<bool> visited(V, false);
    vector<int> queue(V);
    int front = 0, rear = 0;

    visited[start] = true;
    queue[rear++] = start;

    while (front < rear) {
        int node = queue[front++];
        cout << node << " ";

        for (int i = 0; i < V; ++i) {
            if (adjMatrix[node][i] != INF && !visited[i]) {
                visited[i] = true;
                queue[rear++] = i;
            }
        }
    }
    cout << endl;
}

void Graph::DFSUtil(int v, vector<bool>& visited) {
    visited[v] = true;
    cout << v << " ";

    for (int i = 0; i < V; ++i) {
        if (adjMatrix[v][i] != INF && !visited[i])
            DFSUtil(i, visited);
    }
}

void Graph::DFS(int start) {
    vector<bool> visited(V, false);
    DFSUtil(start, visited);
    cout << endl;
}

void Graph::Dijkstra(int src) {
    vector<int> dist(V, INF);
    vector<bool> sptSet(V, false);

    dist[src] = 0;

    for (int count = 0; count < V - 1; ++count) {
        int u = -1;

        for (int i = 0; i < V; ++i)
            if (!sptSet[i] && (u == -1 || dist[i] < dist[u]))
                u = i;

        sptSet[u] = true;

        for (int v = 0; v < V; ++v)
            if (!sptSet[v] && adjMatrix[u][v] != INF && dist[u] != INF && dist[u] + adjMatrix[u][v] < dist[v])
                dist[v] = dist[u] + adjMatrix[u][v];
    }

    cout << "Dijkstra's algorithm:" << endl;
    for (int i = 0; i < V; ++i)
        cout << "Vertex " << i << " Distance from Source " << dist[i] << endl;
}

void Graph::FloydWarshall() {
    vector<vector<int>> dist = adjMatrix;

    for (int k = 0; k < V; ++k) {
        for (int i = 0; i < V; ++i) {
            for (int j = 0; j < V; ++j) {
                if (dist[i][k] != INF && dist[k][j] != INF && dist[i][k] + dist[k][j] < dist[i][j])
                    dist[i][j] = dist[i][k] + dist[k][j];
            }
        }
    }

    cout << "Floyd-Warshall algorithm:" << endl;
    for (int i = 0; i < V; ++i) {
        for (int j = 0; j < V; ++j) {
            if (dist[i][j] == INF)
                cout << "INF ";
            else
                cout << dist[i][j] << " ";
        }
        cout << endl;
    }
}

int main() {
    int V = 5;
    Graph g(V);

    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 7);
    g.addEdge(2, 3, 3);
    g.addEdge(3, 4, 1);

    cout << "BFS from vertex 0:" << endl;
    g.BFS(0);

    cout << "DFS from vertex 0:" << endl;
    g.DFS(0);

    g.Dijkstra(0);

    g.FloydWarshall();

    return 0;
}
