#ifndef GRAFO_H
#define GRAFO_H

#include <vector>
#include <map>
#include <iostream>
#include <stack>
#include <queue>
#include <algorithm>
#include <utility>

template <typename T>
class Grafo {
private:
    std::vector<T> vertices;
    std::map<std::pair<T, T>, int> aristas;

public:
    Grafo();  // Constructor

    void fijarVertices(const std::vector<T>& vertices);  // Set vertices
    void fijarAristas(const std::map<std::pair<T, T>, int>& aristas);  // Set edges
    const std::vector<T>& obtenerVertices() const;  // Get vertices
    std::map<std::pair<T, T>, int> obtenerAristas() const;  // Get edges

    int cantVertices() const;  // Get the number of vertices
    int cantAristas() const;  // Get the number of edges
    int buscarVertice(T ver) const;  // Search for a vertex
    bool insertarVertice(T ver);  // Insert a vertex
    bool insertarArista(T ori, T des, int cos);  // Insert an edge
    bool insAristaNoDir(T ori, T des, int cos);  // Insert a non-directed edge
    int buscarArista(T origen, T destino) const;  // Search for an edge
    bool eliminarVertice(T ver);  // Delete a vertex
    bool eliminarArista(T origen, T destino);  // Delete an edge
    bool elimAristaNoDir(T origen, T destino);  // Delete a non-directed edge
    std::vector<T> vecinosVertice(T ver) const;  // Get neighbors of a vertex
    void plano() const;  // Display graph (or similar operation)

    std::vector<T> DFS(T ver_inicial);  // Depth-First Search
    std::vector<T> BFS(T ver_inicial);  // Breadth-First Search
    std::vector<std::pair<T, T>>algoritmoPrim(T x); 
    int algoritmoDijkstra(const T begin, const T end); 
};

#include "Grafo.hxx"  // Include the implementation

#endif // GRAFO_H
