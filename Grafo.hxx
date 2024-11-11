#ifndef GRAFO_HXX
#define GRAFO_HXX

#include <iostream>
#include "Grafo.h"
#include <limits>
#include <queue>
#include <set>
#include <vector>
#include <map>

// Constructor
template <typename T>
Grafo<T>::Grafo() {}

// Set vertices
template <typename T>
void Grafo<T>::fijarVertices(const std::vector<T>& vertices) {
    this->vertices = vertices;
}

// Set edges
template <typename T>
void Grafo<T>::fijarAristas(const std::map<std::pair<T, T>, int>& aristas) {
    this->aristas = aristas;
}

// Get vertices
template <typename T>
const std::vector<T>& Grafo<T>::obtenerVertices() const {
    return this->vertices;
}

// Obtener un vértice por su índice
template <typename T>
T Grafo<T>::obtenerVertice(int indice) const {
    if (indice >= 0 && indice < vertices.size()) {
        return vertices[indice];
    }
}


// Get edges
template <typename T>
std::map<std::pair<T, T>, int> Grafo<T>::obtenerAristas() const {
    return this->aristas;
}

// Get the number of vertices
template <typename T>
int Grafo<T>::cantVertices() const {
    return vertices.size();
}

// Get the number of edges
template <typename T>
int Grafo<T>::cantAristas() const {
    return aristas.size();
}

// Search for a vertex
template <typename T>
int Grafo<T>::buscarVertice(T ver) const {
    typename std::vector<T>::const_iterator it = std::find(vertices.begin(), vertices.end(), ver);
    return (it != vertices.end()) ? std::distance(vertices.begin(), it) : -1;
}

// Insert a vertex
template <typename T>
bool Grafo<T>::insertarVertice(T ver) {
    if (buscarVertice(ver) == -1) {
        vertices.push_back(ver);
        return true;
    }
    return false;
}

// Insert an edge
template <typename T>
bool Grafo<T>::insertarArista(T ori, T des, int cos) {
    if (buscarVertice(ori) != -1 && buscarVertice(des) != -1) {
        std::pair<T, T> arista(ori, des);
        if (aristas.find(arista) == aristas.end()) {
            aristas[arista] = cos;
            return true;
        }
    }
    return false;
}

// Insert a non-directed edge
template <typename T>
bool Grafo<T>::insAristaNoDir(T ori, T des, int cos) {
    return insertarArista(ori, des, cos) && insertarArista(des, ori, cos);
}

// Search for an edge
template <typename T>
int Grafo<T>::buscarArista(T origen, T destino) const {
    typename std::map<std::pair<T, T>, int>::const_iterator it = aristas.find({origen, destino});
    return (it != aristas.end()) ? it->second : -1;
}

// Delete a vertex
template <typename T>
bool Grafo<T>::eliminarVertice(T ver) {
    int idx = buscarVertice(ver);
    if (idx == -1) return false;

    vertices.erase(vertices.begin() + idx);
    for (typename std::map<std::pair<T, T>, int>::iterator it = aristas.begin(); it != aristas.end();) {
        if (it->first.first == ver || it->first.second == ver) {
            it = aristas.erase(it);
        } else {
            ++it;
        }
    }
    return true;
}

// Delete an edge
template <typename T>
bool Grafo<T>::eliminarArista(T origen, T destino) {
    return aristas.erase({origen, destino}) > 0;
}

// Delete a non-directed edge
template <typename T>
bool Grafo<T>::elimAristaNoDir(T origen, T destino) {
    return eliminarArista(origen, destino) && eliminarArista(destino, origen);
}

// Get neighbors of a vertex
template <typename T>
std::vector<T> Grafo<T>::vecinosVertice(T ver) const {
    std::vector<T> vecinos;
    typename std::map<std::pair<T, T>, int>::const_iterator it;
    for (it = aristas.begin(); it != aristas.end(); ++it) {
        if (it->first.first == ver) {
            vecinos.push_back(it->first.second);
        }
    }
    std::sort(vecinos.begin(), vecinos.end());
    return vecinos;
}

// Display graph (or similar operation)
template <typename T>
void Grafo<T>::plano() const {
    for (size_t i = 0; i < vertices.size(); ++i) {
        std::cout << vertices[i] << " ";
    }
    std::cout << std::endl;
}

// Depth-First Search
template <typename T>
std::vector<T> Grafo<T>::DFS(T ver_inicial) {
    std::vector<bool> ver_visitados(cantVertices(), false);
    std::vector<T> caminoDFS;
    std::stack<T> pila_ver;

    if (buscarVertice(ver_inicial) == -1) {
        std::cout << "El vertice " << ver_inicial << " no esta dentro del grafo" << std::endl;
        return caminoDFS;
    }

    pila_ver.push(ver_inicial);

    while (!pila_ver.empty()) {
        T ver_actual = pila_ver.top();
        pila_ver.pop();
        int ind = buscarVertice(ver_actual);

        if (!ver_visitados[ind]) {
            std::cout << vertices[ind] << " ";
            ver_visitados[ind] = true;
            caminoDFS.push_back(vertices[ind]);

            std::vector<T> vecinos = vecinosVertice(ver_actual);
            for (int i = vecinos.size() - 1; i >= 0; --i) {
                pila_ver.push(vecinos[i]);
            }
        }
    }

    std::cout << std::endl;
    return caminoDFS;
}

// Breadth-First Search
template <typename T>
std::vector<T> Grafo<T>::BFS(T ver_inicial) {
    std::vector<bool> ver_visitados(cantVertices(), false);
    std::vector<T> caminoBFS;
    std::queue<T> cola_ver;

    if (buscarVertice(ver_inicial) == -1) {
        std::cout << "El vertice " << ver_inicial << " no esta dentro del grafo" << std::endl;
        return caminoBFS;
    }

    cola_ver.push(ver_inicial);

    while (!cola_ver.empty()) {
        T ver_actual = cola_ver.front();
        cola_ver.pop();
        int ind = buscarVertice(ver_actual);

        if (!ver_visitados[ind]) {
            std::cout << vertices[ind] << " ";
            ver_visitados[ind] = true;
            caminoBFS.push_back(vertices[ind]);

            std::vector<T> vecinos = vecinosVertice(ver_actual);
            for (size_t i = 0; i < vecinos.size(); ++i) {
                cola_ver.push(vecinos[i]);
            }
        }
    }

    std::cout << std::endl;
    return caminoBFS;
}

// Prim's Algorithm (MST)
template <typename T>
std::vector<std::pair<T, T>> Grafo<T>::algoritmoPrim(T x) {
    std::vector<T> Vnew;
    std::vector<std::pair<T, T>> Enew;
    Vnew.push_back(x);

    while (Vnew.size() < cantVertices()) {
        T u;
        T v;
        int pesoMinimo = std::numeric_limits<int>::max();

        typename std::map<std::pair<T, T>, int>::iterator it;
        for (it = aristas.begin(); it != aristas.end(); ++it) {
            T vertice1 = it->first.first;
            T vertice2 = it->first.second;
            int peso = it->second;

            if (std::find(Vnew.begin(), Vnew.end(), vertice1) != Vnew.end() &&
                std::find(Vnew.begin(), Vnew.end(), vertice2) == Vnew.end()) {
                if (peso < pesoMinimo) {
                    u = vertice1;
                    v = vertice2;
                    pesoMinimo = peso;
                }
            } 
            else if (std::find(Vnew.begin(), Vnew.end(), vertice2) != Vnew.end() &&
                     std::find(Vnew.begin(), Vnew.end(), vertice1) == Vnew.end()) {
                if (peso < pesoMinimo) {
                    u = vertice2;
                    v = vertice1;
                    pesoMinimo = peso;
                }
            }
        }

        Vnew.push_back(v);
        Enew.push_back(std::make_pair(u, v));
    }

    return Enew;
}

// Explicita la instanciación para los tipos que usarás
template class Grafo<int>;

// Dijkstra Algorithm
template <typename T>
std::map<T, int> Grafo<T>::dijkstra(T origen) {
    std::map<T, int> distancias;
    std::set<std::pair<int, T>> cola;

    typename std::vector<T>::const_iterator vertice;
    for (vertice = vertices.begin(); vertice != vertices.end(); ++vertice) {
        distancias[*vertice] = std::numeric_limits<int>::max();
    }
    distancias[origen] = 0;
    cola.insert(std::make_pair(0, origen));

    while (!cola.empty()) {
        T u = cola.begin()->second;
        cola.erase(cola.begin());

        std::vector<T> vecinos = vecinosVertice(u);
        for (size_t i = 0; i < vecinos.size(); ++i) {
            T v = vecinos[i];
            int peso = buscarArista(u, v);

            if (distancias[u] + peso < distancias[v]) {
                cola.erase(std::make_pair(distancias[v], v));
                distancias[v] = distancias[u] + peso;
                cola.insert(std::make_pair(distancias[v], v));
            }
        }
    }

    return distancias;
}

#endif
