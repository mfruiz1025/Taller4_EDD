#ifndef GRAFO_HXX
#define GRAFO_HXX

#include "Grafo.h"

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
    std::vector<T> Vnew;  // Conjunto de vértices seleccionados
    std::vector<std::pair<T, T>> Enew;  // Conjunto de aristas seleccionadas
    Vnew.push_back(x);

    // Mientras no todos los vértices hayan sido añadidos a Vnew
    while (Vnew.size() < cantVertices()) {
        T u;
        T v;
        int pesoMinimo = std::numeric_limits<int>::max();  // Peso mínimo de la arista

        // Buscar la arista con peso mínimo, donde uno de los vértices está en Vnew y el otro no
        typename std::map<std::pair<T, T>, int>::iterator it;
        for (it = aristas.begin(); it != aristas.end(); ++it) {
            T vertice1 = it->first.first;
            T vertice2 = it->first.second;
            int peso = it->second;

            // Si uno de los vértices está en Vnew y el otro no
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

        // Añadir el vértice v a Vnew y la arista (u, v) a Enew
        Vnew.push_back(v);
        Enew.push_back(std::make_pair(u, v));
    }

    return Enew;
}

// Explicita la instanciación para los tipos que usarás
template class Grafo<int>;  

template <typename T>
int Grafo<T>::algoritmoDijkstra(const T begin, const T end) {
    // Inicialización de la cola de prioridad y las distancias
    std::priority_queue<std::pair<int, T>, std::vector<std::pair<int, T>>, std::greater<std::pair<int, T>>> pq;
    std::map<T, int> Dist;  // Mapa para las distancias de cada vértice
    std::map<T, bool> mark; // Mapa para los vértices visitados

    // Inicializamos las distancias a infinito y el vértice de inicio con 0
    for (size_t i = 0; i < vertices.size(); ++i) {
        Dist[vertices[i]] = std::numeric_limits<int>::max();
        mark[vertices[i]] = false;
    }

    Dist[begin] = 0;
    pq.push(std::make_pair(0, begin));  // Empezamos con el vértice de inicio

    while (!pq.empty()) {
        // Extraemos el vértice con el menor costo
        std::pair<int, T> front = pq.top();
        pq.pop();

        int cost = front.first;
        T node = front.second;

        // Si ya hemos procesado este nodo, lo ignoramos
        if (mark[node]) continue;

        // Marcamos el nodo como procesado
        mark[node] = true;

        // Si hemos llegado al nodo destino, retornamos el costo
        if (node == end) {
            return cost;
        }

        // Iteramos sobre las aristas del nodo actual
        typename std::map<std::pair<T, T>, int>::iterator it;
        for (it = aristas.begin(); it != aristas.end(); ++it) {
            T u = it->first.first;  // Primer vértice de la arista
            T v = it->first.second; // Segundo vértice de la arista
            int peso = it->second;  // Peso de la arista

            // Verificamos si la arista conecta el nodo actual con otro vértice
            if (u == node && !mark[v]) {
                // Relajamos la distancia si encontramos un camino más corto
                if (cost + peso < Dist[v]) {
                    Dist[v] = cost + peso;
                    pq.push(std::make_pair(Dist[v], v));
                }
            } else if (v == node && !mark[u]) {
                // Relajamos la distancia si encontramos un camino más corto
                if (cost + peso < Dist[u]) {
                    Dist[u] = cost + peso;
                    pq.push(std::make_pair(Dist[u], u));
                }
            }
        }
    }

    return -1;  // Si no se puede llegar al nodo de destino, retornamos -1
}
#endif // GRAFO_HXX
