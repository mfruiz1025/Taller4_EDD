#include <iostream>
#include "Grafo.hxx"

int main() {
    // Crear un grafo de enteros
    Grafo<int> grafo;

    // Insertar vértices
    grafo.insertarVertice(1);
    grafo.insertarVertice(2);
    grafo.insertarVertice(3);
    grafo.insertarVertice(4);

    // Insertar aristas
    grafo.insertarArista(1, 2, 5);
    grafo.insertarArista(2, 3, 3);
    grafo.insertarArista(3, 4, 2);

    // Mostrar vértices
    std::cout << "Vertices en el grafo: ";
    std::vector<int> vertices = grafo.obtenerVertices();
    for (size_t i = 0; i < vertices.size(); ++i) {
        std::cout << vertices[i] << " ";
    }
    std::cout << std::endl;

    // Mostrar aristas
    std::cout << "Aristas en el grafo:" << std::endl;
    std::map<std::pair<int, int>, int> aristas = grafo.obtenerAristas();
    for (std::map<std::pair<int, int>, int>::iterator it = aristas.begin(); it != aristas.end(); ++it) {
        std::cout << "Arista desde " << it->first.first << " a " << it->first.second << " con peso " << it->second << std::endl;
    }

    // Buscar vértice
    int buscar = 3;
    int indice = grafo.buscarVertice(buscar);
    if (indice != -1) {
        std::cout << "El vertice " << buscar << " esta en el grafo en la posicion " << indice << std::endl;
    } else {
        std::cout << "El vertice " << buscar << " no esta en el grafo" << std::endl;
    }

    // Buscar arista
    int pesoArista = grafo.buscarArista(1, 2);
    if (pesoArista != -1) {
        std::cout << "La arista de 1 a 2 tiene un peso de " << pesoArista << std::endl;
    } else {
        std::cout << "No existe una arista de 1 a 2" << std::endl;
    }

    // Eliminar arista
    std::cout << "Eliminando arista de 2 a 3..." << std::endl;
    grafo.eliminarArista(2, 3);

    // Eliminar vértice
    std::cout << "Eliminando vertice 4..." << std::endl;
    grafo.eliminarVertice(4);

    // Recorrido DFS
    std::cout << "Recorrido DFS desde el vertice 1: ";
    std::vector<int> recorridoDFS = grafo.DFS(1);
    for (size_t i = 0; i < recorridoDFS.size(); ++i) {
        std::cout << recorridoDFS[i] << " ";
    }
    std::cout << std::endl;

    // Recorrido BFS
    std::cout << "Recorrido BFS desde el vertice 1: ";
    std::vector<int> recorridoBFS = grafo.BFS(1);
    for (size_t i = 0; i < recorridoBFS.size(); ++i) {
        std::cout << recorridoBFS[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Ejecutando el algoritmo de Prim desde el vertice 1..." << std::endl;
    std::vector<std::pair<int, int>> aristasPrim = grafo.algoritmoPrim(1);

    std::cout << "Aristas seleccionadas por el algoritmo de Prim:" << std::endl;
    for (size_t i = 0; i < aristasPrim.size(); ++i) {
        std::cout << "Arista desde " << aristasPrim[i].first << " a " << aristasPrim[i].second << std::endl;
    }

    // Ejecutar el algoritmo de Dijkstra
    int distancia = grafo.algoritmoDijkstra(1, 4);
    if (distancia != -1) {
        std::cout << "La distancia mas corta de 1 a 4 es: " << distancia << std::endl;
    } else {
        std::cout << "No se puede llegar de 1 a 4." << std::endl;
    }
    return 0;
}