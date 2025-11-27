//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H

#include <unordered_map>
#include <set>
#include <queue>
#include <limits>
#include <iostream>
#include <chrono>
#include "graph.h"

using namespace std; 

// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    AStar,
    BestFS
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;
    int render_counter = 0;  // Contador para renderizado ocasional
    int nodes_explored = 0;  // Contador de nodos explorados

    struct Entry {
        Node* node;
        double dist;

        // Para priority_queue: el operador debe ser > para crear un MinHeap
        bool operator < (const Entry& other) const {
            return dist > other.dist;
        }
    };

    
    struct Heu_Entry{
        Node* node;
        double dist_heu;
        Edge* arista; 

        bool operator < (const Heu_Entry& other) const {
            return dist_heu < other.dist_heu;
        }
    };
    
    void dijkstra(Graph &graph) {
        // Iniciamos un temporizador para medir tiempo de ejecución del Dijkstra
        auto start_time = std::chrono::high_resolution_clock::now();
        
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> dist;
        std::priority_queue<Entry> pq;  // MinHeap
        
        // distancias como infinito
        for (auto &[id, node] : graph.nodes) {
            dist[node] = std::numeric_limits<double>::max();
        }
        
        // distancia al nodo origen 0
        dist[src] = 0.0;
        pq.push({src, 0.0});
        parent[src] = nullptr;
        
        while (!pq.empty()) {
            // extraer el nodo con menor distancia
            Entry current = pq.top();
            pq.pop();
            
            Node* current_node = current.node;
            double current_dist = current.dist;
            
            // si llegamos al destino, terminamos
            if (current_node == dest) {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                
                std::cout << "\n----- DIJKSTRA -----" << std::endl;
                std::cout << "Nodos explorados: " << nodes_explored << std::endl;
                std::cout << "Tiempo de ejecucion: " << duration.count() << " us ("
                          << duration.count() / 1000.0 << " ms)" << std::endl;
                break;
            }
            
            nodes_explored++;
            
            // si la distancia actual es mayor que la registrada, continuar
            if (current_dist > dist[current_node]) {
                continue;
            }
            
            // explorar todas las aristas del nodo actual
            for (Edge* edge : current_node->edges) {
                Node* neighbor = nullptr;
                
                // determinar el nodo vecino dependiendo de la dirección de la arista
                if (edge->src == current_node) {
                    neighbor = edge->dest;
                } 
                else if (!edge->one_way && edge->dest == current_node) {
                    neighbor = edge->src;
                } 
                else {
                    continue; // Esta arista no es válida desde current_node
                }
                
                // calcular la nueva distancia (usando length como peso)
                double new_dist = current_dist + edge->length;
                
                // si encontramos un camino más corto
                if (new_dist < dist[neighbor]) {
                    // Actualizamos distancia y padre
                    dist[neighbor] = new_dist;
                    parent[neighbor] = current_node;
                    
                    // insertar con la nueva distancia (no necesitamos remover en priority_queue)
                    pq.push({neighbor, new_dist});
                    
                    // Agregar la arista visitada para visualización
                    visited_edges.push_back(
                        sfLine(current_node->coord, neighbor->coord, 
                               sf::Color(255, 255, 0, 100), 0.5f)
                    );
                    
                    // renderizar ocasionalmente (cada 100 iteraciones) para mejor rendimiento
                    render_counter++;
                    if (render_counter % 100 == 0) {
                        render();
                    }
                }
            }
        }

        set_final_path(parent);
    }

    double heuristic(Node* from, Node* to) {
        double dx = from->coord.x - to->coord.x;
        double dy = from->coord.y - to->coord.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void a_star(Graph &graph) {
        // iniciamos un temporizador para calcular el tiempo de ejecución del A*
        auto start_time = std::chrono::high_resolution_clock::now();
        
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> g_score;  // costo real desde el origin
        std::unordered_map<Node *, double> f_score;  // g + heurística
        std::priority_queue<Entry> pq;  // MinHeap
        
        // distancias como infinito
        for (auto &[id, node] : graph.nodes) {
            g_score[node] = std::numeric_limits<double>::max();
            f_score[node] = std::numeric_limits<double>::max();
        }
        
        // distancia al nodo origen será 0
        g_score[src] = 0.0;
        f_score[src] = heuristic(src, dest);
        pq.push({src, f_score[src]});
        parent[src] = nullptr;
        
        while (!pq.empty()) {
            // extraemos el nodo con menor f_score
            Entry current = pq.top();
            pq.pop();
            
            Node* current_node = current.node;
            
            // si llegamos al destino, terminamos
            if (current_node == dest) {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                
                std::cout << "\n----- A* -----" << std::endl;
                std::cout << "Nodos explorados: " << nodes_explored << std::endl;
                std::cout << "Tiempo de ejecucion: " << duration.count() << " ms ("
                          << duration.count() / 1000.0 << " ms)" << std::endl;
                break;
            }
            
            nodes_explored++;
            
            // si el f_score actual es mayor que el registrado, continuar
            if (current.dist > f_score[current_node]) {
                continue;
            }
            
            // explorar todas las aristas del nodo actual
            for (Edge* edge : current_node->edges) {
                Node* neighbor = nullptr;
                
                // determinames el nodo vecino dependiendo de la dirección de la arista
                if (edge->src == current_node) {
                    neighbor = edge->dest;
                } else if (!edge->one_way && edge->dest == current_node) {
                    neighbor = edge->src;
                } else {
                    continue; // Esta arista no es válida desde current_node
                }
                
                // calculamos el nuevo g_score (costo real)
                double tentative_g_score = g_score[current_node] + edge->length;
                
                // si se encontramos un camino mejor
                if (tentative_g_score < g_score[neighbor]) {
                    // sctualizar g_score, f_score y padre
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, dest);
                    parent[neighbor] = current_node;
                    
                    // insertar con el nuevo f_score (no necesitamos remover en priority_queue)
                    pq.push({neighbor, f_score[neighbor]});
                    
                    // agregar la arista visitada para visualización
                    visited_edges.push_back(
                        sfLine(current_node->coord, neighbor->coord, 
                               sf::Color(0, 255, 255, 100), 0.5f)  // Cyan para A*
                    );
                    
                    // renderizamos ocasionalmente (cada 100 iteraciones) para mejor rendimiento
                    render_counter++;
                    if (render_counter % 100 == 0) {
                        render();
                    }
                }
            }
        }

        set_final_path(parent);
    }

    double calcula_heu(Node* dest, Node* init){
        if (init == nullptr || dest == nullptr){
            throw("Error, inicio o destino son punteros nulos"); 
        }
        
        // Calcular la diferencia en coordenadas x e y
        double delta_x = dest->coord.x - init->coord.x;
        double delta_y = dest->coord.y - init->coord.y;

        return sqrt(delta_x * delta_x + delta_y * delta_y);
    }

    void best_first_search(Graph &graph){
        // Iniciamos un temporizador para medir tiempo de ejecución del Best First Search
        auto start_time = std::chrono::high_resolution_clock::now();
        
        std::unordered_map<Node *, Node *> parent; // Para formar el camino de regreso
        std::unordered_map<Node *, bool> visited; // Marca los nodos ya visitados
        std::set<Heu_Entry> queue; // Para obtener la heurística más conveniente (menor costo)
        
        // Inicializar el nodo origen
        queue.insert({src, 0.0, nullptr});
        parent[src] = nullptr;
        visited[src] = false;

        while (!queue.empty()) {
            // Extraer el nodo con menor distancia heurística
            Heu_Entry current = *queue.begin();
            queue.erase(queue.begin());
            
            Node* current_node = current.node;
            
            // Si el nodo ya fue visitado, continuar
            if (visited[current_node]) {
                continue;
            }
            
            // Marcar el nodo como visitado
            visited[current_node] = true;
            
            // Si la arista no es nula, graficar la arista (no se grafica para el nodo origen)
            if (current.arista != nullptr) {
                // Graficar la arista basándose en el nodo actual y su nodo padre
                Node* prev_node = parent[current_node];
                if (prev_node != nullptr) {
                    visited_edges.push_back(
                        sfLine(prev_node->coord, current_node->coord, 
                               sf::Color(255, 255, 0, 100), 0.5f)
                    );
                }
            }
            
            // Si llegamos al destino, terminamos
            if (current_node == dest) {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                
                std::cout << "\n----- BEST FIRST SEARCH -----" << std::endl;
                std::cout << "Nodos explorados: " << nodes_explored << std::endl;
                std::cout << "Tiempo de ejecucion: " << duration.count() << " us ("
                          << duration.count() / 1000.0 << " ms)" << std::endl;
                break;
            }
            
            nodes_explored++;
            
            // Renderizar ocasionalmente (cada 100 iteraciones) para mejor rendimiento
            render_counter++;
            if (render_counter % 100 == 0) {
                render();
            }
            
            // Explorar todas las aristas del nodo actual
            for (Edge* edge : current_node->edges) {
                Node* neighbor = nullptr;
                
                // Determinar el nodo vecino dependiendo de la dirección de la arista
                if (edge->src == current_node) {
                    neighbor = edge->dest;
                } else if (!edge->one_way && edge->dest == current_node) {
                    neighbor = edge->src;
                } else {
                    continue; // Esta arista no es válida desde current_node
                }
                
                // Si el nodo vecino no ha sido visitado
                if (!visited[neighbor]) {
                    // Calcular la distancia heurística del vecino hacia el destino
                    double heuristic_dist = calcula_heu(dest, neighbor);
                    
                    // Actualizar el padre del vecino
                    parent[neighbor] = current_node;
                    
                    // Agregar el vecino a la cola con su distancia heurística y la arista
                    queue.insert({neighbor, heuristic_dist, edge});
                }
            }
        }
        
        set_final_path(parent);
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(1));  // Reducido de 10ms a 1ms
        
        // Limpiar la ventana
        window_manager->clear();
        
        // Dibujar todas las aristas visitadas
        for (sfLine &line : visited_edges) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }
        
        // Dibujar el nodo origen
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }
        
        // Dibujar el nodo destino
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
        
        // Mostrar el frame actual
        window_manager->display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        Node* current = dest;
        
        // Verificar que existe un camino
        if (parent.find(dest) == parent.end()) {
            return; // No hay camino al destino
        }
        
        // Reconstruir el camino desde dest hasta src
        std::vector<sfLine> temp_path;
        while (current != nullptr && parent[current] != nullptr) {
            Node* prev = parent[current];
            // Crear línea del camino con color verde brillante y mayor grosor
            temp_path.push_back(
                sfLine(prev->coord, current->coord, 
                       sf::Color(0, 255, 0), 2.5f)
            );
            current = prev;
        }
        
        // Invertir el camino para que vaya de src a dest
        path.clear();
        for (auto it = temp_path.rbegin(); it != temp_path.rend(); ++it) {
            path.push_back(*it);
        }
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }

        // Resetear las aristas y caminos anteriores
        visited_edges.clear();
        path.clear();
        render_counter = 0;  // Reset del contador
        nodes_explored = 0;  // Reset del contador de nodos
        
        // Ejecutar el algoritmo seleccionado
        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            case BestFS:
                best_first_search(graph);
                break;
            case None:
            default:
                break;
        }
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
