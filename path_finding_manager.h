//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H

#include <unordered_map>
#include <set>
#include <limits>
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

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
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
        std::unordered_map<Node *, Node *> parent;
        std::unordered_map<Node *, double> dist;
        std::set<Entry> pq;
        
        // Inicializar todas las distancias como infinito
        for (auto &[id, node] : graph.nodes) {
            dist[node] = std::numeric_limits<double>::max();
        }
        
        // La distancia al nodo origen es 0
        dist[src] = 0.0;
        pq.insert({src, 0.0});
        parent[src] = nullptr;
        
        while (!pq.empty()) {
            // Extraer el nodo con menor distancia
            Entry current = *pq.begin();
            pq.erase(pq.begin());
            
            Node* current_node = current.node;
            double current_dist = current.dist;
            
            // Si llegamos al destino, terminamos
            if (current_node == dest) {
                break;
            }
            
            // Si la distancia actual es mayor que la registrada, continuar
            if (current_dist > dist[current_node]) {
                continue;
            }
            
            // Explorar todas las aristas del nodo actual
            for (Edge* edge : current_node->edges) {
                Node* neighbor = nullptr;
                
                // Determinar el nodo vecino dependiendo de la dirección de la arista
                if (edge->src == current_node) {
                    neighbor = edge->dest;
                } 
                else if (!edge->one_way && edge->dest == current_node) {
                    neighbor = edge->src;
                } 
                else {
                    continue; // Esta arista no es válida desde current_node
                }
                
                // Calcular la nueva distancia (usando length como peso)
                double new_dist = current_dist + edge->length;
                
                // Si encontramos un camino más corto
                if (new_dist < dist[neighbor]) {
                    // Remover la entrada anterior si existe
                    pq.erase({neighbor, dist[neighbor]});
                    
                    // Actualizar distancia y padre
                    dist[neighbor] = new_dist;
                    parent[neighbor] = current_node;
                    
                    // Insertar con la nueva distancia
                    pq.insert({neighbor, new_dist});
                    
                    // Agregar la arista visitada para visualización
                    visited_edges.push_back(
                        sfLine(current_node->coord, neighbor->coord, 
                               sf::Color(255, 255, 0, 100), 0.5f)
                    );
                    
                    // Renderizar ocasionalmente (cada 100 iteraciones) para mejor rendimiento
                    render_counter++;
                    if (render_counter % 100 == 0) {
                        render();
                    }
                }
            }
        }

        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        // TODO: Add your code here

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
                break;
            }
            
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
