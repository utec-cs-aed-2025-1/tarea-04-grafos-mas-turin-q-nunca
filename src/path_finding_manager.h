//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H

#include <set>
#include <unordered_map>
#include "graph.h"
#include "window_manager.h"

static constexpr double INF = std::numeric_limits<double>::infinity();
static constexpr double HEURISTIC_MULTIPLIER = 200.0;

// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    GreedyBfs,
    Dijkstra,
    AStar
};

//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que
//     'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso
//     del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager* window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    double euclidean(const sf::Vector2f& point) {
        sf::Vector2f query = dest->coord;
        return std::sqrt(std::pow((point.x - (double)query.x), 2) +
                         std::pow((point.y - (double)query.y), 2));
    }

    std::unordered_map<std::size_t, double> heuristic_table;
    void calculate_heuristic(Graph& graph) {
        if (src == nullptr || dest == nullptr) {
            return;
        }
        heuristic_table.clear();
        for (const auto& [id, node] : graph.nodes) {
            heuristic_table[id] = HEURISTIC_MULTIPLIER * euclidean(node->coord);
        }
    }

    template<typename F>
    void pepito(Graph& graph,
                std::unordered_map<Node*, Node*>& parent,
                std::map<std::size_t, double>& dist,
                F&& distance) {
        for (const auto& [id, node] : graph.nodes) {
            dist[id] = INF;
            parent[node] = nullptr;
        }

        std::set<std::pair<double, std::size_t>> q;

        dist[src->id] = 0.0;
        q.emplace(distance(src->id), src->id);

        while (!q.empty()) {
            const auto& [_, v_id] = *q.begin();
            q.erase(q.begin());

            if (v_id == dest->id)
                break;
            for (const auto& edge : graph.nodes[v_id]->edges) {
                const std::size_t dest_id = edge->dest->id;

                if (dist[v_id] + edge->length < dist[dest_id]) {
                    q.erase({distance(dest_id), dest_id});

                    dist[dest_id] = dist[v_id] + edge->length;
                    parent[edge->dest] = graph.nodes[v_id];

                    q.emplace(distance(dest_id), dest_id);
                    visited_edges.emplace_back(edge->src->coord, edge->dest->coord,
                                               sf::Color(0, 0, 255), default_thickness);
                    static uint64_t counter = 0;
                    if (++counter >= 50) {
                        counter = 0;
                        render([&] { draw(true); });
                    }
                }
            }
        }
    }

    void greedy_bfs(Graph& graph, std::unordered_map<Node*, Node*>& parent) {
        calculate_heuristic(graph);
        std::map<std::size_t, double> dist;

        pepito(graph, parent, dist, [&](const std::size_t v_id) { return heuristic_table[v_id]; });
    }

    void dijkstra(Graph& graph, std::unordered_map<Node*, Node*>& parent) {
        std::map<std::size_t, double> dist;

        pepito(graph, parent, dist, [&](const std::size_t v_id) { return dist[v_id]; });
    }

    void a_star(Graph& graph, std::unordered_map<Node*, Node*>& parent) {
        calculate_heuristic(graph);
        std::map<std::size_t, double> dist;

        pepito(graph, parent, dist,
               [&](const std::size_t v_id) { return dist[v_id] + heuristic_table[v_id]; });
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el
    // 'window_manager'
    template<typename F>
    void render(F&& draw) {
        sf::sleep(sf::milliseconds(10));
        draw();
        window_manager->display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del
    // algoritmo. 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el
    // vértice anterior a el, formando así el 'path'.
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
    void set_final_path(std::unordered_map<Node*, Node*>& parent) {
        Node* current = dest;

        while (current != src && current != nullptr && parent[current] != nullptr) {
            path.emplace_back(current->coord, parent[current]->coord, sf::Color(0, 255, 0),
                              default_thickness + 2);
            current = parent[current];
        }
    }

public:
    Node* src = nullptr;
    Node* dest = nullptr;

    explicit PathFindingManager(WindowManager* window_manager)
        : window_manager(window_manager) {}

    void exec(Graph& graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }
        path.clear();
        visited_edges.clear();

        std::unordered_map<Node*, Node*> parent;
        switch (algorithm) {
            case GreedyBfs: {
                greedy_bfs(graph, parent);
                break;
            }
            case Dijkstra: {
                dijkstra(graph, parent);
                break;
            }
            case AStar: {
                a_star(graph, parent);
                break;
            }
            default:
                break;
        }

        set_final_path(parent);
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
            for (sfLine& line : visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine& line : path) {
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

#endif  // HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
