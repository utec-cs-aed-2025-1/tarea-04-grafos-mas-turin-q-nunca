//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H

#include <cstddef>
#include <functional>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>
#include "graph.h"
#include "window_manager.h"

// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
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

    struct Entry {
        Node* node;
        double dist;

        bool operator<(const Entry& other) const {
            return dist < other.dist;
        }
    };

    void dijkstra(Graph graph) {
        std::unordered_map<Node*, Node*> parent;
        std::map<std::size_t, double> dist;
        std::set<std::size_t> visited;

        static constexpr double INF = std::numeric_limits<double>::infinity();
        path.clear();
        visited_edges.clear();

        for (const auto& [id, node] : graph.nodes) {
            dist[id] = INF;
            parent[node] = nullptr;
        }

        using Entry = std::pair<double, std::size_t>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> q;

        dist[src->id] = 0.0;
        q.emplace(dist[src->id], src->id);

        while (!q.empty()) {
            const auto& [v_dist, v_id] = q.top();
            q.pop();

            if (v_id == dest->id)
                break;

            for (const auto& edge : graph.nodes[v_id]->edges) {
                Node* const neigh = edge->dest;
                const double neigh_dist = dist[v_id] + edge->length;

                if (neigh_dist < dist[neigh->id]) {
                    dist[neigh->id] = neigh_dist;
                    parent[neigh] = graph.nodes[v_id];

                    q.emplace(dist[neigh->id], neigh->id);
                    visited_edges.emplace_back(edge->src->coord, edge->dest->coord,
                                               sf::Color(0, 0, 255), default_thickness - 0.5);
                }
            }
        }

        set_final_path(parent);
    }

    void a_star(Graph& graph) {
        std::unordered_map<Node*, Node*> parent;
        // TODO: Add your code here

        set_final_path(parent);
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el
    // 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(10));
        // TODO: Add your code here
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
        path.clear();

        Node* cur = dest;
        if (cur == nullptr)
            return;

        while (cur != src) {
            Node* const par = parent[cur];

            if (par == nullptr)
                break;

            path.emplace_back(cur->coord, par->coord, sf::Color(0, 255, 0), default_thickness + 2);
            cur = par;
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

        switch (algorithm) {
            case Dijkstra: {
                dijkstra(graph);
                break;
            }
            case AStar: {
                a_star(graph);
                break;
            }
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
