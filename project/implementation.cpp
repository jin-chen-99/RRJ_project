#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>

struct GridLocation {
    int x, y;
};

/** implement hash function to store GridLocation into an unordered_set **/
namespace std {
    template<>
    struct hash<GridLocation> {
        typedef GridLocation argument_type;
        typedef std::size_t result_type;

        std::size_t operator()(const GridLocation &id) const noexcept {
            return std::hash<int>()(id.x ^ (id.y << 4));
        }
    };

//    void reverse(vector<GridLocation>::iterator iterator, vector<GridLocation>::iterator iterator1) {
//        vector<GridLocation>::iterator temp;
//        temp = iterator;
//        iterator = iterator1;
//        iterator1 = temp;
//    }
}

/**grid graph, each grid is a node, the barrier is the grid node we delete**/
struct SquareGrid {
    static std::array<GridLocation, 4> DIRS;

    int width, height;//the size of the grid map
    std::unordered_set<GridLocation> barriers;

    SquareGrid(int width_, int height_)
            : width(width_), height(height_) {}

    bool in_bounds(GridLocation id) const {
        return 0 <= id.x && id.x < width
               && 0 <= id.y && id.y < height;
    }

    bool passable(GridLocation id) const {
        return barriers.find(id) == barriers.end();
    }//to check whether this is a barrier

    //get all the neighbor grid
    std::vector<GridLocation> neighbors(GridLocation id) const {
        std::vector<GridLocation> results;

        for (GridLocation dir : DIRS) {
            GridLocation next{id.x + dir.x, id.y + dir.y};
            if (in_bounds(next) && passable(next)) {
                results.push_back(next);
            }
        }

        if ((id.x + id.y) % 2 == 0) {
            // aesthetic improvement on square grids
            std::reverse(results.begin(), results.end());
        }

        return results;
    }
};

/**weighted grid map**/
struct GridWithWeights : SquareGrid {
    std::unordered_set<GridLocation> forests;

    GridWithWeights(int w, int h) : SquareGrid(w, h) {}

    //the cost from one grid to another must be define to be 5(or be 1)
    double cost(GridLocation from_node, GridLocation to_node) const {
        return 1;
    }
};

/**the four possible neighbor/ direction: up, down, left, right**/
std::array<GridLocation, 4> SquareGrid::DIRS =
        {GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}, GridLocation{0, 1}};

/** Helpers for GridLocation**/
bool operator==(GridLocation a, GridLocation b) {
    return a.x == b.x && a.y == b.y;
}

bool operator!=(GridLocation a, GridLocation b) {
    return !(a == b);
}

bool operator<(GridLocation a, GridLocation b) {
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream &
operator<<(std::basic_iostream<char>::basic_ostream &out, const GridLocation &loc) {
    out << '(' << loc.x << ',' << loc.y << ')';
    return out;
}

template<class InputIterator, class T>
InputIterator G_find(InputIterator first, InputIterator last, const T &val) {
    while (first != last) {
        if (*first == val) return first;
        ++first;
    }
    return last;
}

/** This outputs a grid. Pass in a distances map if you want to print
 the distances, or pass in a point_to map if you want to print
 arrows that point to the parent location, or pass in a path vector
 if you want to draw the path.**/
template<class Graph>
void draw_grid(const Graph &graph, int field_width,
               std::unordered_map<GridLocation, double> *distances = nullptr,
               std::unordered_map<GridLocation, GridLocation> *point_to = nullptr,
               std::vector<GridLocation> *path = nullptr) {
    for (int y = 0; y != graph.height; ++y) {
        for (int x = 0; x != graph.width; ++x) {
            GridLocation id{x, y};
            std::cout << std::left << std::setw(field_width);
            if (graph.barriers.find(id) != graph.barriers.end()) {
                //output the barrier
                std::cout << std::string(field_width, '#');
            } else if (point_to != nullptr && point_to->count(id)) {
                GridLocation next = (*point_to)[id];
                //output the trail, the arrow point to the direction of the path
                if (next.x == x + 1) { std::cout << "> "; }
                else if (next.x == x - 1) { std::cout << "< "; }
                else if (next.y == y + 1) { std::cout << "v "; }
                else if (next.y == y - 1) { std::cout << "^ "; }
                    //the terminal is shown as *
                else { std::cout << "* "; }
            } else if (distances != nullptr && distances->count(id)) {
                //display the distance in distance map
                std::cout << (*distances)[id];
            } else if (path != nullptr && G_find(path->begin(), path->end(), id) != path->end()) {
                std::cout << '@';
            } else {
                //the unvisited grid
                std::cout << '.';
            }
        }
        std::cout << '\n';
    }
}


/**this print the path of the search result**/
template<typename Location>
std::vector<Location> reconstruct_path(
        Location start, Location goal,
        std::unordered_map<Location, Location> came_from
) {
    std::vector<Location> path;
    Location current = goal;
    int length = 0;
    while (current != start) {
        path.push_back(current);
        current = came_from[current];
        length++;
    }
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    std::cout << "the shortest path is " << length << std::endl;
    return path;
}

/**a helper to sort the element in ascending order**/
template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
            std::greater<PQElement>> elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};


std::unordered_map<int, std::vector<GridLocation>> edges;
//edges in grid map: a set of grids that consists of all grids on the boundary
//	of the barrier
std::vector<std::vector<GridLocation>> tangents;
//tangents in grid map: If a line is simultaneously tangent to two different obstacles,
// the line is called a common tangent of the obstacles
std::vector<std::vector<GridLocation>> paths;
std::vector<std::vector<GridLocation>> new_paths;
int count = 0;


/**this add square barrier to the map**/
void add_rect(SquareGrid &grid, int x1, int y1, int x2, int y2) {
    for (int x = x1; x <= x2; ++x) {
        for (int y = y1; y <= y2; ++y) {
            grid.barriers.insert(GridLocation{x, y});
        }
    }
}

/**this create a weighted grid map, 10X10**/
GridWithWeights make_diagram() {
    GridWithWeights grid(10, 10);
    add_rect(grid, 0, 7, 4, 9);
    add_rect(grid, 2, 2, 2, 2);
    add_rect(grid, 6, 0, 6, 5);
//    add_rect(grid, 10, 16, 14, 26);
//    add_rect(grid, 13, 4, 15, 15);
//    add_rect(grid, 21, 0, 23, 7);
//    add_rect(grid, 23, 5, 26, 7);
//    add_rect(grid, 20, 25, 28, 30);
//    add_rect(grid, 10, 24, 19, 25);


    typedef GridLocation L;
    grid.forests = std::unordered_set<GridLocation>{
            L{3, 4}, L{3, 5}, L{4, 1}, L{4, 2},
            L{4, 3}, L{4, 4}, L{4, 5}, L{4, 6},
            L{4, 7}, L{4, 8}, L{5, 1}, L{5, 2},
            L{5, 3}, L{5, 4}, L{5, 5}, L{5, 6},
            L{5, 7}, L{5, 8}, L{6, 2}, L{6, 3},
            L{6, 4}, L{6, 5}, L{6, 6}, L{6, 7},
            L{7, 3}, L{7, 4}, L{7, 5}
    };
    return grid;
}


/**helper to A* algorithm**/
inline double heuristic(GridLocation a, GridLocation b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/**dijkstra search for the grid map with barriers**/
template<typename Location, typename Graph>
void dijkstra_search
        (Graph graph,
         Location start,
         Location goal,
         std::unordered_map<Location, Location> &came_from,
         std::unordered_map<Location, double> &cost_so_far) {
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Location current = frontier.get();

        if (current == goal) {
            break;
        }

        for (Location next : graph.neighbors(current)) {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end()
                || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                came_from[next] = current;
                frontier.put(next, new_cost);
            }
        }
    }
}

/**A* search on grid map**/
template<typename Location, typename Graph>
void a_star_search
        (Graph graph,
         Location start,
         Location goal,
         std::unordered_map<Location, Location> &came_from,
         std::unordered_map<Location, double> &cost_so_far) {
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Location current = frontier.get();

        if (current == goal) {
            break;
        }

        for (Location next : graph.neighbors(current)) {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end()
                || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
}

