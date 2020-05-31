/**A comparasion of dijkstra, A* and RRJ algorithm**/
#include "implementation.cpp"
#include <iostream>
#include <algorithm>
int main() {

    /**dijkstra search on grid map**/
    std::cout << "-------------------------The output of dijkstra algorithm-------------------------" << std::endl;
    GridWithWeights grid = make_diagram();//create unweighted grid map with barrier
    GridLocation start{0, 0};//set start point
    GridLocation goal{9, 9};//set goal point
    std::unordered_map<GridLocation, GridLocation> came_from;//record the trail
    std::unordered_map<GridLocation, double> cost_so_far;//record the cost
    dijkstra_search(grid, start, goal, came_from, cost_so_far);//dijkstra search
    std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);//record the final path
    draw_grid(grid, 3, nullptr, nullptr, &path);//print the map with path

    /**a star search on grid search**/
    std::cout << "-------------------------The output of A* algorithm-------------------------" << std::endl;
    GridWithWeights grid2 = make_diagram();
    GridLocation start2{0, 0};//set start point
    GridLocation goal2{9, 9};//set goal point
    std::unordered_map<GridLocation, GridLocation> came_from2;
    std::unordered_map<GridLocation, double> cost_so_far2;
    a_star_search(grid2, start2, goal2, came_from2, cost_so_far2);
    std::vector<GridLocation> path2 = reconstruct_path(start2, goal2, came_from2);
    draw_grid(grid2, 3, nullptr, nullptr, &path2);


}

