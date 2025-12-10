#ifndef PATH_FINDING_HPP
#define PATH_FINDING_HPP

#pragma once
#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <limits>
#include <iostream>
#include "NumCpp.hpp"

using namespace std;

typedef pair<int, int> Pair;  // (row, col)

struct Cell {
    int parent_i, parent_j, penalty_cost;
    double f, g, h;
};

struct Node {
    double f;
    int row, col;
    bool operator>(const Node& other) const { return f > other.f; }
};

class PathFinding
{
public:
    PathFinding();
    void setGrid(int ROW, int COL, const nc::NdArray<int>& grid);
    bool findPath(Pair src, Pair dest);
    vector<Pair> getPath() const;

private:
    bool isValid(int row, int col) const;
    bool isBlocked(int row, int col) const;
    bool isDestination(int row, int col, Pair dest) const;
    double calculateH(int row, int col, Pair dest) const;
    void tracePath(const nc::NdArray<Cell>& cellDetails, Pair dest);
    bool aStarSearch(Pair src, Pair dest);


    int map_row_, map_col_;

    nc::NdArray<int> grid_;
    vector<Pair> path_;

}; // class PathFinding

#endif
