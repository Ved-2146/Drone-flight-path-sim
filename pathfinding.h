#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "grid.h"

#define MAX_CELLS (GRID_SIZE * GRID_SIZE)

// priority queue can end up holding a few stale duplicate entries since we
// dont bother removing old ones when we relax a node again, so give it
// some extra room
#define PQ_CAPACITY (MAX_CELLS * 4)

typedef struct {
    int row[MAX_CELLS];
    int col[MAX_CELLS];
    int len;
} Path;

// plain fifo queue, used by bfs
typedef struct {
    int row[MAX_CELLS];
    int col[MAX_CELLS];
    int front, back;
} Queue;

// priority queue for dijkstra. had to look up how these usually work,
// doing it the lazy way with an unsorted array + linear scan for the min
// instead of an actual heap. grid is only 400 cells so it doesnt matter
typedef struct {
    int row[PQ_CAPACITY];
    int col[PQ_CAPACITY];
    int priority[PQ_CAPACITY];
    int size;
} PQueue;

int run_bfs(Grid *g, int start_row, int start_col, int goal_row, int goal_col, Path *out_path, int *nodes_explored);
int run_dijkstra(Grid *g, int start_row, int start_col, int goal_row, int goal_col, Path *out_path, int *nodes_explored);

#endif
