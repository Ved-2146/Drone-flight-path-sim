#ifndef GRID_H
#define GRID_H

// size of the grid. this lives here instead of main.c because grid.c
// needs it at compile time too (arrays need a fixed size). if you change
// this the hardcoded map in init_grid() probably stops making sense
#define GRID_SIZE 20

typedef enum {
    OPEN,
    NOFLY,
    WIND,
    STORM,
    RECHARGE
} Terrain;

typedef struct {
    Terrain cells[GRID_SIZE][GRID_SIZE];
    int on_path[GRID_SIZE][GRID_SIZE]; // gets marked up after pathfinding runs
} Grid;

void init_grid(Grid *g);
int terrain_cost(Terrain t);
char terrain_symbol(Terrain t);
void print_grid(Grid *g, int start_row, int start_col, int goal_row, int goal_col, int show_path);

#endif
