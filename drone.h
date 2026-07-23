#ifndef DRONE_H
#define DRONE_H

#include "grid.h"

#define START_BATTERY 100

typedef struct {
    char name[32];
    int row, col;
    int battery;
    int max_speed;
} Drone;

void init_drone(Drone *d, const char *name, int start_row, int start_col, int max_speed);

// works out how much energy reserve a move into this terrain drains, factoring in
// wind direction. wind blows east (increasing col) globally
int get_move_cost(Terrain t, int drow, int dcol);

#endif
