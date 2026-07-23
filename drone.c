#include <string.h>
#include "drone.h"
#include "grid.h"

void init_drone(Drone *d, const char *name, int start_row, int start_col, int max_speed) {
    strncpy(d->name, name, sizeof(d->name) - 1);
    d->name[sizeof(d->name) - 1] = '\0';
    d->row = start_row;
    d->col = start_col;
    d->battery = START_BATTERY;
    d->max_speed = max_speed;
}

int get_move_cost(Terrain t, int drow, int dcol) {
    int base = terrain_cost(t);
    int extra = 0;

    // wind only matters if youre actually in an adverse wind/IMC cell, otherwise
    // just use the normal terrain energy expenditure
    if (t == WIND || t == STORM) {
        if (dcol == -1) {
            extra = 3; // flying west, straight into the headwind
        } else if (dcol == 1) {
            extra = 0; // flying east, wind pushes us along so no penalty
        } else {
            extra = 1; // moving up/down is crosswind, small extra energy expenditure
        }
    }

    return base + extra;
}
