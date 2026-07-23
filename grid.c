#include <stdio.h>
#include "grid.h"

void init_grid(Grid *g) {
    int i, j;

    for (i = 0; i < GRID_SIZE; i++) {
        for (j = 0; j < GRID_SIZE; j++) {
            g->cells[i][j] = OPEN;
            g->on_path[i][j] = 0;
        }
    }

    // wall of restricted airspace near the launch corner. forces the drone through
    // one of the two gaps below instead of just cutting straight to the waypoint.
    // (this is a diagonal band where row+col is 9, 10 or 11)
    for (i = 0; i < GRID_SIZE; i++) {
        for (j = 0; j < GRID_SIZE; j++) {
            if (i + j == 9 || i + j == 10 || i + j == 11) {
                g->cells[i][j] = NOFLY;
            }
        }
    }

    // gap #1 - goes through some adverse wind and IMC cells but theres a recharge station
    // right after, so this ends up being the cheaper route overall
    g->cells[4][5] = WIND;
    g->cells[4][6] = STORM;
    g->cells[5][5] = STORM;
    g->cells[5][6] = WIND;
    g->cells[6][4] = WIND;
    g->cells[6][5] = OPEN;
    g->cells[7][4] = RECHARGE;

    // gap #2 - a bit more direct but no recharge nearby and more IMC cells.
    // this is the one BFS tends to end up taking since it doesnt care about energy expenditure
    g->cells[8][1] = WIND;
    g->cells[9][1] = STORM;
    g->cells[9][2] = STORM;
    g->cells[10][0] = STORM;
    g->cells[10][1] = WIND;
    g->cells[11][0] = WIND;

    // extra stuff scattered around so the back half of the grid isnt empty
    g->cells[12][12] = NOFLY;
    g->cells[12][13] = NOFLY;
    g->cells[13][12] = NOFLY;
    g->cells[15][10] = WIND;
    g->cells[16][10] = WIND;
    g->cells[15][11] = STORM;
    g->cells[17][15] = STORM;
    g->cells[17][16] = STORM;
    g->cells[14][8] = RECHARGE;
    g->cells[18][17] = RECHARGE;
}

int terrain_cost(Terrain t) {
    switch (t) {
        case OPEN: return 1;
        case NOFLY: return -1; // shouldnt actually get used, restricted airspace is impassable
        case WIND: return 3;
        case STORM: return 6;
        case RECHARGE: return 1;
    }
    return 1;
}

char terrain_symbol(Terrain t) {
    switch (t) {
        case OPEN: return '.';
        case NOFLY: return '#';
        case WIND: return 'W';
        case STORM: return 'S';
        case RECHARGE: return 'R';
    }
    return '?';
}

void print_grid(Grid *g, int start_row, int start_col, int goal_row, int goal_col, int show_path) {
    int i, j;

    for (i = 0; i < GRID_SIZE; i++) {
        for (j = 0; j < GRID_SIZE; j++) {
            char c;

            if (i == start_row && j == start_col) {
                c = 'D';
            } else if (i == goal_row && j == goal_col) {
                c = 'G';
            } else if (show_path && g->on_path[i][j]) {
                c = '*';
            } else {
                c = terrain_symbol(g->cells[i][j]);
            }

            printf("%c ", c);
        }
        printf("\n");
    }
}
