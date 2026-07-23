#include <stdio.h>
#include <stdlib.h>
#include "grid.h"
#include "drone.h"
#include "pathfinding.h"

// terrain_symbol() in grid.c just gives the single char, not very
// readable for the telemetry log so spelling it out here
static const char *terrain_name(Terrain t) {
    switch (t) {
        case OPEN: return "VFR corridor";
        case NOFLY: return "restricted airspace";
        case WIND: return "adverse wind corridor";
        case STORM: return "IMC region";
        case RECHARGE: return "recharge station";
    }
    return "unknown";
}

int main() {
    Grid grid;
    Drone drone;
    Path path;
    int nodes_explored = 0;
    int found = 0;
    int i;
    int start_row, start_col, goal_row, goal_col;

    init_grid(&grid);

    // ask the user where to fly from/to instead of just hardcoding it
    printf("enter launch point (row col): ");
    scanf("%d %d", &start_row, &start_col);

    printf("enter waypoint (row col): ");
    scanf("%d %d", &goal_row, &goal_col);

    if (start_row < 0 || start_row > 19 || start_col < 0 || start_col > 19 ||
        goal_row < 0 || goal_row > 19 || goal_col < 0 || goal_col > 19) {
        printf("coords have to be between 0 and 19, try again\n");
        return 0;
    }

    if (grid.cells[start_row][start_col] == NOFLY || grid.cells[goal_row][goal_col] == NOFLY) {
        printf("cant launch from or fly into restricted airspace\n");
        return 0;
    }

    init_drone(&drone, "UAV", start_row, start_col, 55);

    // manhattan dist between launch point and waypoint, used in the mission
    // brief and again later for the efficiency ratio
    int straight_line_dist = abs(goal_row - start_row) + abs(goal_col - start_col);

    printf("=========================================\n");
    printf(" DRONE FLIGHT SIM\n");
    printf("=========================================\n");
    printf("drone name : %s\n", drone.name);
    printf("energy reserve : %d\n", drone.battery);
    printf("max speed  : %d km/h\n", drone.max_speed);

    printf("\n");

    printf("--- MISSION BRIEF ---\n");
    printf("launch point      : (%d, %d)\n", start_row, start_col);
    printf("waypoint          : (%d, %d)\n", goal_row, goal_col);
    printf("straight-line dist: %d cells\n", straight_line_dist);
    printf("energy budget     : 100 Wh\n");
    printf("\n");

    printf("airspace map - pre-mission:\n");
    printf("legend: . = VFR corridor (unrestricted, standard energy expenditure)\n");
    printf("        # = restricted airspace - NFZ (impassable)\n");
    printf("        W = adverse wind corridor (elevated energy expenditure - headwind resistance)\n");
    printf("        S = IMC region (severe energy expenditure - instrument conditions)\n");
    printf("        R = recharge station (energy replenishment point)\n");
    printf("        D = launch point\n");
    printf("        G = waypoint (destination)\n");
    print_grid(&grid, start_row, start_col, goal_row, goal_col, 0);
    printf("\n");

    found = run_dijkstra(&grid, start_row, start_col, goal_row, goal_col, &path, &nodes_explored);

    if (!found) {
        // this shouldnt actually happen with the hardcoded map but just in case
        printf("no path exists from launch point to waypoint. drone stuck on the ground :(\n");
        return 0;
    }

    // mark the path cells so print_grid can draw them with * later
    for (i = 0; i < path.len; i++) {
        grid.on_path[path.row[i]][path.col[i]] = 1;
    }

    printf("telemetry log:\n");

    int total_cost = 0;
    int failed = 0;
    int imc_count = 0; // counts how many steps landed on an S / IMC cell
    int step;

    for (step = 1; step < path.len; step++) {
        int pr = path.row[step - 1];
        int pc = path.col[step - 1];
        int cr = path.row[step];
        int cc = path.col[step];
        int drow = cr - pr;
        int dcol = cc - pc;
        Terrain t = grid.cells[cr][cc];
        int cost = get_move_cost(t, drow, dcol);

        if (t == STORM) imc_count++;

        drone.battery -= cost;
        total_cost += cost;

        // energy reserve ran out before we actually made it to the waypoint cell
        if (drone.battery <= 0 && !(cr == goal_row && cc == goal_col)) {
            drone.battery = 0;
            failed = 1;
            printf("[T+%02d] pos (%d,%d) | terrain: %s | MISSION ABORT - energy depleted before waypoint\n",
                step, cr, cc, terrain_name(t));
            break;
        }

        if (t == RECHARGE) {
            drone.battery = 100; // free refill
        }

        printf("[T+%02d] pos (%d,%d) | terrain: %s | energy delta: -%d Wh | reserve: %d Wh | cumulative: %d Wh\n",
            step, cr, cc, terrain_name(t), cost, drone.battery, total_cost);

        drone.row = cr;
        drone.col = cc;
    }

    printf("\n");
    printf("airspace map - flight path overlaid:\n");
    print_grid(&grid, start_row, start_col, goal_row, goal_col, 1);
    printf("\n");

    // efficiency ratio - straight_line_dist / total_steps. 1.0 would mean the
    // path was literally a straight line, lower means more of a detour
    int total_steps = path.len - 1;
    double efficiency = (double)straight_line_dist / total_steps;
    const char *efficiency_note;
    if (efficiency > 0.85) {
        efficiency_note = "near-optimal routing";
    } else if (efficiency >= 0.60) {
        efficiency_note = "acceptable deviation - terrain avoidance";
    } else {
        efficiency_note = "significant deviation - complex airspace";
    }

    printf("========================================\n");
    printf("MISSION REPORT\n");
    printf("========================================\n");
    printf("search space     : %d nodes evaluated\n", nodes_explored);
    printf("path length      : %d cells traversed\n", total_steps);
    printf("energy expended  : %d Wh\n", total_cost);
    printf("energy remaining : %d Wh (%d%% reserve)\n", drone.battery, drone.battery);
    printf("efficiency ratio : %.2f\n", efficiency);
    printf("                   %s\n", efficiency_note);
    printf("IMC exposure     : %d cells\n", imc_count);
    printf("mission status   : %s\n", failed ? "MISSION ABORT - energy depleted before waypoint" : "MISSION COMPLETE - waypoint reached");
    printf("========================================\n");

    return 0;
}
