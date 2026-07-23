#include "pathfinding.h"
#include "drone.h"

static const int dr[4] = {-1, 1, 0, 0};
static const int dc[4] = {0, 0, -1, 1};

static void q_push(Queue *q, int row, int col) {
    q->row[q->back] = row;
    q->col[q->back] = col;
    q->back++;
}

static int q_empty(Queue *q) {
    return q->front >= q->back;
}

static void pq_push(PQueue *pq, int row, int col, int priority) {
    pq->row[pq->size] = row;
    pq->col[pq->size] = col;
    pq->priority[pq->size] = priority;
    pq->size++;
}

// finds the lowest priority entry and yanks it out. swaps the last item
// into the gap instead of shifting the whole array down
static int pq_pop_min(PQueue *pq, int *out_row, int *out_col) {
    int i, min_idx;

    if (pq->size == 0) return 0;

    min_idx = 0;
    for (i = 1; i < pq->size; i++) {
        if (pq->priority[i] < pq->priority[min_idx]) {
            min_idx = i;
        }
    }

    *out_row = pq->row[min_idx];
    *out_col = pq->col[min_idx];

    pq->size--;
    pq->row[min_idx] = pq->row[pq->size];
    pq->col[min_idx] = pq->col[pq->size];
    pq->priority[min_idx] = pq->priority[pq->size];

    return 1;
}

// walks backwards from the waypoint using the from_ arrays then flips the
// list around so out_path goes launch point -> waypoint. copy pasted between
// both algorithms since they reconstruct the path the same way
static void build_path(int from_row[GRID_SIZE][GRID_SIZE], int from_col[GRID_SIZE][GRID_SIZE],
                        int start_row, int start_col, int goal_row, int goal_col, Path *out_path) {
    int tmp_row[MAX_CELLS], tmp_col[MAX_CELLS];
    int len = 0;
    int cr = goal_row, cc = goal_col;
    int i;

    while (!(cr == start_row && cc == start_col)) {
        tmp_row[len] = cr;
        tmp_col[len] = cc;
        len++;
        int pr = from_row[cr][cc];
        int pc = from_col[cr][cc];
        cr = pr;
        cc = pc;
    }
    tmp_row[len] = start_row;
    tmp_col[len] = start_col;
    len++;

    for (i = 0; i < len; i++) {
        out_path->row[i] = tmp_row[len - 1 - i];
        out_path->col[i] = tmp_col[len - 1 - i];
    }
    out_path->len = len;
}

// BFS doesn't care about terrain energy expenditure at all, it just finds the path with
// the fewest steps. so adverse wind/IMC cells are treated exactly the same as
// VFR corridors here, only restricted airspace actually blocks anything
int run_bfs(Grid *g, int start_row, int start_col, int goal_row, int goal_col, Path *out_path, int *nodes_explored) {
    int visited[GRID_SIZE][GRID_SIZE];
    int from_row[GRID_SIZE][GRID_SIZE];
    int from_col[GRID_SIZE][GRID_SIZE];
    int i, j, k, found = 0;
    Queue q;

    for (i = 0; i < GRID_SIZE; i++)
        for (j = 0; j < GRID_SIZE; j++)
            visited[i][j] = 0;

    q.front = 0;
    q.back = 0;
    q_push(&q, start_row, start_col);
    visited[start_row][start_col] = 1;
    *nodes_explored = 0;

    while (!q_empty(&q)) {
        int cur_row = q.row[q.front];
        int cur_col = q.col[q.front];
        q.front++;
        (*nodes_explored)++;

        if (cur_row == goal_row && cur_col == goal_col) {
            found = 1;
            break;
        }

        for (k = 0; k < 4; k++) {
            int nr = cur_row + dr[k];
            int nc = cur_col + dc[k];

            if (nr < 0 || nr >= GRID_SIZE || nc < 0 || nc >= GRID_SIZE) continue;
            if (g->cells[nr][nc] == NOFLY) continue;
            if (visited[nr][nc]) continue;

            visited[nr][nc] = 1;
            from_row[nr][nc] = cur_row;
            from_col[nr][nc] = cur_col;
            q_push(&q, nr, nc);
        }
    }

    if (!found) return 0;

    build_path(from_row, from_col, start_row, start_col, goal_row, goal_col, out_path);
    return 1;
}

int run_dijkstra(Grid *g, int start_row, int start_col, int goal_row, int goal_col, Path *out_path, int *nodes_explored) {
    int dist[GRID_SIZE][GRID_SIZE];
    int visited[GRID_SIZE][GRID_SIZE];
    int from_row[GRID_SIZE][GRID_SIZE];
    int from_col[GRID_SIZE][GRID_SIZE];
    int i, j, k, found = 0;
    PQueue pq;

    for (i = 0; i < GRID_SIZE; i++) {
        for (j = 0; j < GRID_SIZE; j++) {
            dist[i][j] = 999999; // pretending this is infinity
            visited[i][j] = 0;
        }
    }

    dist[start_row][start_col] = 0;
    pq.size = 0;
    pq_push(&pq, start_row, start_col, 0);
    *nodes_explored = 0;

    while (pq.size > 0) {
        int cur_row, cur_col;
        pq_pop_min(&pq, &cur_row, &cur_col);

        // might be an old stale copy of a node we already relaxed with a
        // better distance, if so just skip it instead of redoing the work
        if (visited[cur_row][cur_col]) continue;
        visited[cur_row][cur_col] = 1;
        (*nodes_explored)++;

        if (cur_row == goal_row && cur_col == goal_col) {
            found = 1;
            break;
        }

        for (k = 0; k < 4; k++) {
            int nr = cur_row + dr[k];
            int nc = cur_col + dc[k];

            if (nr < 0 || nr >= GRID_SIZE || nc < 0 || nc >= GRID_SIZE) continue;
            if (g->cells[nr][nc] == NOFLY) continue;
            if (visited[nr][nc]) continue;

            int move_cost = get_move_cost(g->cells[nr][nc], dr[k], dc[k]);
            int new_dist = dist[cur_row][cur_col] + move_cost;

            if (new_dist < dist[nr][nc]) {
                dist[nr][nc] = new_dist;
                from_row[nr][nc] = cur_row;
                from_col[nr][nc] = cur_col;
                pq_push(&pq, nr, nc, new_dist);
            }
        }
    }

    if (!found) return 0;

    build_path(from_row, from_col, start_row, start_col, goal_row, goal_col, out_path);
    return 1;
}
