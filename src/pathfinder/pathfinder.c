#include <cocobot.h>
#include <stdio.h>

static cocobot_node_s g_table[TABLE_LENGTH/GRID_SIZE][TABLE_WIDTH/GRID_SIZE];
static cocobot_node_s* g_head_openList;

cocobot_trajectory_s cocobot_pathfinder_get_trajectory(cocobot_point_s starting_point, cocobot_point_s target_point)
{
    initialiseTable(g_table, TABLE_LENGTH/GRID_SIZE, TABLE_WIDTH/GRID_SIZE);
    cocobot_console_send_asynchronous("DEBUG", "Started");
    cocobot_trajectory_s toto;
    return toto;
}

