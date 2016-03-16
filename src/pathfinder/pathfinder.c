#include <cocobot.h>

static cocobot_node_s g_table[TABLE_LENGTH/GRID_SIZE][TABLE_WIDTH/GRID_SIZE];
static cocobot_node_s* g_head_openList;
static cocobot_node_s* g_startNode;
static cocobot_node_s* g_targetNode;

cocobot_trajectory_s cocobot_pathfinder_get_trajectory(cocobot_point_s starting_point, cocobot_point_s target_point)
{
    cocobot_trajectory_s toto;
    return toto;
}

