#include <cocobot.h>
#include <stdio.h>
#include "cocobot/pathfinder_internal.h"
#include "cocobot/pathfinder_table.h"

static cocobot_node_s g_table[TABLE_LENGTH/GRID_SIZE][TABLE_WIDTH/GRID_SIZE];
//static cocobot_node_s* g_head_openList;

char cocobot_pathfinder_get_trajectory(cocobot_point_s starting_point, cocobot_point_s target_point, cocobot_trajectory_s *trajectory)
{
    char _return_value = TRAJECTORY_AVAILABLE;

    cocobot_console_send_asynchronous("DEBUG", "Started");
    
    cocobot_pathfinder_initialise_table(g_table, TABLE_LENGTH/GRID_SIZE, TABLE_WIDTH/GRID_SIZE);
    cocobot_pathfinder_init_trajectory(trajectory);
    //Open list
    list_s* p_openList = NULL;
   
    //start_node
    cocobot_node_s* start_node = &g_table[starting_point.x/GRID_SIZE][starting_point.y/GRID_SIZE];
    cocobot_pathfinder_set_start_node(start_node);

    //target_node
    cocobot_node_s* target_node = &g_table[target_point.x/GRID_SIZE][target_point.y/GRID_SIZE];
    cocobot_pathfinder_set_target_node(target_node);

    cocobot_node_s* p_currentNode = start_node;
    p_currentNode->cost = cocobot_pathfinder_get_distance(p_currentNode, target_node);

    while(p_currentNode != target_node)
    {
        //cocobot_console_send_asynchronous("DEBUG","current node x=%d, y=%d \n", p_currentNode->x, p_currentNode->y);
        //Treat adjacent node
        for(int i=p_currentNode->x-1; i<=p_currentNode->x+1; i++)
        {
            for(int j=p_currentNode->y-1; j<=p_currentNode->y+1; j++)
            {
                //cocobot_console_send_asynchronous("DEBUG","i=%d, j=%d\n", i, j);
                if((i>=0) && (j>=0) && (i<(TABLE_LENGTH/GRID_SIZE)) && (j<(TABLE_WIDTH/GRID_SIZE)) && ((i != p_currentNode->x) || (j!=p_currentNode->y)))
                {
                    cocobot_pathfinder_compute_node(&p_openList, &g_table[i][j], p_currentNode);
                }
            }
        }
        if(p_openList->p_node != NULL)
        {
            p_openList->p_node->nodeType = CLOSED_LIST;
            p_currentNode = p_openList->p_node;
            cocobot_pathfinder_remove_from_list(p_openList, p_openList->p_node);
        }
        else
        {
            cocobot_console_send_asynchronous("DEBUG", "No solution");
            _return_value = NO_TRAJECTORY_AVAILABLE;
            break;
        }
    }
    
    cocobot_pathfinder_get_path(p_currentNode, g_table, trajectory);
    return _return_value;
}

