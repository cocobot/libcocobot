#include <cocobot.h>
#include <stdio.h>
#include "cocobot/pathfinder_internal.h"
#include "cocobot/pathfinder_table.h"

static cocobot_node_s g_table[TABLE_LENGTH/GRID_SIZE][TABLE_WIDTH/GRID_SIZE];
static cocobot_list_s open_list;

char cocobot_pathfinder_get_trajectory(cocobot_point_s starting_point, cocobot_point_s target_point, cocobot_trajectory_s *trajectory)
{
    char _return_value = TRAJECTORY_AVAILABLE;

    cocobot_console_send_asynchronous("PATHFINDER", "Started");
    
    cocobot_pathfinder_reset_table(g_table);
    //cocobot_pathfinder_init();
    cocobot_pathfinder_initialize_list(&open_list);
    cocobot_pathfinder_init_trajectory(trajectory);
   
    //start_node
    cocobot_node_s* start_node = &g_table[starting_point.x/GRID_SIZE][starting_point.y/GRID_SIZE];
    cocobot_pathfinder_set_start_node(start_node);

    //target_node
    cocobot_node_s* target_node = &g_table[target_point.x/GRID_SIZE][target_point.y/GRID_SIZE];
    cocobot_pathfinder_set_target_node(target_node);

    cocobot_node_s current_node = *start_node;
    current_node.cost = cocobot_pathfinder_get_distance(&current_node, target_node);

    while((current_node.x != target_node->x) || (current_node.y != target_node->y))
    {
        //cocobot_console_send_asynchronous("PATHFINDER","current node x=%d, y=%d", p_currentNode->x, p_currentNode->y);
        //Treat adjacent node
        for(int i=current_node.x-1; i<=current_node.x+1; i++)
        {
            for(int j=current_node.y-1; j<=current_node.y+1; j++)
            {
                //cocobot_console_send_asynchronous("DEBUG","i=%d, j=%d\n", i, j);
                if((i>=0) && (j>=0) && (i<(TABLE_LENGTH/GRID_SIZE)) && (j<(TABLE_WIDTH/GRID_SIZE)) && ((i != current_node.x) || (j!=current_node.y)))
                {
                    cocobot_pathfinder_compute_node(&open_list, &g_table[i][j], &current_node);
                }
            }
        }
        //open_list is not null
        if(open_list.nb_elements != 0)
        {
            //get first of the list
            open_list.table[0].nodeType = CLOSED_LIST;
            current_node = open_list.table[0];
            cocobot_pathfinder_remove_from_list(&open_list, &open_list.table[0]);
        }
        else
        {
            cocobot_console_send_asynchronous("PATHFINDER", "No solution");
            _return_value = NO_TRAJECTORY_AVAILABLE;
            break;
        }
    }
    
    cocobot_pathfinder_get_path(&current_node, g_table, trajectory);
    return _return_value;
}

void cocobot_pathfinder_init()
{
    cocobot_pathfinder_initialize_table(g_table);
}
