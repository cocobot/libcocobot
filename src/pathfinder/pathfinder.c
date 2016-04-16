#include <cocobot.h>
#include <stdio.h>
#include "cocobot/pathfinder_internal.h"
#include "cocobot/pathfinder_table.h"

static cocobot_node_s g_table[TABLE_LENGTH/GRID_SIZE][TABLE_WIDTH/GRID_SIZE];
static cocobot_list_s open_list;

uint16_t cocobot_pathfinder_get_trajectory_time(int16_t starting_point_x, int16_t starting_point_y, int16_t target_point_x, int16_t target_point_y)
{
    cocobot_console_send_asynchronous("PATHFINDER", "Get trajectory time started");
    
    cocobot_pathfinder_reset_table(g_table);

    cocobot_pathfinder_initialize_list(&open_list);
   
    //start_node
    cocobot_node_s* start_node = &g_table[(starting_point_x + (TABLE_LENGTH / 2)) / GRID_SIZE][(starting_point_y + (TABLE_WIDTH / 2))/GRID_SIZE];
    cocobot_pathfinder_set_start_node(start_node);

    //target_node
    cocobot_node_s* target_node = &g_table[(target_point_x + (TABLE_LENGTH / 2))/GRID_SIZE][(target_point_y + (TABLE_WIDTH / 2))/GRID_SIZE];
    cocobot_pathfinder_set_target_node(target_node);

    cocobot_node_s current_node = *start_node;
    current_node.cost = cocobot_pathfinder_get_distance(&current_node, target_node);

    while((current_node.x != target_node->x) || (current_node.y != target_node->y))
    {
        //cocobot_console_send_asynchronous("PATHFINDER","current node x=%d, y=%d", current_node.x, current_node.y);
        //Treat adjacent node
        for(int i=current_node.x-1; i<=current_node.x+1; i++)
        {
            for(int j=current_node.y-1; j<=current_node.y+1; j++)
            {
                //cocobot_console_send_asynchronous("DEBUG","i=%d, j=%d", i, j);
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
            return 0;
        }
    }
    
    return cocobot_pathfinder_get_time(&current_node, g_table);
}

char cocobot_pathfinder_execute_trajectory(int16_t starting_point_x, int16_t starting_point_y, int16_t target_point_x, int16_t target_point_y)
{
    cocobot_console_send_asynchronous("PATHFINDER", "execute trajectory started");
    
    cocobot_pathfinder_reset_table(g_table);

    cocobot_pathfinder_initialize_list(&open_list);
   
    //start_node
    cocobot_node_s* start_node = &g_table[(starting_point_x + (TABLE_LENGTH / 2)) / GRID_SIZE][(starting_point_y + (TABLE_WIDTH / 2))/GRID_SIZE];
    cocobot_pathfinder_set_start_node(start_node);

    //target_node
    cocobot_node_s* target_node = &g_table[(target_point_x + (TABLE_LENGTH / 2))/GRID_SIZE][(target_point_y + (TABLE_WIDTH / 2))/GRID_SIZE];
    cocobot_pathfinder_set_target_node(target_node);

    cocobot_node_s current_node = *start_node;
    current_node.cost = cocobot_pathfinder_get_distance(&current_node, target_node);

    cocobot_trajectory_s _trajectory;
    cocobot_pathfinder_init_trajectory(&_trajectory);

    while((current_node.x != target_node->x) || (current_node.y != target_node->y))
    {
        //cocobot_console_send_asynchronous("PATHFINDER","current node x=%d, y=%d", current_node.x, current_node.y);
        //Treat adjacent node
        for(int i=current_node.x-1; i<=current_node.x+1; i++)
        {
            for(int j=current_node.y-1; j<=current_node.y+1; j++)
            {
                //cocobot_console_send_asynchronous("DEBUG","i=%d, j=%d", i, j);
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
            return NO_TRAJECTORY_AVAILABLE;
        }
    }

    cocobot_pathfinder_get_path(&current_node, g_table, &_trajectory);
    for(int i = 0; i < _trajectory.nbr_points; i++)
    {
        cocobot_console_send_asynchronous("REAL_PATH", "x: %d, y:%d", _trajectory.trajectory[i].x, _trajectory.trajectory[i].y);
    }
    
    
    return TRAJECTORY_AVAILABLE;
}

void cocobot_pathfinder_init(uint16_t robot_length, uint16_t robot_width)
{
    cocobot_pathfinder_initialize_table(g_table, robot_length, robot_width);
}
