#include <cocobot.h>
#include <stdio.h>
#include <stdlib.h>
#include "cocobot/pathfinder_internal.h"
#include "cocobot/pathfinder_table.h"

static cocobot_node_s g_table[TABLE_LENGTH/GRID_SIZE][TABLE_WIDTH/GRID_SIZE];
static cocobot_list_s open_list;
static cocobot_trajectory_s final_traj;
static opponent_table_s opponent_robot;

uint16_t cocobot_pathfinder_get_trajectory_time(int16_t starting_point_x, int16_t starting_point_y, int16_t target_point_x, int16_t target_point_y)
{
    cocobot_console_send_asynchronous("PATHFINDER", "Get trajectory time started");
    
    cocobot_pathfinder_reset_table(g_table);

    cocobot_pathfinder_initialize_list(&open_list);
   
    //target_node
    cocobot_node_s* target_node = &g_table[(target_point_x + (TABLE_LENGTH / 2))/GRID_SIZE][(target_point_y - (TABLE_WIDTH / 2))/GRID_SIZE];
    if(((target_node->nodeType & OBSTACLE) == OBSTACLE) || ((target_node->nodeType & SOFT_OBSTACLE) == SOFT_OBSTACLE) || ((target_node->nodeType & FORBIDDEN) == FORBIDDEN))
    {
        cocobot_console_send_asynchronous("PATHFINDER", "Target not reachable");
        return 0;
    }
    cocobot_pathfinder_set_target_node(target_node);

    //start_node
    cocobot_node_s* start_node = &g_table[(starting_point_x + (TABLE_LENGTH / 2)) / GRID_SIZE][(starting_point_y - (TABLE_WIDTH / 2))/GRID_SIZE];
    cocobot_pathfinder_set_start_node(start_node);

    cocobot_node_s current_node = *start_node;
    current_node.cost = cocobot_pathfinder_get_distance(&current_node, target_node);

    while((current_node.x != target_node->x) || (current_node.y != target_node->y))
    {
       // cocobot_console_send_asynchronous("PATHFINDER","current node x=%d, y=%d", current_node.x, current_node.y);
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
            open_list.table[0]->nodeType &= 0xFF0;
            open_list.table[0]->nodeType |= CLOSED_LIST;
            current_node = *open_list.table[0];
            cocobot_pathfinder_remove_from_list(&open_list, open_list.table[0]);
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
   
    //target_node
    cocobot_pathfinder_save_real_target_node(target_point_x, target_point_y);
    cocobot_node_s* target_node = &g_table[(target_point_x + (TABLE_LENGTH / 2))/GRID_SIZE][((TABLE_WIDTH / 2) - target_point_y)/GRID_SIZE];
    if(((target_node->nodeType & OBSTACLE) == OBSTACLE) || ((target_node->nodeType & SOFT_OBSTACLE) == SOFT_OBSTACLE) || ((target_node->nodeType & FORBIDDEN) == FORBIDDEN))
    {
        cocobot_console_send_asynchronous("PATHFINDER", "Target not reachable");
        return DESTINATION_NOT_AVAILABLE;
    }
    cocobot_pathfinder_set_target_node(target_node);

    //start_node
    cocobot_node_s* start_node = &g_table[(starting_point_x + (TABLE_LENGTH / 2)) / GRID_SIZE][((TABLE_WIDTH / 2) - starting_point_y)/GRID_SIZE];
    cocobot_pathfinder_set_start_node(start_node);

    cocobot_node_s current_node = *start_node;
    current_node.cost = cocobot_pathfinder_get_distance(&current_node, target_node);

    cocobot_pathfinder_init_trajectory(&final_traj);

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
            open_list.table[0]->nodeType &= 0xFF0;
            open_list.table[0]->nodeType |= CLOSED_LIST;
            //cocobot_console_send_asynchronous("OPEN_LIST","x=%d, y=%d px=%d, py=%d, status=%x cost :%f", open_list.table[0].x, open_list.table[0].y, open_list.table[0].pX, open_list.table[0].pY, open_list.table[0].nodeType,(double)open_list.table[0].cost);
            current_node = *open_list.table[0];
            cocobot_pathfinder_remove_from_list(&open_list, open_list.table[0]);
        }
        else
        {
            cocobot_console_send_asynchronous("PATHFINDER", "No solution");
            return NO_ROUTE_TO_TARGET;
        }
    }
    cocobot_pathfinder_get_path(&current_node, g_table, &final_traj);
    for(int i = 0; i < final_traj.nbr_points; i++)
    {
        cocobot_console_send_asynchronous("REAL_PATH", "x: %d, y:%d", final_traj.trajectory[i].x, final_traj.trajectory[i].y);
    }
    
    cocobot_pathfinder_set_trajectory(&final_traj);

    if(getenv("DEBUG_TABLE") != NULL)
    {
        if(strcmp(getenv("DEBUG_TABLE"), "1") == 0)
        {
            FILE* f = fopen("debug_table.txt","w+");
            for(int i=0; i<TABLE_LENGTH/GRID_SIZE; i++)
            {
                for(int j=0; j<TABLE_WIDTH/GRID_SIZE; j++)
                {
                    fwrite(&g_table[i][j].nodeType, sizeof(uint16_t), 1, f);
                }
            }
            fclose(f);
        }
    }
    
    return TRAJECTORY_READY;
}

void cocobot_pathfinder_set_robot(int adv_x, int adv_y)
{
    if(opponent_robot.nbr_slot_used < NBR_OTHER_ROBOT_MAX)
    {
        //cocobot_console_send_asynchronous("SET ROBOT", "x:%d, y:%d, slot:%d", (adv_x + (TABLE_LENGTH / 2))/GRID_SIZE, ((TABLE_WIDTH / 2) - adv_y)/GRID_SIZE, opponent_robot.nbr_slot_used + 1);
        opponent_robot.other_robot[opponent_robot.nbr_slot_used].x = adv_x;
        opponent_robot.other_robot[opponent_robot.nbr_slot_used].y = adv_y;
        cocobot_pathfinder_set_robot_zone(g_table, (adv_x + (TABLE_LENGTH / 2))/GRID_SIZE, ((TABLE_WIDTH / 2) - adv_y)/GRID_SIZE);
        opponent_robot.nbr_slot_used++;
    }
    else
    {
        //cocobot_console_send_asynchronous("SET ROBOT", "x:%d, y:%d, slot:%d", (adv_x + (TABLE_LENGTH / 2))/GRID_SIZE, ((TABLE_WIDTH / 2) - adv_y)/GRID_SIZE, opponent_robot.nbr_slot_used);
        cocobot_pathfinder_remove_robot(opponent_robot.other_robot[0].x, opponent_robot.other_robot[0].y);
        opponent_robot.other_robot[2].x = adv_x;
        opponent_robot.other_robot[2].y = adv_y;
        cocobot_pathfinder_set_robot_zone(g_table, (adv_x + (TABLE_LENGTH / 2))/GRID_SIZE, ((TABLE_WIDTH / 2) - adv_y)/GRID_SIZE);
        opponent_robot.nbr_slot_used++;
    }
}

void cocobot_pathfinder_remove_robot(int adv_x, int adv_y)
{
    for(int i = 0; i < opponent_robot.nbr_slot_used; i++)
    {
        if((opponent_robot.other_robot[i].x == adv_x) && (opponent_robot.other_robot[i].y == adv_y))
        {
            //cocobot_console_send_asynchronous("UNSET ROBOT", "x:%d, y:%d, slot:%d", (adv_x + (TABLE_LENGTH / 2))/GRID_SIZE, ((TABLE_WIDTH / 2) - adv_y)/GRID_SIZE, i + 1);
            cocobot_pathfinder_unset_robot_zone(g_table, (adv_x + (TABLE_LENGTH / 2))/GRID_SIZE, ((TABLE_WIDTH / 2) - adv_y)/GRID_SIZE);
            if(i < (opponent_robot.nbr_slot_used - 1))
            {
                memmove(&opponent_robot.other_robot[i], &opponent_robot.other_robot[i+1], (opponent_robot.nbr_slot_used - 1 - i) * sizeof(cocobot_point_s));
            }
            else
            {
                memset(&opponent_robot.other_robot[i], 0, sizeof(cocobot_point_s));
            }
            opponent_robot.nbr_slot_used--;

            //set all remaining robot to avoid wrong unset caused by two opponent robots closed to each other
            for(int j = 0; j < opponent_robot.nbr_slot_used; j++)
            {
                cocobot_pathfinder_set_robot_zone(g_table, (opponent_robot.other_robot[j].x + (TABLE_LENGTH / 2))/GRID_SIZE, ((TABLE_WIDTH / 2) - opponent_robot.other_robot[j].y)/GRID_SIZE);
            }
        }
        else
        {
            //No opponent robot in the zone
        }
    }
}

void cocobot_pathfinder_allow_start_zone()
{
    cocobot_pathfinder_set_start_zone_allowed();
}

void cocobot_pathfinder_init(uint16_t robot_length, uint16_t robot_width)
{
    memset(&opponent_robot, 0, sizeof(opponent_table_s));

    cocobot_pathfinder_initialize_table(g_table, robot_length, robot_width);
}
