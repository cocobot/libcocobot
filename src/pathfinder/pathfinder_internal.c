#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <cocobot.h>
#include "cocobot/pathfinder_internal.h"

static cocobot_node_s g_target_node;
static cocobot_node_s g_start_node;
static cocobot_point_s g_real_target_point;

//char cocobot_pathfinder_internal_execute_algo(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], cocobot_node_s *current_node, cocobot_list_s *open_list)
//{
//    while((current_node->x != g_target_node.x) || (current_node->y != g_target_node.y))
//    {
//        cocobot_console_send_asynchronous("PATHFINDER","current node x=%d, y=%d", current_node->x, current_node->y);
//        printf("CURRENT current_node x=%d, y=%d\n", current_node->x, current_node->y);
//        //Treat adjacent node
//        for(int i=((current_node->x)-1); i<=((current_node->x)+1); i++)
//        {
//            for(int j=((current_node->y)-1); j<=((current_node->y)+1); j++)
//            {
//                cocobot_console_send_asynchronous("DEBUG","i=%d, j=%d", i, j);
//                printf("i=%d, j=%d\n", i, j);
//                if((i >= 0) && (j >= 0) && (i < (TABLE_LENGTH/GRID_SIZE)) && (j < (TABLE_WIDTH/GRID_SIZE)) && ((i != current_node->x) || (j != current_node->y)))
//                {
//                    cocobot_pathfinder_compute_node(open_list, &table[i][j], current_node);
//                }
//                printf("current_node x=%d, y=%d\n", current_node->x, current_node->y);
//            }
//        }
//        //open_list is not null
//        if(open_list->nb_elements != 0)
//        {
//            //get first of the list
//            open_list->table[0].nodeType = CLOSED_LIST;
//            current_node = &(open_list->table[0]);
//            cocobot_pathfinder_remove_from_list(open_list, &(open_list->table[0]));
//        }
//        else
//        {
//            cocobot_console_send_asynchronous("PATHFINDER", "No solution");
//            return NO_TRAJECTORY_AVAILABLE;
//        }
//    }
//    return TRAJECTORY_AVAILABLE;
//}

void cocobot_pathfinder_compute_node(cocobot_list_s *open_list, cocobot_node_s* node, cocobot_node_s* parent_node)
{
    //printf("Node type = %d\n", node->nodeType);//Do nothing for all other cases
    if(node->nodeType == NEW_NODE)
    {
        node->pX = parent_node->x;
        node->pY = parent_node->y;
        node->nodeType = OPEN_LIST;
        node->cost = (parent_node->cost - cocobot_pathfinder_get_distance(parent_node, &g_target_node)) + cocobot_pathfinder_get_distance(parent_node, node) + cocobot_pathfinder_get_distance(node, &g_target_node);
        cocobot_pathfinder_add_to_list(open_list, node);
    }
    else if(node->nodeType == OPEN_LIST)
    {
        if((parent_node->cost - cocobot_pathfinder_get_distance(parent_node, &g_target_node)) + cocobot_pathfinder_get_distance(parent_node, node) + cocobot_pathfinder_get_distance(node, &g_target_node) < node->cost)
        {
            node->cost = (parent_node->cost - cocobot_pathfinder_get_distance(parent_node, &g_target_node)) + cocobot_pathfinder_get_distance(parent_node, node) + cocobot_pathfinder_get_distance(node, &g_target_node);
            node->pX = parent_node->x;
            node->pY = parent_node->y;
            cocobot_pathfinder_remove_from_list(open_list, node);
            cocobot_pathfinder_add_to_list(open_list, node);
        }
    }
    else
    {
        ;//Do nothing for all other cases
    }
}

float cocobot_pathfinder_get_distance(cocobot_node_s *source, cocobot_node_s *dest)
{
    // Distance between a node and an other one located just next to it in the diagonal
    static float distance_diag = sqrtf(2);

    float _return_value = 0.0;

    //Same node, in case of a dummy wants a distance between the same node
    if((source->x == dest->x) && (source->y == dest->y))
        _return_value = 0.0;
    else if(source->x == dest->x)
        _return_value = (float)abs(source->y - dest->y);
    else if(source->y == dest->y)
        _return_value = (float)abs(source->x - dest->x);
    else if(abs(source->y - dest->y) == abs(source->x - dest->x))
        _return_value = (float)abs(source->x - dest->x) * distance_diag;
    else
        _return_value = sqrtf((dest->y - source->y)*(dest->y - source->y) + (dest->x - source->x)*(dest->x - source->x));

    return _return_value;
}

void cocobot_pathfinder_initialize_list(cocobot_list_s *list)
{
    list->nb_elements = 0;
    //in theory, not usefull 
    memset(list, 0, sizeof(cocobot_list_s));
}

void cocobot_pathfinder_add_to_list(cocobot_list_s *list, cocobot_node_s *node)
{
    //printf("cout: %d\n", node->cost);
    //If the list is empty
    if(list->nb_elements == 0)
    {
        memcpy(&list->table[0], node, sizeof(cocobot_node_s));
    }
    else
    {
        int index = 0;
        //find its position in the list
        while(node->cost > list->table[index].cost)
        {
            index++;
            if(index == list->nb_elements)
                break;
        }
        //if the list is not full
        if(index < MAXIMUM_NODE_IN_LIST)
        {
            memmove(&list->table[index+1], &list->table[index], (list->nb_elements - index) * sizeof(cocobot_node_s));
            memmove(&list->table[index], node, sizeof(cocobot_node_s));
        }
        else
        {
            ;
            //TBD
            //Cost is to big, not added in list --> not supposed to happen, take a list size big enough
        }
    }
    list->nb_elements++;
}

int cocobot_pathfinder_remove_from_list(cocobot_list_s *list, cocobot_node_s *node)
{
    if(list->nb_elements == 0)
    {
        //printf("List is empty");
        return -1;
    }
    else
    {
        int index = 0;
        while((list->table[index].x != node->x) && (list->table[index].y != node->y))
        {
            index++;
            if(index == list->nb_elements)
                return -2;
        }
        if(index != (MAXIMUM_NODE_IN_LIST - 1))
            memmove(&list->table[index], &list->table[index+1], (list->nb_elements - index - 1) * sizeof(cocobot_node_s));
        else
        {
            ; //Nothing to do
        }
        list->nb_elements--;
    }
    return 0;
}

void cocobot_pathfinder_set_target_node(cocobot_node_s *target_node)
{
    memcpy(&g_target_node, target_node, sizeof(cocobot_node_s));
    cocobot_console_send_asynchronous("TARGET_NODE","x= %d, y= %d", g_target_node.x, g_target_node.y);
}

void cocobot_pathfinder_save_real_target_node(int16_t x, int16_t y)
{
    g_real_target_point.x = x;
    g_real_target_point.y = y;
}

void cocobot_pathfinder_set_start_node(cocobot_node_s *start_node)
{
    memcpy(&g_start_node, start_node, sizeof(cocobot_node_s));
    cocobot_console_send_asynchronous("STARTING_NODE","x= %d, y= %d", g_start_node.x, g_start_node.y);
}

void cocobot_pathfinder_get_path(cocobot_node_s *final_node, cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], cocobot_trajectory_s* trajectory)
{
    //cocobot_console_send_asynchronous("PATH","x= %d, y= %d", final_node->x, final_node->y);
    while((final_node->x !=  g_start_node.x) || (final_node->y != g_start_node.y))
    {
        cocobot_console_send_asynchronous("PATH","x= %d, y= %d", final_node->x, final_node->y);
        //fill trajectory beginning by the end
        trajectory->trajectory[TRAJECTORY_NBR_POINTS_MAX - 1 - trajectory->nbr_points] = cocobot_pathfinder_get_point_from_node(final_node);
        trajectory->nbr_points++;
        final_node = &table[final_node->pX][final_node->pY];
    }
    cocobot_console_send_asynchronous("PATH","x= %d, y= %d", final_node->x, final_node->y);
    //last point
    trajectory->trajectory[TRAJECTORY_NBR_POINTS_MAX - 1 - trajectory->nbr_points] = cocobot_pathfinder_get_point_from_node(final_node);
    trajectory->nbr_points++;
    
    //put the trajectory at the begining of the array using memcpy if possible, memmove otherwise
    if(trajectory->nbr_points <= (TRAJECTORY_NBR_POINTS_MAX / 2))
        memcpy(trajectory->trajectory, &trajectory->trajectory[TRAJECTORY_NBR_POINTS_MAX - trajectory->nbr_points], trajectory->nbr_points * sizeof(cocobot_point_s));
    else
        memmove(trajectory->trajectory, &trajectory->trajectory[TRAJECTORY_NBR_POINTS_MAX - trajectory->nbr_points], trajectory->nbr_points * sizeof(cocobot_point_s));
}

void cocobot_pathfinder_set_trajectory(cocobot_trajectory_s *trajectory)
{
    cocobot_trajectory_s final;
    cocobot_pathfinder_init_trajectory(&final);
    final = cocobot_pathfinder_douglas_peucker(trajectory, 1.0);
    cocobot_point_s point;

    for(int i = 1; i < (final.nbr_points - 1); i++)
    {
        point = cocobot_pathfinder_get_real_coordinate(final.trajectory[i]);
        cocobot_trajectory_goto_xy(point.x, point.y, COCOBOT_TRAJECTORY_UNLIMITED_TIME);
        cocobot_console_send_asynchronous("LINEAR_PATH:","x= %d, y=%d", point.x, point.y);
    }
    cocobot_trajectory_goto_xy(g_real_target_point.x, g_real_target_point.y, COCOBOT_TRAJECTORY_UNLIMITED_TIME);
    cocobot_console_send_asynchronous("LINEAR_PATH:","x= %d, y=%d", g_real_target_point.x, g_real_target_point.y);

}

uint16_t cocobot_pathfinder_get_time(cocobot_node_s *final_node, cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE])
{
    uint16_t time = 0;
    while((final_node->x !=  g_start_node.x) || (final_node->y != g_start_node.y))
    {
        //get time between node and parent node 
        time += (uint16_t)(cocobot_pathfinder_get_distance(final_node, &table[final_node->pX][final_node->pY]) * (float)GRID_SIZE) / cocobot_asserv_get_linear_speed();
        final_node = &table[final_node->pX][final_node->pY];
    }

    return time;
} 

cocobot_trajectory_s cocobot_pathfinder_douglas_peucker(cocobot_trajectory_s *trajectory, float threshold)
{
    float d = 0.0;
    float dMax = 0.0;
    uint8_t index = 0;
    cocobot_trajectory_s resultTraj;
    cocobot_pathfinder_init_trajectory(&resultTraj); 
    //get radial distance for all the points
    for(int i = 1; i < (trajectory->nbr_points - 1); i++)
    {
        d = cocobot_pathfinder_get_radial_distance(trajectory->trajectory[0], trajectory->trajectory[trajectory->nbr_points - 1], trajectory->trajectory[i]);
        if (d > dMax)
        {
            dMax = d;
            index = i;
        }
    }
    
    //if the farest point is above the threshold, the polyline is cut and the algorythm is performed in the 2 polylines created
    if(dMax >= threshold)
    {
        cocobot_trajectory_s traj2;
        cocobot_pathfinder_init_trajectory(&traj2);
        cocobot_pathfinder_cut_trajectory(trajectory, &resultTraj, &traj2, index);

        resultTraj = cocobot_pathfinder_douglas_peucker(&resultTraj, threshold);
        traj2 = cocobot_pathfinder_douglas_peucker(&traj2, threshold);

        cocobot_pathfinder_concatenate_traj(&resultTraj, &traj2);
    }
    //All the point between the bounds are removed from the traj
    else
    {
        resultTraj.nbr_points = 2;
        resultTraj.trajectory[0] = trajectory->trajectory[0];
        resultTraj.trajectory[1] = trajectory->trajectory[trajectory->nbr_points - 1];
    }
    
    return resultTraj;
}

float cocobot_pathfinder_get_radial_distance(cocobot_point_s start, cocobot_point_s end, cocobot_point_s point)
{
    return fabsf(((float)(end.y - start.y)/(float)(end.x - start.x)) * (float)(point.x - start.x) - (float)point.y + (float)start.y)/sqrtf(1.0+(((float)(end.y - start.y)/(float)(end.x - start.x)) * ((float)(end.y - start.y)/(float)(end.x - start.x))));
}

void cocobot_pathfinder_concatenate_traj(cocobot_trajectory_s *first, cocobot_trajectory_s *second)
{
    if((first->nbr_points + second->nbr_points) <= TRAJECTORY_NBR_POINTS_MAX)
    {
        //Check if the last point of first is the same as the first of second --> avoid to have the same point twice 
        if((first->trajectory[first->nbr_points - 1].x == second->trajectory[0].x) && (first->trajectory[first->nbr_points - 1].y == second->trajectory[0].y))
        {
            memcpy(&first->trajectory[first->nbr_points - 1], second->trajectory, second->nbr_points * sizeof(cocobot_point_s));
            first->nbr_points += (second->nbr_points - 1);
        }
        else
        {
            memcpy(&first->trajectory[first->nbr_points], second->trajectory, second->nbr_points * sizeof(cocobot_point_s));
            first->nbr_points += second->nbr_points;
        }
    }
    else
        ;//TODO: Set an error code --> Assuming I know what I do for now
}

void cocobot_pathfinder_cut_trajectory(cocobot_trajectory_s *base, cocobot_trajectory_s *first, cocobot_trajectory_s *second, uint8_t cut_index)
{
    if((cut_index + 1) <= base->nbr_points)
    {
        first->nbr_points = cut_index + 1;
        memcpy(first->trajectory, base->trajectory, first->nbr_points * sizeof(cocobot_point_s));
        second->nbr_points = base->nbr_points - cut_index;
        memcpy(second->trajectory, &base->trajectory[cut_index], second->nbr_points * sizeof(cocobot_point_s));
    }
    else
        ;//TODO: Set an error code --> Assuming I know what I do for now
}

cocobot_point_s cocobot_pathfinder_get_point_from_node(cocobot_node_s *node)
{
    cocobot_point_s _point;
    _point.x = node->x;
    _point.y = node->y;
    return _point;
}

cocobot_point_s cocobot_pathfinder_get_real_coordinate(cocobot_point_s point)
{
    cocobot_point_s _point;
    _point.x = point.x * GRID_SIZE + GRID_SIZE/2 - TABLE_LENGTH/2;
    _point.y = point.y * GRID_SIZE + GRID_SIZE/2 - TABLE_WIDTH/2;
    return _point;
}

void cocobot_pathfinder_init_trajectory(cocobot_trajectory_s *trajectory)
{
    trajectory->nbr_points = 0;
    memset(trajectory->trajectory, 0, TRAJECTORY_NBR_POINTS_MAX*sizeof(cocobot_point_s));
}
