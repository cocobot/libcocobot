#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <cocobot.h>
#include "cocobot/pathfinder_internal.h"

//static cocobot_node_s g_start_node;
static cocobot_node_s g_target_node;
static cocobot_node_s g_start_node;

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
}

void cocobot_pathfinder_set_start_node(cocobot_node_s *start_node)
{
    memcpy(&g_start_node, start_node, sizeof(cocobot_node_s));
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

    //TODO : get execution_time
    trajectory->execution_time = 0;
}

cocobot_point_s cocobot_pathfinder_get_point_from_node(cocobot_node_s *node)
{
    cocobot_point_s _point;
    _point.x = node->x * GRID_SIZE + GRID_SIZE/2;
    _point.y = node->y * GRID_SIZE + GRID_SIZE/2;
    return _point;
}

void cocobot_pathfinder_init_trajectory(cocobot_trajectory_s *trajectory)
{
    trajectory->execution_time = 0;
    trajectory->nbr_points = 0;
    memset(trajectory->trajectory, 0, TRAJECTORY_NBR_POINTS_MAX*sizeof(cocobot_point_s));
}
