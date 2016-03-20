#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <cocobot.h>
#include "cocobot/pathfinder_internal.h"

//static cocobot_node_s g_start_node;
static cocobot_node_s g_target_node;
static cocobot_node_s g_start_node;

void cocobot_pathfinder_compute_node(list_s** open_list, cocobot_node_s* node, cocobot_node_s* parent_node)
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
    return sqrt((dest->y - source->y)*(dest->y - source->y) + (dest->x - source->x)*(dest->x - source->x));
}

void cocobot_pathfinder_add_to_list(list_s **list, cocobot_node_s *node)
{
    //New element is created
    list_s* p_newElement = malloc(sizeof(list_s));
    p_newElement->p_node = node;
    p_newElement->p_nextElement = NULL;
    
    //If the list is empty
    //printf("cout: %d\n", node->cost);
    if(*list == NULL)
    {
        *list = p_newElement;
    }
    else
    {
        list_s* p_tmp = *list;
        list_s* p_prec = NULL;
        while(node->cost > p_tmp->p_node->cost)
        {
            // printf("cout: %d\n", p_tmp->p_node->cost);
            //getchar();
            p_prec = p_tmp;
            p_tmp = p_tmp->p_nextElement;
            if(p_tmp == NULL)
            {
                break;
            }
        }
        //First element of the list
        if(p_prec == NULL)
        {
            p_newElement->p_nextElement = *list;
            *list = p_newElement;
        }
        else
        {
            p_prec->p_nextElement = p_newElement;
            p_newElement->p_nextElement = p_tmp; 
        }
    }
}

int cocobot_pathfinder_remove_from_list(list_s *list, cocobot_node_s *node)
{
    if(list == NULL)
    {
        //printf("List is empty");
        return -1;
    }
    else
    {
        list_s* p_tmp = list;
        list_s* p_prec = NULL;
        while((p_tmp->p_node->x != node->x) && (p_tmp->p_node->y != node->y))
        {
            if(p_tmp->p_nextElement != NULL)
            {
                p_prec = p_tmp;
                p_tmp = p_tmp->p_nextElement; 
            }
            else
            {
                //printf("Element not in the list\n");
                return -2;
            }
        }
        if(p_prec != NULL)
        {
            p_prec->p_nextElement = p_tmp->p_nextElement;
        }
        else
        {
            list->p_node = p_tmp->p_nextElement->p_node;
            list->p_nextElement = p_tmp->p_nextElement->p_nextElement;
        }
        //free(p_tmp);
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
