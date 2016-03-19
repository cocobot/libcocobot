#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cocobot/pathfinder_internal.h"

//static cocobot_node_s g_start_node;
static cocobot_node_s g_target_node;

void cocobot_pathfinder_compute_node(list_s** open_list, cocobot_node_s* node, cocobot_node_s* parent_node)
{
    //printf("Node type = %d\n", node->nodeType);//Do nothing for all other cases
    if(node->nodeType == NEW_NODE)
    {
        node->pX = parent_node->x;
        node->pY = parent_node->y;
        node->nodeType = OPEN_LIST;
        node->cost = (parent_node->cost - cocobot_get_distance(parent_node, &g_target_node)) + cocobot_get_distance(parent_node, node) + cocobot_get_distance(node, &g_target_node);
        addToList(open_list, node);
    }
    else if(node->nodeType == OPEN_LIST)
    {
        if((parent_node->cost - cocobot_get_distance(parent_node, &g_target_node)) + cocobot_get_distance(parent_node, node) + cocobot_get_distance(node, &g_target_node) < node->cost)
        {
            node->cost = (parent_node->cost - cocobot_get_distance(parent_node, &g_target_node)) + cocobot_get_distance(parent_node, node) + cocobot_get_distance(node, &g_target_node);
            node->pX = parent_node->x;
            node->pY = parent_node->y;
            addToList(open_list, node);
        }
    }
    else
    {
        ;//Do nothing for all other cases
    }
}

float cocobot_get_distance(cocobot_node_s *_p_source, cocobot_node_s *_p_dest)
{
    return sqrt((_p_dest->y - _p_source->y)*(_p_dest->y - _p_source->y) + (_p_dest->x - _p_source->x)*(_p_dest->x - _p_source->x));
}

void addToList(list_s** _p_list, cocobot_node_s* _p_node)
{
    //New element is created
    list_s* p_newElement = malloc(sizeof(list_s));
    p_newElement->p_node = _p_node;
    p_newElement->p_nextElement = NULL;
    
    //If the list is empty
    //printf("cout: %d\n", _p_node->cost);
    if(*_p_list == NULL)
    {
        *_p_list = p_newElement;
    }
    else
    {
        list_s* p_tmp = *_p_list;
        list_s* p_prec = NULL;
        while(_p_node->cost > p_tmp->p_node->cost)
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
            p_newElement->p_nextElement = *_p_list;
            *_p_list = p_newElement;
        }
        else
        {
            p_prec->p_nextElement = p_newElement;
            p_newElement->p_nextElement = p_tmp; 
        }
    }
}

int removeFromList(list_s* _p_list, cocobot_node_s* _p_node)
{
    if(_p_list == NULL)
    {
        //printf("List is empty");
        return -1;
    }
    else
    {
        list_s* p_tmp = _p_list;
        list_s* p_prec = NULL;
        while((p_tmp->p_node->x != _p_node->x) && (p_tmp->p_node->y != _p_node->y))
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
            _p_list->p_node = p_tmp->p_nextElement->p_node;
            _p_list->p_nextElement = p_tmp->p_nextElement->p_nextElement;
        }
        //free(p_tmp);
    }
    return 0;
}

list_s* getPath(cocobot_node_s*_p_finalNode, cocobot_node_s** p_table)
{
    //TRACE_DEBUG("x=%d y=%d\n",_p_finalNode->x, _p_finalNode->y);
    printf("%p, %p", _p_finalNode,p_table);
    list_s* finalTraj = NULL;
    //while((_p_finalNode->x !=  g_startNode->x) || (_p_finalNode->y != g_startNode->y))
    //{
    //    //TRACE_DEBUG("px= %d, py= %d\n", _p_finalNode->pX, _p_finalNode->pY);
    //    _p_finalNode->nodeType = FINAL_TRAJ; 
    //    addToList(&finalTraj, _p_finalNode);
    //    _p_finalNode = &p_table[_p_finalNode->pX][_p_finalNode->pY];

    //}
    //_p_finalNode->nodeType = FINAL_TRAJ; 
    return finalTraj;
}

