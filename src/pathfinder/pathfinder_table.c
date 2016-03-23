#include "cocobot/pathfinder_table.h"
#include <string.h>
#include <math.h>

void cocobot_pathfinder_initialise_table(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int length, int width)
{
    memset(table, 0, length * sizeof(cocobot_node_s));
    for(int i = 0; i < length; i++)
    {
        memset(table[i], 0, width * sizeof(cocobot_node_s));
        for(int j = 0; j < width; j++)
        {
            table[i][j].x = i;
            table[i][j].y = j;
        }
    }

    // ARound the dune
    cocobot_pathfinder_set_rectangle(table, 20/GRID_SIZE, 200/GRID_SIZE, 800/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, 20/GRID_SIZE, 200/GRID_SIZE, 2200/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);
    
    //Central T (construction area)
    cocobot_pathfinder_set_rectangle(table, 1200/GRID_SIZE, 20/GRID_SIZE, 900/GRID_SIZE, 750/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, 60/GRID_SIZE, 600/GRID_SIZE, 1470/GRID_SIZE, 750/GRID_SIZE, OBSTACLE);
    
    //iles
    cocobot_pathfinder_set_circle(table, 0/GRID_SIZE, 1990/GRID_SIZE, 250/GRID_SIZE, OBSTACLE); 
    cocobot_pathfinder_set_circle(table, 2990/GRID_SIZE, 1990/GRID_SIZE, 250/GRID_SIZE, OBSTACLE); 
}

void cocobot_pathfinder_set_point(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x, int y, cocobot_nodeType_e node_type)
{
    //Security to avoid buffer overflow
    //TODO : Change Hard coded Value
    if((x < TABLE_LENGTH/GRID_SIZE) && (y < TABLE_WIDTH/GRID_SIZE) && (x >= 0) && (y >= 0))
        table[x][y].nodeType = node_type;
    //printf("x:%d, y:%d\n", _x, _y);
}

void cocobot_pathfinder_set_rectangle(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x_dimension, int y_dimension, int x_position, int y_position, cocobot_nodeType_e node_type)
{
    //A rectangle has obviously a dimension --> if dimensions are null, set them to 1
    if(x_dimension == 0)
        x_dimension = 1;
    if(y_dimension == 0)
        y_dimension = 1;

    for(int x = x_position; x < x_dimension + x_position; x++)
    {
        for(int y = y_position ; y < y_dimension + y_position; y++)
        {
            cocobot_pathfinder_set_point(table, x, y, node_type);
        }   
    }
}

void cocobot_pathfinder_set_circle(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x_center, int y_center, int radius, cocobot_nodeType_e node_type)
{
    //A circle without a radius is nothing --> set radius to 1
    if(radius == 0)
        radius = 1;

    double xMax = 0.0;
    for(int y = y_center; y < y_center + radius; y++)
    {
        xMax = sqrt((float)radius*(float)radius - (float)(y-y_center)*(float)(y-y_center));
        //printf("x= %d, y= %d\n", (int)round(xMax), y);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, y_center, y, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, y_center - (int)round(xMax) , y, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, y_center, 2*y_center - y - 1, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, y_center - (int)round(xMax), 2*y_center - y - 1, node_type);
    }
}
