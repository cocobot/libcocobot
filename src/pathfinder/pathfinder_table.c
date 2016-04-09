#include "cocobot/pathfinder_table.h"
#include <string.h>
#include <math.h>

void cocobot_pathfinder_initialize_table(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], uint16_t robot_length, uint16_t robot_width)
{
    memset(table, 0, (TABLE_LENGTH / GRID_SIZE) * sizeof(cocobot_node_s));
    for(int i = 0; i < TABLE_LENGTH / GRID_SIZE; i++)
    {
        memset(table[i], 0, (TABLE_WIDTH / GRID_SIZE) * sizeof(cocobot_node_s));
        for(int j = 0; j < TABLE_WIDTH / GRID_SIZE; j++)
        {
            table[i][j].x = i;
            table[i][j].y = j;
        }
    }
    
    //Compute a half diag of the robot to expand all obstacle to the size of half a robot
    uint16_t half_diag = (uint16_t) (sqrtf((robot_length * robot_length) + (robot_width * robot_width)) / 2) + GRID_SIZE;
    uint8_t grid_half_diag = half_diag / GRID_SIZE;

    //Expand walls
    cocobot_pathfinder_set_rectangle(table, TABLE_LENGTH / GRID_SIZE, grid_half_diag, 0, 0, OBSTACLE);  
    cocobot_pathfinder_set_rectangle(table, TABLE_LENGTH / GRID_SIZE, grid_half_diag, 0, TABLE_WIDTH/GRID_SIZE - grid_half_diag, OBSTACLE);  
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, TABLE_WIDTH / GRID_SIZE, 0, 0, OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, TABLE_WIDTH / GRID_SIZE, TABLE_LENGTH / GRID_SIZE - grid_half_diag, 0, OBSTACLE);  

    // Around the dune (expanded)
    cocobot_pathfinder_set_rectangle(table, (20 + half_diag * 2)/GRID_SIZE, 200/GRID_SIZE, (800 - half_diag)/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_circle(table, 810/GRID_SIZE, 200/GRID_SIZE, (half_diag + 10) / GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, (20 + half_diag * 2)/GRID_SIZE, 200/GRID_SIZE, (2200 - half_diag)/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_circle(table, 2210/GRID_SIZE, 200/GRID_SIZE, (half_diag + 10) / GRID_SIZE, OBSTACLE);
    
    //Central T (construction area)
    cocobot_pathfinder_set_rectangle(table, 1200/GRID_SIZE, (20 + half_diag *2)/GRID_SIZE, 900/GRID_SIZE, (750 - half_diag)/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_circle(table, 900/GRID_SIZE, 760/GRID_SIZE, (half_diag + 10) / GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_circle(table, 2100/GRID_SIZE, 760/GRID_SIZE, (half_diag + 10) / GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, (50 + half_diag * 2)/GRID_SIZE, 600/GRID_SIZE, (1470 - half_diag)/GRID_SIZE, 750/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_circle(table, 1500/GRID_SIZE, 1350/GRID_SIZE, (half_diag + 25) / GRID_SIZE, OBSTACLE);
    //TODO expand nose
    
    //iles
    cocobot_pathfinder_set_circle(table, 0/GRID_SIZE, 1990/GRID_SIZE, (250 + half_diag)/GRID_SIZE, OBSTACLE); 
    cocobot_pathfinder_set_circle(table, 2990/GRID_SIZE, 1990/GRID_SIZE, (250 + half_diag)/GRID_SIZE, OBSTACLE); 
}

void cocobot_pathfinder_reset_table(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE])
{
   for(int i = 0; i < TABLE_LENGTH/GRID_SIZE; i++)
   {
       for(int j = 0; j < TABLE_WIDTH/GRID_SIZE; j++)
       {
           table[i][j].nodeType &= MASK_NEW_NODE;
       }
   }
}

void cocobot_pathfinder_set_point(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x, int y, cocobot_nodeType_e node_type)
{
    //Security to avoid buffer overflow
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
        xMax = sqrtf((float)radius*(float)radius - (float)(y-y_center)*(float)(y-y_center));
        //printf("x= %d, y= %d\n", (int)round(xMax), y);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center, y, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center - (int)round(xMax) , y, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center, 2*y_center - y - 1, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center - (int)round(xMax), 2*y_center - y - 1, node_type);
    }
}
