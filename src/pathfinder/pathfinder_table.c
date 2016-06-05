#include "cocobot/pathfinder_table.h"
#include "cocobot.h"
#include <string.h>
#include <math.h>

static uint8_t cocobot_start_zone_allowed = 0;

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
            table[i][j].nodeType = NEW_NODE;
        }
    }
    
    uint16_t half_width = robot_width/2 + (GRID_SIZE-1);
    uint8_t grid_half_width = (half_width / GRID_SIZE) + 1;
    uint16_t half_diag = (uint16_t) (sqrtf((robot_length * robot_length) + (robot_width * robot_width)) / 2) + (GRID_SIZE-1);
    uint8_t grid_half_diag = half_diag / GRID_SIZE;

    //walls half diag = soft_obstacle
    cocobot_pathfinder_set_rectangle(table, TABLE_LENGTH / GRID_SIZE, grid_half_diag, 0, 0, SOFT_OBSTACLE);  
    cocobot_pathfinder_set_rectangle(table, TABLE_LENGTH / GRID_SIZE, grid_half_diag, 0, TABLE_WIDTH/GRID_SIZE - grid_half_diag, SOFT_OBSTACLE);  
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, TABLE_WIDTH / GRID_SIZE, 0, 0, SOFT_OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, TABLE_WIDTH / GRID_SIZE, TABLE_LENGTH / GRID_SIZE - grid_half_diag, 0, SOFT_OBSTACLE);  

    // dune soft_obstacle
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, 200/GRID_SIZE, 800/GRID_SIZE - grid_half_diag, 0/GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, 200/GRID_SIZE, 800/GRID_SIZE + 1, 0/GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_circle(table, 810/GRID_SIZE, 200/GRID_SIZE, (half_diag + 10) / GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, 200/GRID_SIZE, 2200/GRID_SIZE - grid_half_diag, 0/GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, 200/GRID_SIZE, 2200/GRID_SIZE + 1, 0/GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_circle(table, 2210/GRID_SIZE, 200/GRID_SIZE, (half_diag + 10) / GRID_SIZE, SOFT_OBSTACLE);
    
    //Central T soft_obstacle
    cocobot_pathfinder_set_rectangle(table, 1200/GRID_SIZE, grid_half_diag, 900/GRID_SIZE, 750/GRID_SIZE - grid_half_diag, SOFT_OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, 1200/GRID_SIZE, grid_half_diag, 900/GRID_SIZE, 750/GRID_SIZE + 1, SOFT_OBSTACLE);
    cocobot_pathfinder_set_circle(table, 900/GRID_SIZE, 760/GRID_SIZE, (half_diag + 10) / GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_circle(table, 2100/GRID_SIZE, 760/GRID_SIZE, (half_diag + 10) / GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, 600/GRID_SIZE, 1470/GRID_SIZE - grid_half_diag, 750/GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, grid_half_diag, 600/GRID_SIZE, 1470/GRID_SIZE + 1, 750/GRID_SIZE, SOFT_OBSTACLE);
    cocobot_pathfinder_set_circle(table, 1500/GRID_SIZE, 1350/GRID_SIZE, (half_diag + 25) / GRID_SIZE, SOFT_OBSTACLE);
    
    //iles soft_obstacle
    cocobot_pathfinder_set_circle(table, 0/GRID_SIZE, 2000/GRID_SIZE, (250 + half_diag)/GRID_SIZE, SOFT_OBSTACLE); 
    cocobot_pathfinder_set_circle(table, 3000/GRID_SIZE, 2000/GRID_SIZE, (250 + half_diag)/GRID_SIZE, SOFT_OBSTACLE); 

    //Expand walls half width = forbidden 
    cocobot_pathfinder_set_rectangle(table, TABLE_LENGTH / GRID_SIZE, grid_half_width, 0, 0, FORBIDDEN);  
    cocobot_pathfinder_set_rectangle(table, TABLE_LENGTH / GRID_SIZE, grid_half_width, 0, TABLE_WIDTH/GRID_SIZE - grid_half_width, FORBIDDEN);  
    cocobot_pathfinder_set_rectangle(table, grid_half_width, TABLE_WIDTH / GRID_SIZE, 0, 0, FORBIDDEN);
    cocobot_pathfinder_set_rectangle(table, grid_half_width, TABLE_WIDTH / GRID_SIZE, TABLE_LENGTH / GRID_SIZE - grid_half_width, 0, FORBIDDEN);  

    // dune forbidden
    cocobot_pathfinder_set_rectangle(table, grid_half_width, 200/GRID_SIZE, 800/GRID_SIZE - grid_half_width, 0/GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_rectangle(table, grid_half_width, 200/GRID_SIZE, 800/GRID_SIZE + 1, 0/GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_circle(table, 810/GRID_SIZE, 200/GRID_SIZE, (half_width + 10) / GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_rectangle(table, grid_half_width, 200/GRID_SIZE, 2200/GRID_SIZE - grid_half_width, 0/GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_rectangle(table, grid_half_width, 200/GRID_SIZE, 2200/GRID_SIZE + 1, 0/GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_circle(table, 2210/GRID_SIZE, 200/GRID_SIZE, (half_width + 10) / GRID_SIZE, FORBIDDEN);
    
    //Central T forbidden
    cocobot_pathfinder_set_rectangle(table, 1200/GRID_SIZE, grid_half_width, 900/GRID_SIZE, 750/GRID_SIZE - grid_half_width, FORBIDDEN);
    cocobot_pathfinder_set_rectangle(table, 1200/GRID_SIZE, grid_half_width, 900/GRID_SIZE, 750/GRID_SIZE + 1, FORBIDDEN);
    cocobot_pathfinder_set_circle(table, 900/GRID_SIZE, 760/GRID_SIZE, (half_width + 10) / GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_circle(table, 2100/GRID_SIZE, 760/GRID_SIZE, (half_width + 10) / GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_rectangle(table, grid_half_width, 600/GRID_SIZE, 1470/GRID_SIZE - grid_half_width, 750/GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_rectangle(table, grid_half_width, 600/GRID_SIZE, 1470/GRID_SIZE + 1, 750/GRID_SIZE, FORBIDDEN);
    cocobot_pathfinder_set_circle(table, 1500/GRID_SIZE, 1350/GRID_SIZE, (half_width + 25) / GRID_SIZE, FORBIDDEN);
    
    //iles soft_obstacle
    //iles forbidden
    cocobot_pathfinder_set_circle(table, 0/GRID_SIZE, 2000/GRID_SIZE, (250 + half_width)/GRID_SIZE, FORBIDDEN); 
    cocobot_pathfinder_set_circle(table, 3000/GRID_SIZE, 2000/GRID_SIZE, (250 + half_width)/GRID_SIZE, FORBIDDEN); 

    //dune obstacle
    cocobot_pathfinder_set_rectangle(table, 20/GRID_SIZE, 200/GRID_SIZE, 800/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, 20/GRID_SIZE, 200/GRID_SIZE, 2200/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);

    // central T obstacle
    cocobot_pathfinder_set_rectangle(table, 1200/GRID_SIZE, 20/GRID_SIZE, 900/GRID_SIZE, 750/GRID_SIZE, OBSTACLE);
    cocobot_pathfinder_set_rectangle(table, 60/GRID_SIZE, 600/GRID_SIZE, 1470/GRID_SIZE, 750/GRID_SIZE, OBSTACLE);

    // iles obstacle
    cocobot_pathfinder_set_circle(table, 0/GRID_SIZE, 2000/GRID_SIZE, 250/GRID_SIZE, OBSTACLE); 
    cocobot_pathfinder_set_circle(table, 3000/GRID_SIZE, 2000/GRID_SIZE, 250/GRID_SIZE, OBSTACLE);

}

void cocobot_pathfinder_set_point_mask(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x, int y, cocobot_nodeType_e node_type)
{
    //Security to avoid buffer overflow
    if((x < TABLE_LENGTH/GRID_SIZE) && (y < TABLE_WIDTH/GRID_SIZE) && (x >= 0) && (y >= 0))
        table[x][y].nodeType |= node_type;
    //cocobot_console_send_asynchronous("TABLE","x=%d, y=%d status=%x", x, y, table[x][y].nodeType );
}

void cocobot_pathfinder_set_rectangle_mask(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x_dimension, int y_dimension, int x_position, int y_position, cocobot_nodeType_e node_type)
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
            cocobot_pathfinder_set_point_mask(table, x, y, node_type);
        }   
    }
}

void cocobot_pathfinder_set_circle_mask(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x_center, int y_center, int radius, cocobot_nodeType_e node_type)
{
    //A circle without a radius is nothing --> set radius to 1
    if(radius == 0)
        radius = 1;

    double xMax = 0.0;
    for(int y = y_center; y < y_center + radius; y++)
    {
        xMax = sqrtf((float)radius*(float)radius - (float)(y-y_center)*(float)(y-y_center));
        cocobot_pathfinder_set_rectangle_mask(table, (int)round(xMax), 1, x_center, y, node_type);
        cocobot_pathfinder_set_rectangle_mask(table, (int)round(xMax), 1, x_center - (int)round(xMax) , y, node_type);
        cocobot_pathfinder_set_rectangle_mask(table, (int)round(xMax), 1, x_center, 2*y_center - y - 1, node_type);
        cocobot_pathfinder_set_rectangle_mask(table, (int)round(xMax), 1, x_center - (int)round(xMax), 2*y_center - y - 1, node_type);
    }
}

void cocobot_pathfinder_set_point_unmask(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x, int y, cocobot_nodeType_e node_type)
{
    //Security to avoid buffer overflow
    if((x < TABLE_LENGTH/GRID_SIZE) && (y < TABLE_WIDTH/GRID_SIZE) && (x >= 0) && (y >= 0))
        table[x][y].nodeType &= ~node_type;
    //cocobot_console_send_asynchronous("TABLE","x=%d, y=%d status=%x", x, y, table[x][y].nodeType );
}

void cocobot_pathfinder_set_rectangle_unmask(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x_dimension, int y_dimension, int x_position, int y_position, cocobot_nodeType_e node_type)
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
            cocobot_pathfinder_set_point_unmask(table, x, y, node_type);
        }   
    }
}

void cocobot_pathfinder_set_circle_unmask(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x_center, int y_center, int radius, cocobot_nodeType_e node_type)
{
    //A circle without a radius is nothing --> set radius to 1
    if(radius == 0)
        radius = 1;

    double xMax = 0.0;
    for(int y = y_center; y < y_center + radius; y++)
    {
        xMax = sqrtf((float)radius*(float)radius - (float)(y-y_center)*(float)(y-y_center));
        cocobot_pathfinder_set_rectangle_unmask(table, (int)round(xMax), 1, x_center, y, node_type);
        cocobot_pathfinder_set_rectangle_unmask(table, (int)round(xMax), 1, x_center - (int)round(xMax) , y, node_type);
        cocobot_pathfinder_set_rectangle_unmask(table, (int)round(xMax), 1, x_center, 2*y_center - y - 1, node_type);
        cocobot_pathfinder_set_rectangle_unmask(table, (int)round(xMax), 1, x_center - (int)round(xMax), 2*y_center - y - 1, node_type);
    }
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
   if(cocobot_start_zone_allowed)
   {
       if(cocobot_game_state_get_color() == COCOBOT_GAME_STATE_COLOR_NEG)
       {
           cocobot_pathfinder_set_rectangle_mask(table, 300/GRID_SIZE, 500/GRID_SIZE, 0, 600/GRID_SIZE, TEMPORARY_ALLOWED);
       }
       else
       {
           cocobot_pathfinder_set_rectangle_mask(table, 300/GRID_SIZE, 500/GRID_SIZE, 2700/GRID_SIZE, 600/GRID_SIZE, TEMPORARY_ALLOWED);
       }
       cocobot_start_zone_allowed = 0;
   }  
}

void cocobot_pathfinder_set_start_zone_allowed()
{
    cocobot_start_zone_allowed = 1;
}

void cocobot_pathfinder_set_robot_zone(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], uint8_t x, uint8_t y)
{
    //cocobot_console_send_asynchronous("TABLE","Set robot x=%d, y=%d", x, y);
    cocobot_pathfinder_set_circle_mask(table, x, y, OPPONENT_HALF_DIAG/GRID_SIZE, ROBOT); 
}

void cocobot_pathfinder_unset_robot_zone(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], uint8_t x, uint8_t y)
{
    //cocobot_console_send_asynchronous("TABLE","Unset robot x=%d, y=%d", x, y);
    cocobot_pathfinder_set_circle_unmask(table, x, y, OPPONENT_HALF_DIAG/GRID_SIZE, ROBOT); 
}

void cocobot_pathfinder_set_point(cocobot_node_s table[][TABLE_WIDTH/GRID_SIZE], int x, int y, cocobot_nodeType_e node_type)
{
    //Security to avoid buffer overflow
    if((x < TABLE_LENGTH/GRID_SIZE) && (y < TABLE_WIDTH/GRID_SIZE) && (x >= 0) && (y >= 0))
        table[x][y].nodeType = node_type;
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
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center, y, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center - (int)round(xMax) , y, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center, 2*y_center - y - 1, node_type);
        cocobot_pathfinder_set_rectangle(table, (int)round(xMax), 1, x_center - (int)round(xMax), 2*y_center - y - 1, node_type);
    }
}
