#ifndef COCOBOT_PATHFINDER_TABLE_H
#define COCOBOT_PATHFINDER_TABLE_H

#include <stdint.h>

#define TABLE_LENGTH 300
#define TABLE_WIDTH 200
#define GRID_SIZE 5

typedef enum
{
    NEW_NODE = 0,
    OBSTACLE,
    CLOSED_LIST,
    FINAL_TRAJ,
    OPEN_LIST,
    ROBOT
}cocobot_nodeType_e;

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t nodeType; //is a cocobot_nodeType_e cast in uint8_t to be sure of the size
    float cost;
    uint8_t pX;
    uint8_t pY;
}cocobot_node_s;


typedef struct list list_s;

struct list
{
    cocobot_node_s* p_node;
    list_s* p_nextElement;
};

//Table related functions
void initialiseTable(cocobot_node_s _table[][TABLE_WIDTH/GRID_SIZE], int _length, int _width);
void setPoint(cocobot_node_s _p_table[][TABLE_WIDTH/GRID_SIZE], int _x, int _y, cocobot_nodeType_e _nodeType);
void setRectangle(cocobot_node_s _p_table[][TABLE_WIDTH/GRID_SIZE], int _xDimension, int _yDimension, int _xPosition, int _yPosition, cocobot_nodeType_e _nodeType);
void setCircle(cocobot_node_s _p_table[][TABLE_WIDTH/GRID_SIZE], int _xCenter, int _yCenter, int _radius, cocobot_nodeType_e _nodeType);

#endif //COCOBOT_PATHFINDER_TABLE_H
