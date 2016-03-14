#ifndef COCOBOT_PATHFINDER_H
#define COCOBOT_PATHFINDER_H

typedef enum
{
    NEW_NODE = 0,
    OBSTACLE,
    CLOSED_LIST,
    FINAL_TRAJ,
    OPEN_LIST,
    ROBOT
}nodeType_e;

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t nodeType; //is a nodeType_e cast in uint8_t to be sure of the size
    float cost;
    uint8_t pX;
    uint8_t pY;
}node_s;

typedef struct list list_s;

struct list
{
    node_s* p_node;
    list_s* p_nextElement;
};



#endif //COCOBOT_PATHFINDER_H
