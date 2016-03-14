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
}cocobot_nodeType_e;

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t nodeType; //is a nodeType_e cast in uint8_t to be sure of the size
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

/**
 * A point on the table
 * x in mm
 * y in mm
 * 
 *   ___x___
 * y|       |
 *  |       |
 *   -------
 */
typedef struct 
{
    uint16_t x;
    uint16_t y;
}cocobot_point_s;

//TODO
typedef struct
{
    uint16_t execution_time;
}cocobot_trajectory_s;


/**
 * Get path from starting_point to target point
 * Arguments:
 *  - starting_point: trajectory starting point
 *  - target_point: trajectory target point 
 *  *
 * Return Value: t
 */
cocobot_strategy_t cocobot_pathfinder_get_trajectory(cocobot_point_s starting_point, cocobot_point_s target_point);

#endif //COCOBOT_PATHFINDER_H
