#ifndef COCOBOT_PATHFINDER_H
#define COCOBOT_PATHFINDER_H

#include "cocobot/pathfinder_table.h"

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
 *  
 * Return Value: cocobot_strategy_t : contains the trajectory
 */
cocobot_trajectory_s cocobot_pathfinder_get_trajectory(cocobot_point_s starting_point, cocobot_point_s target_point);

#endif //COCOBOT_PATHFINDER_H
