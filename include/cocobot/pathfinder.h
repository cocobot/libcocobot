#ifndef COCOBOT_PATHFINDER_H
#define COCOBOT_PATHFINDER_H

#include "cocobot/pathfinder_table.h"

#define NO_TRAJECTORY_AVAILABLE -1
#define TRAJECTORY_AVAILABLE 0

#define TRAJECTORY_NBR_POINTS_MAX 160

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
    cocobot_point_s trajectory[TRAJECTORY_NBR_POINTS_MAX]; //TODO:Adjust number of points
    uint8_t nbr_points; // Number of points in the trajectory
    uint16_t execution_time; // ms?
}cocobot_trajectory_s;


/**
 * Get path from starting_point to target point
 * Arguments:
 *  - starting_point: trajectory starting point
 *  - target_point: trajectory target point
 *  - trajectory : pointer on the final trajectory 
 *  
 * Return Value: either NO_TRAJECTORY_AVAILABLE or TRAJECTORY_OK
 */
char cocobot_pathfinder_get_trajectory(cocobot_point_s starting_point, cocobot_point_s target_point, cocobot_trajectory_s *trajectory);

/**
 * Initialize the pathFinder
 * Arguments : TBD 
 *  - Maybe robot speed/acceleration
 *  - robot dimensionc
 */
void cocobot_pathfinder_init();

#endif //COCOBOT_PATHFINDER_H
