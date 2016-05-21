#ifndef COCOBOT_PATHFINDER_DOUGLAS_PEUCKER_H
#define COCOBOT_PATHFINDER_DOUGLAS_PEUCKER_H

#include "pathfinder_internal.h"

/**
 * Execute Douglas-Peucker algorythm on trajectory
 * NOTE: Algorythm is recurcive
 * Arguments:
 *  - trajectory : pointer on the trajectory to linearise
 *  - threshold : max radial distance (used for the algorythm)
 * 
 * Return Value : void
 */
void cocobot_pathfinder_douglas_peucker(cocobot_trajectory_final_s *trajectory, float threshold);

/**
 * Get radial distance between the point and the line passing by start and end point
 * Arguments:
 *  - start : first bound of the line
 *  - end : last bound of the line
 *  - point : point from witch radial distance is calculated
 *
 * Return Value: the distance
 */
float cocobot_pathfinder_get_radial_distance(cocobot_point_final_s start, cocobot_point_final_s end, cocobot_point_final_s point);


#endif
