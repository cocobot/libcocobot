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

/**
 * Initilize the final traj struct
 * Arguments:
 *  - in_traj : trajectory from pathfinder
 *  - final_traj : trajectory used for Douglas peucker reduction
 */
void cocobot_pathfinder_init_final_traj(cocobot_trajectory_s *in_traj, cocobot_trajectory_final_s *final_traj);

/**
 * Get next TO_KEEP point in the Douglas Peucker trajectory
 * Arguments:
 *  - trajectory : pointer on the D-P traj
 *  - start_index : index of the starting point in the traj
 *  - target_index : index of the target point in the traj
 */
uint8_t cocobot_pathfinder_get_next_point(cocobot_trajectory_final_s *trajectory, uint8_t start_index, uint8_t target_index);

/**
 * Find farest (radial distance) in the traj defined by the starting point start_index and target point at target_index
 * Arguments:
 *  - traj : pointer on the final traj
 *  - start_index : index of starting point 
 *  - target_index : index of the target point in traj
 *  - threshold : threshold used by D-P algorythm
 */
uint8_t cocobot_pathfinder_find_farest_point(cocobot_trajectory_final_s *traj, uint8_t start_index, uint8_t target_index, float threshold);

/**
 * Concatenate two trajectories into one
 * Arguments:
 *  - first : pointer on the first trajectory. It's also in that trajectory that the second will be merged
 *  - second : pointer on the second trajectory, merged into first
 */
void cocobot_pathfinder_concatenate_traj(cocobot_trajectory_s *first, cocobot_trajectory_s *second);

/**
 * Cut base trajectory into two others first and second around cut_index
 * Arguments:
 *  - base: pointer on the trajectory to be cut
 *  - first : pointer on the first part of cut trajectory
 *  - second: pointer on the second part of the cut traj
 *  - cut_index : determine where the base is cut
 */
void cocobot_pathfinder_cut_trajectory(cocobot_trajectory_s *base, cocobot_trajectory_s *first, cocobot_trajectory_s *second, uint8_t cut_index);


#endif
