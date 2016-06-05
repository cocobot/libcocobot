#include "cocobot/pathfinder_douglas_peucker.h"
#include <cocobot.h>
#include <math.h>

void cocobot_pathfinder_douglas_peucker(cocobot_trajectory_final_s *trajectory, float threshold)
{
    cocobot_console_send_asynchronous("DOUGLAS:","Starting douglas peucker algo");
    uint8_t start_index = 0;
    uint8_t start_index_sav = 0;
    uint8_t target_index = trajectory->nbr_points - 1;
    while(start_index != target_index)
    {
        //cocobot_console_send_asynchronous("DOUGLAS:","start_index = %d target_index = %d", start_index, target_index);
        if(cocobot_pathfinder_find_farthest_point(trajectory, start_index, target_index, threshold) == NO_POINT_TO_KEEP)
        {
            start_index = target_index;
            start_index_sav = start_index;
            target_index = cocobot_pathfinder_get_next_point(trajectory, start_index, trajectory->nbr_points - 1);
        }
        else
        {
            start_index = start_index_sav;
            target_index = cocobot_pathfinder_get_next_point(trajectory, start_index, target_index);
        }
    }
}

float cocobot_pathfinder_get_radial_distance(cocobot_point_final_s start, cocobot_point_final_s end, cocobot_point_final_s point)
{
    return fabsf(((float)(end.y - start.y)/(float)(end.x - start.x)) * (float)(point.x - start.x) - (float)point.y + (float)start.y)/sqrtf(1.0+(((float)(end.y - start.y)/(float)(end.x - start.x)) * ((float)(end.y - start.y)/(float)(end.x - start.x))));
}

void cocobot_pathfinder_init_final_traj(cocobot_trajectory_s *in_traj, cocobot_trajectory_final_s *final_traj)
{
    final_traj->trajectory[0].status = POINT_TO_KEEP;
    int i = 0;
    for(i = 0; i < in_traj->nbr_points; i++)
    {
        final_traj->trajectory[i].x = in_traj->trajectory[i].x;
        final_traj->trajectory[i].y = in_traj->trajectory[i].y;
    }
    final_traj->trajectory[i].status = POINT_TO_KEEP;
    final_traj->nbr_points = in_traj->nbr_points;
}

uint8_t cocobot_pathfinder_get_next_point(cocobot_trajectory_final_s *trajectory, uint8_t start_index, uint8_t target_index)
{
    int i = 0;
    if(start_index >= target_index)
    {
        return target_index;
    }
    else
    {
        for(i = (start_index+1); i <= target_index; i++)
        {
            if(trajectory->trajectory[i].status == POINT_TO_KEEP)
                return i;
        }
        return i;
    }
}

uint8_t cocobot_pathfinder_find_farthest_point(cocobot_trajectory_final_s *traj, uint8_t start_index, uint8_t target_index, float threshold)
{
    float d = 0.0;
    float dMax = 0.0;
    uint8_t index = 0;
    uint8_t returnValue = NO_POINT_TO_KEEP;
    for(int i = start_index; i <= target_index; i++)
    {
        d = cocobot_pathfinder_get_radial_distance(traj->trajectory[start_index], traj->trajectory[target_index], traj->trajectory[i]);
        //cocobot_console_send_asynchronous("DOUGLAS:","index: %d dmax: %f, d: %f", i, (double)dMax, (double)d);
        if (d > dMax)
        {
            dMax = d;
            index = i;
        }
    }
    
    if(dMax >= threshold)
    {
        traj->trajectory[index].status = POINT_TO_KEEP;
        //cocobot_console_send_asynchronous("DOUGLAS","to keep: index: %d", index);
        returnValue = POINT_TO_KEEP;
    }
    else
        returnValue = NO_POINT_TO_KEEP;

    return returnValue;
}

