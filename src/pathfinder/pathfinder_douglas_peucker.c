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
        if(cocobot_pathfinder_find_farest_point(trajectory, start_index, target_index, threshold) == NO_POINT_TO_KEEP)
        {
            start_index = target_index;
            start_index_sav = start_index;
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

