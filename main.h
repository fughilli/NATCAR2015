#ifndef MAIN_H
#define MAIN_H

// some variables which control the strategy used by the car
typedef enum
{
    SAFE = 0,
    BEZIER = 1,
    AVERAGE = 2,
    WEIGHTED = 3
} strategy_t;

typedef enum
{
    SPEEDCONT_FAR_INVSQ = 0,
    SPEEDCONT_NEAR_INVSQ,
    SPEEDCONT_AVG_INVSQ,
    SPEEDCONT_WEIGHTED_INVSQ,
    SPEEDCONT_FAR_LINEAR,
    SPEEDCONT_NEAR_LINEAR,
    SPEEDCONT_AVG_LINEAR,
    SPEEDCONT_WEIGHTED_LINEAR
} speedcont_policy_t;

extern bool speed_control;
extern bool killswitch;

#endif // MAIN_H
