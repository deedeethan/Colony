/**
 * @file scoutsim_internal.h
 *
 * @ingroup scoutsim
 * @{
 */

#ifndef _SCOUTSIM_INTERNAL_
#define _SCOUTSIM_INTERNAL_

    /**
     * State of the whole world, positions of all scouts!
     * Passed by reference to scouts for reading on update loop.
     * Updated only by sim_frame to prevent concurrency problems.
     */
    typedef struct world_state
    {
        float canvas_width;
        float canvas_height;

    } world_state;

#endif /* _SCOUTSIM_INTERNAL_ */

/** @} */
