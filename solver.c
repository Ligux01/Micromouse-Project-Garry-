/*
 * solver.c - Team T?? - robot name: ????
 *
 * Your maze solver. This file must:
 *   - include only mouse_hal.h and standard C headers (no simulator API,
 *     no Arduino or ESP32 headers),
 *   - talk to the mouse only through mm_... functions,
 *   - never print with printf(); use mm_log() instead,
 *   - never assume the maze size or goal; read them from mm_maze().
 * The same file must build unchanged for the simulator and for the ESP32.
 */
#include "mouse_hal.h"

void solver_run(void)
{
    mm_begin("T18");
    static int x = 0 , y = 0, heading = MM_NORTH;
    static uint8_t walls_arr[MM_MAX_SIZE][MM_MAX_SIZE],
    known_walls[MM_MAX_SIZE][MM_MAX_SIZE]; // uint8_t because we only need 4 bits 
    // and this type uses the least amount which is a byte, since all modern machines are byte addressable now
    static int distance[MM_MAX_SIZE][MM_MAX_SIZE]; // -1 means not reached yet
    
    mm_maze_t maze = mm_maze();
    mm_log("solver started: maze %dx%d", maze.width, maze.height);
    int goal = {maze.goal_x0,maze.goal_y0,maze.goal_x1,maze.goal_y1};
    
    /* note from claude
     y = height-1   ┌───────────┐
                    │           │      x grows east
    y = 0           └───────────┘      y grows north
                x=0     x=width-1  (0,0) = south-west corner
    */
    for (int x = 0; x< maze.width;x++){

        for (int y = 0; y<maze.height;y++){

            if  (y == 0){
                walls_arr[x][y] |= MM_WALL(MM_SOUTH);
                known_walls[x][y] |= MM_WALL(MM_SOUTH);
            }
            if  (x == 0){
                walls_arr[x][y] |= MM_WALL(MM_WEST);
                known_walls[x][y] |= MM_WALL(MM_WEST);
            }

            if  (y == maze.height-1){
                walls_arr[x][y] |= MM_WALL(MM_NORTH);
                known_walls[x][y] |= MM_WALL(MM_NORTH);
            }

            if  (x == maze.width-1){
                walls_arr[x][y] |= MM_WALL(MM_EAST);
                known_walls[x][y] |= MM_WALL(MM_EAST);
            }
        }
    }
    mm_log("(0,0): %X\n(15,15): %X\n(3,4): %X", walls_arr[0][0],walls_arr[15][15],walls_arr[3][4]);

    /* M1 step 1 (week of 25 Oct): map and position
    *   - a wall map for every cell, using MM_WALL(dir) bits
     *   - track x, y and heading after every mm_forward() / mm_turn()
     *   - convert mm_sense() (relative) into absolute walls
     */
     



     /* M1 step 2 (week of 1 Nov): flood fill with a queue, show the
     *   distances with mm_label()
     */
     /* M1 step 3 (week of 8 Nov): explore to the goal, return to the
     *   start, then run the shortest known path
     */
     /* M1 step 4 (week of 15 Nov): report your map with mm_map_begin(),
     *   mm_map_cell() for every cell, mm_map_end()
     */
}
