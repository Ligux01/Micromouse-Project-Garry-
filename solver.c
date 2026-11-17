/*
 * solver.c - Team T18 - robot name: Garry
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

static void turn_update_heading(int offset);
static int move_x_y(void);
static uint8_t decode_sense(uint8_t sense);
static char* trans_abs_to_chars (uint8_t walls); // translate absolute directions to characters
// N E S W
const static int x_change[] = {0,1,0,-1};
const static int y_change[] = {1,0,-1,0};
static int x = 0 , y = 0, heading = MM_NORTH;

static uint8_t walls_arr[MM_MAX_SIZE][MM_MAX_SIZE],
known_walls[MM_MAX_SIZE][MM_MAX_SIZE]; // uint8_t because we only need 
// 4 bits for direction: NESW   
// and this type uses the least amount which is a byte, since all modern machines are byte addressable now

static int distance[MM_MAX_SIZE][MM_MAX_SIZE]; // -1 means not reached yet

void solver_run(void)
{
    mm_begin("T18");
    
    
    mm_maze_t maze = mm_maze();
    mm_log("solver started: maze %dx%d", maze.width, maze.height);
    int goal[] = {maze.goal_x0,maze.goal_y0,maze.goal_x1,maze.goal_y1};
    
    /* note from claude
    y = height-1    ┌───────────┐
                    │           │      x grows east
    y = 0           └───────────┘      y grows north
                    x=0     x=width-1  (0,0) = south-west corner
    */

    for (int i = 0; i< maze.width;i++){

        for (int j = 0; j<maze.height;j++){

            if  (j == 0){
                walls_arr[i][j] |= MM_WALL(MM_SOUTH);
                known_walls[i][j] |= MM_WALL(MM_SOUTH);
            }
            if  (i == 0){
                walls_arr[i][j] |= MM_WALL(MM_WEST);
                known_walls[i][j] |= MM_WALL(MM_WEST);
            }

            if  (j == maze.height-1){
                walls_arr[i][j] |= MM_WALL(MM_NORTH);
                known_walls[i][j] |= MM_WALL(MM_NORTH);
            }

            if  (i == maze.width-1){
                walls_arr[i][j] |= MM_WALL(MM_EAST);
                known_walls[i][j] |= MM_WALL(MM_EAST);
            }
        }
    }
    mm_log("(0,0): %X\ntop-right corner: %X\ninner cell: %X",
         walls_arr[0][0],walls_arr[maze.width-1][maze.height-1],walls_arr[2][2]);

      

    mm_mark(x,y,MM_COLOR_GREEN);
    
    // mm_sense returns 3 bits in this order {MM_SEE_RIGHT, MM_SEE_FRONT, MM_SEE_LEFT}
    mm_log("%s",trans_abs_to_chars(decode_sense(mm_sense())));

    move_x_y();
    mm_log("%s",trans_abs_to_chars(decode_sense(mm_sense())));
    turn_update_heading(1);

    move_x_y();
    mm_log("%s",trans_abs_to_chars(decode_sense(mm_sense())));
    turn_update_heading(1);

    move_x_y();
    mm_log("%s",trans_abs_to_chars(decode_sense(mm_sense())));


    

    }

static void turn_update_heading(int offset){
    mm_turn(offset);
    heading = (heading + offset + 4) % 4;
}

static int move_x_y(void){
    int out = mm_forward(1);
    if(out == MM_OK){
        x += x_change[heading];
        y += y_change[heading];
    }
    mm_mark(x,y,MM_COLOR_GREEN);
    return out;
}

static uint8_t decode_sense (uint8_t sense){
    uint8_t res = 0;
    // mm_sense {MM_SEE_RIGHT, MM_SEE_FRONT, MM_SEE_LEFT}
    // { N , E , S , W}
    switch (sense){
    case 0:
        res = 0;
        break;
    case MM_SEE_LEFT:
        res |= MM_WALL((MM_WEST + heading) % 4);
        break;
    case MM_SEE_FRONT:
        res |= MM_WALL((MM_NORTH + heading) % 4);
        break;
    case MM_SEE_FRONT | MM_SEE_LEFT:
         res |= MM_WALL((MM_WEST + heading) % 4);
         res |= MM_WALL((MM_NORTH + heading) % 4);
        break;
    case MM_SEE_RIGHT:
        res |= MM_WALL((MM_EAST + heading) % 4);
        break;
    case MM_SEE_RIGHT | MM_SEE_LEFT:
        res |= MM_WALL((MM_WEST + heading) % 4);
        res |= MM_WALL((MM_EAST + heading) % 4);
        break;
    case MM_SEE_RIGHT | MM_SEE_FRONT:
        res |= MM_WALL((MM_NORTH + heading) % 4);
        res |= MM_WALL((MM_EAST + heading) % 4);
        break;
    case MM_SEE_RIGHT | MM_SEE_FRONT | MM_SEE_LEFT:
        res |= MM_WALL((MM_NORTH + heading) % 4);
        res |= MM_WALL((MM_EAST + heading) % 4);
        res |= MM_WALL((MM_WEST + heading) % 4);
        break;
    }
    return res;
}

static char* trans_abs_to_chars (uint8_t walls){

    switch (walls){
        //N E S W
        case 0:
            return "";
        case 1:
            return "W";
        case 2:
            return "S";
        case 3:
            return "SW";
        case 4:
            return "E";
        case 5:
            return "EW";
        case 6:
            return "ES";
        case 7:
            return "ESW";
        case 8:
            return "N";
        case 9:
            return "NW";
        case 10:
            return "NS";      
        case 11:
            return "NSW";  
        case 12:
            return "NE";
        case 13:
            return "NEW"; 
        case 14:
            return "NES"; 
        case 15:
            return "NESW";  
        
    }
}
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

