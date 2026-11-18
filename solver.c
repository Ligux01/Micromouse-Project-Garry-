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

#include <stdio.h>
#include <string.h>
#include "mouse_hal.h"

typedef struct{
    int x;
    int y;
}crd;
typedef struct {
    crd arr[MM_MAX_SIZE*MM_MAX_SIZE];
    int head;
    int tail;
    int count;
} Queue;

static void turn_update_heading(int);
static int move_x_y(void);
static const char* walls_to_text (uint8_t); // translate absolute directions to characters
static uint8_t decode_and_look(void);
static void record_a_side(int,int);
static void printMaze(uint8_t [16][16]);
static void printDistance(void);
void initialize(Queue*);
int isEmpty(Queue*);
void enqueue(Queue*, int,int);
crd dequeue(Queue*);
static void push_neighbors(crd, int);

// N E S W
const static int x_change[] = {0,1,0,-1};
const static int y_change[] = {1,0,-1,0};
static int x = 0 , y = 0, heading = MM_NORTH;
static mm_maze_t maze;
static Queue crds_queue;

static uint8_t walls_arr[MM_MAX_SIZE][MM_MAX_SIZE],
known_walls[MM_MAX_SIZE][MM_MAX_SIZE]; // uint8_t because we only need 
// 4 bits for direction: NESW   
// and this type uses the least amount which is a byte, since all modern machines are byte addressable now

static int distance[MM_MAX_SIZE][MM_MAX_SIZE]; // -1 means not reached yet

void solver_run(void)
{
    mm_begin("T18");
    
    maze = mm_maze();
    mm_log("solver started: maze %d x %d", maze.width, maze.height);
    int goal[] = {maze.goal_x0,maze.goal_y0,maze.goal_x1,maze.goal_y1};
    initialize(&crds_queue);

    /* note from claude
    y = height-1    ┌───────────┐
                    │           │      x grows east
    y = 0           └───────────┘      y grows north
                    x=0     x=width-1  (0,0) = south-west corner
    */

    for (int i = 0; i< maze.width;i++){

        for (int j = 0; j< maze.height;j++){
            // Filling the distance array
            if((i >= goal[0] && i <= goal[2]) && (j >= goal[1] && j <= goal[3])){
            // the goal square distance is 0
            distance[i][j] = 0;
            enqueue(&crds_queue,i,j);
            }
            else
            distance[i][j] = -1;    // others are unkown or -1

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
    
    
    while(!isEmpty(&crds_queue)){
        crd res = dequeue(&crds_queue);
        
        push_neighbors(res,MM_NORTH);
        push_neighbors(res,MM_EAST);
        push_neighbors(res,MM_WEST);
        push_neighbors(res,MM_SOUTH);
    }

         
    
    // mm_sense returns 3 bits in this order {MM_SEE_RIGHT, MM_SEE_FRONT, MM_SEE_LEFT}
    decode_and_look();
    mm_log("%s",walls_to_text(walls_arr[x][y]));
    mm_mark(x,y,MM_COLOR_GREEN);


    move_x_y();
    decode_and_look();
    mm_log("%s",walls_to_text(walls_arr[x][y]));
    turn_update_heading(1);

    move_x_y();
    decode_and_look();
    mm_log("%s",walls_to_text(walls_arr[x][y]));
    turn_update_heading(1);

    move_x_y();
    decode_and_look();
    mm_log("%s",walls_to_text(walls_arr[x][y]));

    printMaze(walls_arr);
    printDistance();

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



static const char* walls_to_text (uint8_t walls){

    switch (walls){
        //N E S W
        case 0: return "NON";
        case 1:  return "W";
        case 2:  return "S";
        case 3:  return "SW";
        case 4:  return "E";
        case 5:  return "EW";
        case 6:  return "ES";
        case 7:  return "ESW";
        case 8:  return "N";
        case 9:  return "NW";
        case 10: return "NS";      
        case 11: return "NSW";  
        case 12: return "NE";
        case 13: return "NEW"; 
        case 14: return "NES"; 
        case 15: return "NESW";  
        default: return "ERR";
        
    }
}
static void record_a_side(int dir, int wall){
    known_walls[x][y] |= MM_WALL(dir);
    int neighbor_x = x + x_change[dir];
    int neighbor_y = y + y_change[dir];
    if((neighbor_x >= 0 && neighbor_x < maze.width) && (neighbor_y >= 0 && neighbor_y < maze.height) )
    known_walls[neighbor_x][neighbor_y] |= MM_WALL(((dir + 2) % 4)) ;

    if(wall){
        walls_arr[x][y] |= MM_WALL(dir);
        if((neighbor_x >= 0 && neighbor_x < maze.width) && (neighbor_y >= 0 && neighbor_y < maze.height) )
        walls_arr[neighbor_x][neighbor_y] |= MM_WALL(((dir + 2) % 4)) ;
    }
}

static uint8_t decode_and_look(){
    uint8_t s = mm_sense();
    uint8_t res = 0;
    
    int front_absolute = (MM_NORTH + heading) % 4;
    int left_absolute = (MM_WEST + heading) % 4;
    int right_absolute = (MM_EAST + heading) % 4;

    record_a_side(front_absolute,(s & MM_SEE_FRONT));
    record_a_side(left_absolute,(s & MM_SEE_LEFT));
    record_a_side(right_absolute,(s & MM_SEE_RIGHT));

    if(s & MM_SEE_FRONT)
    res |= MM_WALL(front_absolute); 
    if(s & MM_SEE_LEFT)
    res |= MM_WALL(left_absolute);
    if(s & MM_SEE_RIGHT)
    res |= MM_WALL(right_absolute);
    
    return res;

}


// AI Code for Printing
static void printMaze(uint8_t maze_arr[16][16]) {
    char rowBuffer[128]; // Buffer to hold one full row of text
    char cellStr[16];    // Temporary buffer for one cell's number

    for (int r = 15; r >= 0; r--) {
        rowBuffer[0] = '\0'; // Reset buffer at the start of each row
        
        for (int c = 0; c < 16; c++) {
            // Format each integer into a string with 3 spaces of padding
            snprintf(cellStr, sizeof(cellStr), "%-4s ", walls_to_text(maze_arr[c][r]));
            strcat(rowBuffer, cellStr);
        }
        
        // Pass the string using a "%s" placeholder in case mm_log is variadic (like printf)
        mm_log("%s", rowBuffer); 
    }
    mm_log("%s", "------------------------------------------------");
}

static void printDistance(void){
    char rowBuffer[255]; // Buffer to hold one full row of text
    char cellStr[16];    // Temporary buffer for one cell's number

    for (int r = 15; r >= 0; r--) {
        rowBuffer[0] = '\0'; // Reset buffer at the start of each row
        
        for (int c = 0; c < 16; c++) {
            // Format each integer into a string with 3 spaces of padding
            snprintf(cellStr, sizeof(cellStr), "%3d\t", distance[c][r]);
            strcat(rowBuffer, cellStr);
        }
        
        // Pass the string using a "%s" placeholder in case mm_log is variadic (like printf)
        mm_log("%s", rowBuffer); 
    }
    mm_log("%s", "------------------------------------------------");
}

void initialize(Queue *q) {
    q->head = 0;
    q->tail = -1;
    q->count = 0;
}

int isEmpty(Queue *q) {
    return q->count == 0;
}

// Insert an element
void enqueue(Queue *q, int x,int y) {

    q->tail = (q->tail + 1);
    q->arr[q->tail].x = x;
    q->arr[q->tail].y = y;
    q->count++;

}

// Remove an element
crd dequeue(Queue *q) {
    if (isEmpty(q)) {
        return (crd){-1,-1};
    }

    crd value;
    value.x = q->arr[q->head].x;
    value.y = q->arr[q->head].y;

    q->head = (q->head + 1);
    q->count--;

    return value;
}

static void push_neighbors (crd res,int dir){

    int current_cost = distance[res.x][res.y] + 1;
    crd neighbor = (crd){res.x + x_change[dir],res.y + y_change[dir]};
        

        if((neighbor.x >= 0 && neighbor.x < maze.width) && 
        (neighbor.y >= 0 && neighbor.y < maze.height) && 
        (distance[neighbor.x][neighbor.y] == -1 ) &&
        !(walls_arr[res.x][res.y] &  MM_WALL(dir))){
            enqueue(&crds_queue,neighbor.x,neighbor.y);
            distance[neighbor.x][neighbor.y] = current_cost;
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

