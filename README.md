# Garry (Team T18)

ENCS4380 Interfacing Techniques, Birzeit University, first semester 2026/27.

Four of us are building a micromouse: a small robot that explores a maze it has never seen, finds the centre, and then races back to it as fast as it can. This repo holds the maze solver and, later, the robot code. Names and roles are in TEAM.md.

The project follows the course milestone plan. M1 is the solver running in the mms simulator, which is what is in `solver.c` now. Next is M2, the 3D-printed chassis. Each milestone gets a git tag (M1, M2, and so on).

## How the solver works

The mouse keeps two 16x16 arrays of bytes. One records the walls it believes exist, the other records which sides it has actually looked at. Each side of a cell is one bit, set with `MM_WALL(dir)`. A side it has not looked at counts as open when planning, and as unknown when we report the map. Every wall is written on both cells that share it.

The hardware layer does not track position, so the solver does. It updates x, y and heading after every move and turn, and converts the sensor's left/front/right reading into compass directions using the heading.

To decide where to go, it floods the maze from a target (the goal block, or the start cell on the way home). That is a breadth-first search with a plain array as the queue, no recursion, so every cell ends up with its distance to the target. Known walls block the spreading. The mouse then steps to the open neighbour with the smallest number, senses the new cell, and floods again, so the route bends around every wall it discovers.

Because unseen walls count as open, the flood always gives a best case. We use that to decide when to stop exploring. The mouse repeats the start-to-goal trip until one trip learns no new wall. That trip followed a route no shorter one can beat, so it is the one we keep for the speed run.

At the end of a run the solver reports its whole map with `mm_map_begin`, `mm_map_cell` and `mm_map_end`. Sides it never saw are reported as unknown.

## Build and run in mms

Install mms, add a mouse, and set the directory to this folder.

Windows (MinGW):

    gcc -O2 -std=c99 -o solver.exe main_sim.c hal_sim.c mm_common.c solver.c

For the run command, mms needed the full path to `solver.exe` on our machines. A bare file name did not work.

Linux or the course VM:

    gcc -O2 -std=c99 -o solver main_sim.c hal_sim.c mm_common.c solver.c
    ./solver

Then load a maze from `mazes/` and press Run. Debug output goes through `mm_log`, never `printf`, because the simulator reads stdout as commands.

To test a different goal, copy the maze's `.goal` file to `mm_goal.txt` in this folder. Delete the file to go back to the centre.

`example_wallfollow.c` is the instructor's wall follower, not ours. Build it in place of `solver.c` if you want to watch it fail on a maze whose goal is not connected to the outer wall.

## What is in the repo

`solver.c` is ours. The files it sits next to come from the starter kit and should not be edited: `mouse_hal.h`, `mm_internal.h`, `mm_common.c`, `hal_sim.c`, `main_sim.c`, `Makefile`. `mazes/` holds the practice mazes plus the one we designed. `esp32/micromouse/` is the robot side and is still mostly empty. `.gitignore` keeps compiled files out of the repo.

## Notes

The debug printers `printMaze` and `printDistance` in `solver.c` came from an AI tool and are labelled as such in the code.
