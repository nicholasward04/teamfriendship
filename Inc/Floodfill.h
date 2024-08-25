/*
 * Module for the implementation of floodfill
 */

#ifndef __Floodfill_H
#define __Floodfill_H

#include "IR.h"

#include <stdbool.h>
#include <stdlib.h>

typedef enum{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
} Direction;

typedef enum{
    NORTH_MASK = 0b1000,
    EAST_MASK  = 0b0100,
    SOUTH_MASK = 0b0010,
    WEST_MASK  = 0b0001
} DirectionBitmask;

typedef struct{
    int x;
    int y;
} Coord;

typedef struct{
    Coord pos;
    Direction dir;
    bool blocked;
} Cell;

typedef struct{
    int size;
    Cell* cells;
} CellList;

typedef struct{
	Coord mouse_pos;
	Direction mouse_dir;

    uint8_t distances[16][16];
    uint8_t cellWalls[16][16];

    Coord goalPos[4];
}Maze;

void init_maze(Maze* maze);

bool offMaze(int mouse_pos_x, int mouse_pos_y);

CellList* getNeighborCells(Maze* maze, Coord pos);

void scanWalls(Maze* maze);

void updateMousePos(Coord *pos, Direction dir);

void setGoalCell(Maze* maze, int num_of_goals);

void Floodfill(Maze* maze);

Cell bestCell(Maze* maze);

#endif
