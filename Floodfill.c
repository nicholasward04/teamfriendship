/*
 * Module for the implementation of floodfill
 */
#include "Floodfill.h"

bool front;
bool left;
bool right;

char dir_chars[4] = {'n', 'e', 's', 'w'};
int dir_mask[4] = {0b1000, 0b0100, 0b0010, 0b0001};

void init_maze(Maze* maze) {
	for (int x = 0; x < 16; x++) {
		for (int y = 0; y < 16; y++) {
			maze->cellWalls[y][x] = 0;
			if  (x == 0) {
				maze->cellWalls[y][x] |= WEST_MASK;
			}
			if (y == 0) {
				maze->cellWalls[y][x] |= SOUTH_MASK;
			}
			if (x == 15) {
				maze->cellWalls[y][x] |= NORTH_MASK;
			}
			if (y == 15) {
				maze->cellWalls[y][x] |= EAST_MASK;
			}
		}
	}
}

bool offMaze(int mouse_pos_x, int mouse_pos_y)
{
    if (mouse_pos_x < 0 || mouse_pos_x > 15 || mouse_pos_y < 0 || mouse_pos_y > 15)
    {
        return false; // False if offMaze for coding purposes
    }
    return true;
}

CellList* getNeighborCells(Maze* maze, Coord pos)
{
	CellList* cellList = (CellList*)malloc(sizeof(CellList));

    bool north = true;
    bool east = true;
    bool south = true;
    bool west = true;

    int x_coord = pos.x;
    int y_coord = pos.y;
    int size_to_set = 0;

    // if the neighbor in that dir is not on the maze, set false
    if (!offMaze(x_coord + 1, y_coord))
    {
        east = false;
    }
    else
    {
        size_to_set++;
    }
    if (!offMaze(x_coord - 1, y_coord))
    {
        west = false;
    }
    else
    {
        size_to_set++;
    }
    if (!offMaze(x_coord, y_coord + 1))
    {
        north = false;
    }
    else
    {
        size_to_set++;
    }
    if (!offMaze(x_coord, y_coord - 1))
    {
        south = false;
    }
    else
    {
        size_to_set++;
    }

    cellList->size = size_to_set;
    cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell));

    int i = 0;
    if (north)
    {
        bool north_blocked = NORTH_MASK & maze->cellWalls[y_coord][x_coord];
        Cell new_cell;
        new_cell.blocked = north_blocked;
        new_cell.dir = NORTH;
        new_cell.pos.x = x_coord;
        new_cell.pos.y = y_coord + 1;
        cellList->cells[i] = new_cell;
        i++;
    }
    if (east)
    {
        bool east_blocked = EAST_MASK & maze->cellWalls[y_coord][x_coord];
        Cell new_cell;
        new_cell.blocked = east_blocked;
        new_cell.dir = EAST;
        new_cell.pos.x = x_coord + 1;
        new_cell.pos.y = y_coord;
        cellList->cells[i] = new_cell;
        i++;
    }
    if (south)
    {
        bool south_blocked = SOUTH_MASK & maze->cellWalls[y_coord][x_coord];
        Cell new_cell;
        new_cell.blocked = south_blocked;
        new_cell.dir = SOUTH;
        new_cell.pos.x = x_coord;
        new_cell.pos.y = y_coord - 1;
        cellList->cells[i] = new_cell;;
        i++;
    }
    if (west)
    {
        bool west_blocked = WEST_MASK & maze->cellWalls[y_coord][x_coord];
        Cell new_cell;
        new_cell.blocked = west_blocked;
        new_cell.dir = WEST;
        new_cell.pos.x = x_coord - 1;
        new_cell.pos.y = y_coord;
        cellList->cells[i] = new_cell;;
        i++;
    }
    return cellList;
};

void scanWalls(Maze* maze)
{
	poll_sensors();
	front = wallFront();
    if (wallFront())
    {
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[maze->mouse_dir];
        if (maze->mouse_dir == NORTH)
        {
            if (offMaze(maze->mouse_pos.x, maze->mouse_pos.y + 1))
            {
                maze->cellWalls[maze->mouse_pos.y + 1][maze->mouse_pos.x] |= SOUTH_MASK;
            }
        }
        else if (maze->mouse_dir == EAST)
        {
            if (offMaze(maze->mouse_pos.x + 1, maze->mouse_pos.y))
            {
                maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x + 1] |= WEST_MASK;
            }
        }
        else if (maze->mouse_dir == SOUTH)
        {
            if (offMaze(maze->mouse_pos.x, maze->mouse_pos.y - 1))
            {
                maze->cellWalls[maze->mouse_pos.y - 1][maze->mouse_pos.x] |= NORTH_MASK;
            }
        }
        else if (maze->mouse_dir == WEST)
        {
            if (offMaze(maze->mouse_pos.x - 1, maze->mouse_pos.y))
            {
                maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x - 1] |= EAST_MASK;
            }
        }
    }
    right = wallRight();
    if (wallRight())
    {
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 1) % 4];
        if (maze->mouse_dir == NORTH)
        {
            if (offMaze(maze->mouse_pos.x + 1, maze->mouse_pos.y))
            {
                maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x + 1] |= WEST_MASK;
            }
        }
        else if (maze->mouse_dir == EAST)
        {
            if (offMaze(maze->mouse_pos.x, maze->mouse_pos.y - 1))
            {
                maze->cellWalls[maze->mouse_pos.y - 1][maze->mouse_pos.x] |= NORTH_MASK;
            }
        }
        else if (maze->mouse_dir == SOUTH)
        {
            if (offMaze(maze->mouse_pos.x - 1, maze->mouse_pos.y))
            {
                maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x - 1] |= EAST_MASK;
            }
        }
        else if (maze->mouse_dir == WEST)
        {
            if (offMaze(maze->mouse_pos.x, maze->mouse_pos.y + 1))
            {
                maze->cellWalls[maze->mouse_pos.y + 1][maze->mouse_pos.x] |= SOUTH_MASK;
            }
        }
    }
    left = wallLeft();
    if (wallLeft())
    {
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 3) % 4];
        if (maze->mouse_dir == NORTH)
        {
            if (offMaze(maze->mouse_pos.x - 1, maze->mouse_pos.y))
            {
                maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x - 1] |= EAST_MASK;
            }
        }
        else if (maze->mouse_dir == EAST)
        {
            if (offMaze(maze->mouse_pos.x, maze->mouse_pos.y + 1))
            {
                maze->cellWalls[maze->mouse_pos.y + 1][maze->mouse_pos.x] |= SOUTH_MASK;
            }
        }
        else if (maze->mouse_dir == SOUTH)
        {
            if (offMaze(maze->mouse_pos.x + 1, maze->mouse_pos.y))
            {
                maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x + 1] |= WEST_MASK;
            }
        }
        else if (maze->mouse_dir == WEST)
        {
            if (offMaze(maze->mouse_pos.x, maze->mouse_pos.y - 1))
            {
                maze->cellWalls[maze->mouse_pos.y - 1][maze->mouse_pos.x] |= NORTH_MASK;
            }
        }
    }
}

void updateMousePos(Coord *pos, Direction dir)
{
    if (dir == NORTH)
        pos->y++;
    if (dir == SOUTH)
        pos->y--;
    if (dir == WEST)
        pos->x--;
    if (dir == EAST)
        pos->x++;
}

void setGoalCell(Maze* maze, int num_of_goals) // Changed goalPos to static array
{
    if (num_of_goals == 4)
    {
        maze->goalPos[0] = (Coord){7, 7};
        maze->goalPos[1] = (Coord){8, 7};
        maze->goalPos[2] = (Coord){7, 8};
        maze->goalPos[3] = (Coord){8, 8};
    }
    else
    {
        maze->goalPos[0] = (Coord){0, 0};
    }
}

void Floodfill(Maze* maze)
{
    int MAX_COST = 255;

    for (int x = 0; x < 16; x++)
    {
        for (int y = 0; y < 16; y++)
        {
            maze->distances[y][x] = MAX_COST;
        }
    }

    int goal_loop_size = 4;

    if ((maze->goalPos[0].x == 0) && ((maze->goalPos[0].y == 0)))
    {
        goal_loop_size = 1;
    }

    for (int cell = 0; cell < goal_loop_size; cell++)
    {
        maze->distances[maze->goalPos[cell].y][maze->goalPos[cell].x] = 0;
    }

    Coord queue[255];
    int head = 0;
    int tail = 0;

    for (int cell = 0; cell < goal_loop_size; cell++)
    {
        queue[tail] = maze->goalPos[cell];
        tail++;
    }

    while ((tail - head) > 0)
    {
    	Coord pos = queue[head];
        head++;
        int newdistance = maze->distances[pos.y][pos.x] + 1;

        CellList* neighborCells = getNeighborCells(maze, pos);

        for (int neighbor = 0; neighbor < neighborCells->size; neighbor++)
        {
            if (!neighborCells->cells[neighbor].blocked)
            {
                if (maze->distances[neighborCells->cells[neighbor].pos.y][neighborCells->cells[neighbor].pos.x] > newdistance)
                {
                    maze->distances[neighborCells->cells[neighbor].pos.y][neighborCells->cells[neighbor].pos.x] = newdistance;
                    queue[tail] = neighborCells->cells[neighbor].pos;
                    tail++;
                }
            }
        }

        free(neighborCells->cells);
        free(neighborCells);
    }

}

Cell bestCell(Maze* maze)
{
	CellList* adjacentCells = getNeighborCells(maze, maze->mouse_pos);

	Cell best_cell;
	int best_cell_ind = 0;
	int best_value = maze->distances[maze->mouse_pos.y][maze->mouse_pos.x];
	int loop_size = adjacentCells->size;
	int turn_around_ind;

	for (int cell = 0; cell < loop_size; cell++)
	{
		if (((maze->distances[adjacentCells->cells[cell].pos.y][adjacentCells->cells[cell].pos.x] < best_value) ||
			((maze->distances[adjacentCells->cells[cell].pos.y][adjacentCells->cells[cell].pos.x] == best_value) &&
			 (maze->mouse_dir == adjacentCells->cells[cell].dir))) && !(adjacentCells->cells[cell].blocked) &&
			 (adjacentCells->cells[cell].dir != ((maze->mouse_dir + 2) % 4)))
		{
			best_cell_ind = cell;
			best_value = maze->distances[adjacentCells->cells[cell].pos.y][adjacentCells->cells[cell].pos.x];
		}
		if (adjacentCells->cells[cell].dir == ((maze->mouse_dir + 2) % 4))
		{
			turn_around_ind = cell;
		}
	}

	if (best_value != maze->distances[maze->mouse_pos.y][maze->mouse_pos.x])
	{
		best_cell.blocked = adjacentCells->cells[best_cell_ind].blocked;
		best_cell.dir = adjacentCells->cells[best_cell_ind].dir;
		best_cell.pos = adjacentCells->cells[best_cell_ind].pos;
	}
	else
	{
		best_cell.blocked = adjacentCells->cells[turn_around_ind].blocked;
		best_cell.dir = adjacentCells->cells[turn_around_ind].dir;
		best_cell.pos = adjacentCells->cells[turn_around_ind].pos;
	}

	free(adjacentCells->cells);
	free(adjacentCells);


    return best_cell;
}
