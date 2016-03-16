#include "cocobot/pathfinder_table.h"
#include <string.h>
#include <math.h>

void initialiseTable(cocobot_node_s _table[][TABLE_WIDTH/GRID_SIZE], int _length, int _width)
{
    memset(_table, 0, _length * sizeof(cocobot_node_s));
    for(int i = 0; i < _length; i++)
    {
        memset(_table, 0, _length * sizeof(cocobot_node_s));
        for(int j = 0; j < _width; j++)
        {
            _table[i][j].x = i;
            _table[i][j].y = j;
        }
    }

    // ARound the dune
    setRectangle(_table, 2/GRID_SIZE, 20/GRID_SIZE, 80/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);
    setRectangle(_table, 2/GRID_SIZE, 20/GRID_SIZE, 220/GRID_SIZE, 0/GRID_SIZE, OBSTACLE);
    
    //Central T (construction area)
    setRectangle(_table, 120/GRID_SIZE, 2/GRID_SIZE, 90/GRID_SIZE, 75/GRID_SIZE, OBSTACLE);
    setRectangle(_table, 6/GRID_SIZE, 60/GRID_SIZE, 147/GRID_SIZE, 75/GRID_SIZE, OBSTACLE);
    
    //iles
    setCircle(_table, 0/GRID_SIZE, 199/GRID_SIZE, 25/GRID_SIZE, OBSTACLE); 
    setCircle(_table, 299/GRID_SIZE, 199/GRID_SIZE, 25/GRID_SIZE, OBSTACLE); 
}

void setPoint(cocobot_node_s _p_table[][TABLE_WIDTH/GRID_SIZE], int _x, int _y, cocobot_nodeType_e _nodeType)
{
    //Security to avoid buffer overflow
    //TODO : Change Hard coded Value
    if((_x < TABLE_LENGTH/GRID_SIZE) && (_y < TABLE_WIDTH/GRID_SIZE) && (_x >= 0) && (_y >= 0))
        _p_table[_x][_y].nodeType = _nodeType;
    //printf("x:%d, y:%d\n", _x, _y);
}

void setRectangle(cocobot_node_s _p_table[][TABLE_WIDTH/GRID_SIZE], int _xDimension, int _yDimension, int _xPosition, int _yPosition, cocobot_nodeType_e _nodeType)
{
    if(_xDimension == 0)
        _xDimension = 1;
    if(_yDimension == 0)
        _yDimension = 1;

    for(int x = _xPosition; x < _xDimension + _xPosition; x++)
    {
        for(int y = _yPosition ; y < _yDimension + _yPosition; y++)
        {
            setPoint(_p_table, x, y, _nodeType);
        }   
    }
}

void setCircle(cocobot_node_s _p_table[][TABLE_WIDTH/GRID_SIZE], int _xCenter, int _yCenter, int _radius, cocobot_nodeType_e _nodeType)
{
    if(_radius == 0)
        _radius = 1;

    double xMax = 0.0;
    for(int y = _yCenter; y < _yCenter + _radius; y++)
    {
        xMax = sqrt((float)_radius*(float)_radius - (float)(y-_yCenter)*(float)(y-_yCenter));
        //printf("x= %d, y= %d\n", (int)round(xMax), y);
        setRectangle(_p_table, (int)round(xMax), 1, _xCenter, y, _nodeType);
        setRectangle(_p_table, (int)round(xMax), 1, _xCenter - (int)round(xMax) , y, _nodeType);
        setRectangle(_p_table, (int)round(xMax), 1, _xCenter, 2*_yCenter - y - 1, _nodeType);
        setRectangle(_p_table, (int)round(xMax), 1, _xCenter - (int)round(xMax), 2*_yCenter - y - 1, _nodeType);
    }
}
