/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pso_set.h"

#include <vector>

using namespace std;

Pso_set::Pso_set()
{
}
Pso_set::Pso_set(float originX, float originY, float resolution, int width, int height, unsigned char *costs)
{
    setoriginX(originX);
    setoriginY(originY);
    setresolution(resolution);
    setwidth(width);
    setheight(height);
    setCosts(costs);
}

Pso_set::~Pso_set()
{
}
/**********************************************************/
//Function: Mutators and Accessors
/**********************************************************/

void Pso_set::setoriginX(float originx)
{
    originX = originx;
}
float Pso_set::getoriginX()
{
    return originX;
}
void Pso_set::setoriginY(float originy)
{
    originY = originy;
}
float Pso_set::getoriginY()
{
    return originY;
}
void Pso_set::setresolution(float Resolution)
{
    resolution = Resolution;
}
float Pso_set::getresolution()
{
    return resolution;
}
void Pso_set::setwidth(int Width)
{
    width = Width;
}
int Pso_set::getwidth()
{
    return width;
}
void Pso_set::setheight(int Height)
{
    height = Height;
}
int Pso_set::getheight()
{
    return height;
}
void Pso_set::setCosts(unsigned char *Costs)
{
    costs = Costs;
}
unsigned char *Pso_set::getCosts()
{
    return costs;
}
