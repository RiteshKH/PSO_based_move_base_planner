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

#ifndef PSO_SET_H
#define PSO_SET_H

#include <ros/ros.h>
#include <vector>
#define POT_HIGH 1.0e10 // unassigned cell potential

using namespace std;
/**
 * @class Ant
 * @brief A class that implements ----
 */
class Pso_set
{
public:
    /**
	* @brief Constructor.
	*/
    Pso_set();
    Pso_set(int nx, int ny) : unknown_(true), lethal_cost_(253), neutral_cost_(50), factor_(3.0)
    {
        setSize(nx, ny);
    }

    Pso_set(float originX, float originY, float resolution, int width, int height, unsigned char *costs);
    /**
	* @brief Destructor.
	*/
    ~Pso_set();

    // Defining Getters and Setters
    void setoriginX(float originx);
    float getoriginX();
    void setoriginY(float originy);
    float getoriginY();
    void setresolution(float Resolution);
    float getresolution();
    void setwidth(int Width);
    int getwidth();
    void setheight(int Height);
    int getheight();
    void setCosts(unsigned char *Costs);
    unsigned char *getCosts();

    virtual void setSize(int nx, int ny)
    {
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;
    } /**< sets or resets the size of the map */
    void setLethalCost(unsigned char lethal_cost)
    {
        lethal_cost_ = lethal_cost;
    }
    void setNeutralCost(unsigned char neutral_cost)
    {
        neutral_cost_ = neutral_cost;
    }
    void setFactor(float factor)
    {
        factor_ = factor;
    }
    void setHasUnknown(bool unknown)
    {
        unknown_ = unknown;
    }

protected:
    inline int toIndex(int x, int y)
    {
        return x + nx_ * y;
    }
    int nx_, ny_, ns_; /**< size of grid, in pixels */
    bool unknown_;
    unsigned char lethal_cost_, neutral_cost_;
    float factor_;

private:
    float originX;
    float originY;
    float resolution;
    int width;
    int height;
    unsigned char *costs;
};

#endif
