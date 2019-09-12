//============================================================================
// Name        : PSO.cpp(Ubuntu_18_Ritesh)
// Copyright   : Your copyright notice
// Description : Particle Swarm Implementation in C++, Ansi-style Header file
//============================================================================

#ifndef PSO_H
#define PSO_H

#include <iostream>
#include <random>
#include <time.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <string>

#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include <ros/ros.h>
#include "pso_set.h"
// #include "../../src/global_planner/expander.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
// const int PARTICLE_COUNT = 5;

// Structure to store positions and velocities
struct Coord
{
    double x;
    double y;
    double velx;
    double vely;
};

/**
 * @class PSO_depend
 * @brief new class that implements -------------
 */
class PSO_depend : public Pso_set
{
public:
    /**
	* @brief Constructor.
	*/
    PSO_depend();
    PSO_depend(ros::NodeHandle &nh);
    ros::NodeHandle nh, g_nh;

    PSO_depend(float originX, float originY, float resolution, int width, int height, unsigned char *costs);

    ~PSO_depend();

    // Function declarations
    vector<int> PsoPath(double start_x, double start_y, double end_x, double end_y, vector<vector<double>> &particle_pos_x, vector<vector<double>> &particle_pos_y);
    void incomingMap(const nav_msgs::OccupancyGridConstPtr &new_map);

    int getCellIndex(int i, int j) //get the index of the cell to be used in Path
    {
        return (i * getwidth()) + j;
    }
    int getCellRowID(int index) //get the row ID from cell index
    {
        return index / getwidth();
    }
    int getCellColID(int index) //get colunm ID from cell index
    {
        return index % getwidth();
    }
    void getCoordinate(float x, float y)
    {
        x = x - getoriginX();
        y = y - getoriginY();
    }
    int convertToCellIndex(float x, float y)
    {
        int cellIndex;
        float newX = x / getresolution();
        float newY = y / getresolution();
        cellIndex = getCellIndex(newY, newX);
        return cellIndex;
    }

    void convertToCoordinate(int index, float x, float y)
    {
        x = getCellColID(index) * getresolution();
        y = getCellRowID(index) * getresolution();
        x = x + getoriginX();
        y = y + getoriginY();
    }
    // bool isFree(int CellID)
    // {
    //     return OGM[CellID];
    // }
    // vector<int> findFreeNeighborCell(int CellID)
    // {
    //     int rowID = getCellRowID(CellID);
    //     int colID = getCellColID(CellID);
    //     int neighborIndex;
    //     vector<int> freeNeighborCells;

    //     for (int i = -1; i <= 1; i++)
    //         for (int j = -1; j <= 1; j++)
    //         {
    //             //check whether the index is valid
    //             if ((rowID + i >= 0) && (rowID + i < height) && (colID + j >= 0) && (colID + j < width) && (!(i == 0 && j == 0)))
    //             {
    //                 neighborIndex = getCellIndex(rowID + i, colID + j);
    //                 if (isFree(neighborIndex))
    //                     freeNeighborCells.push_back(neighborIndex);
    //             }
    //         }
    //     return freeNeighborCells;
    // }

    // Generate random numbers
    double *generate_random(int swarm_size, double lower_bound, double upper_bound, int i)
    {
        srand(time(NULL));
        // std::default_random_engine re;
        std::mt19937 re;
        // initialize the random number generator with time-dependent seed
        uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count() + i++;
        std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
        re.seed(ss);

        // double lower_bound = -500;
        // double upper_bound = 500;
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);

        double *a_random_array = new double[swarm_size];
        const int nSimulations = swarm_size;
        for (int i = 0; i < nSimulations; i++)
        {
            double a_random_double = unif(re);
            a_random_array[i] = a_random_double;
        }
        return a_random_array;
    }

    // Find distance
    // double fitness(Coord input, Coord goal)
    // {
    //     double distance;
    //     distance = sqrt(pow((int)input.x - (int)goal.x, 2.0) + pow((int)input.y - (int)goal.y, 2.0));
    //     return distance;
    // }

    //============================================================================================================================//
    // Implementing Fitness function update from "Multi-Objective PSO-based Algorithm for Robot Path Planning"
    // Find shortest path
    double shortfit(Coord input, Coord goal)
    {
        double distance;
        // cout << "input (x,y) : " << input.x << "   " << input.y << "  goal (x, y):" << goal.x << " " << goal.y << endl;
        distance = sqrt(pow((int)input.x - (int)goal.x, 2.0) + pow((int)input.y - (int)goal.y, 2.0));
        // cout << "distance/fitness: " << distance << endl;
        return distance;
    }
    // Find smoothest path
    double smoothfit(Coord input, Coord target, Coord gbest)
    {
        double num = ((input.x - target.x) * (gbest.x - target.x)) + ((input.y - target.y) * (gbest.y - target.y));
        double den = sqrt(pow((input.x - target.x), 2.0) + pow((input.y - target.y), 2.0)) * sqrt(pow((gbest.x - target.x), 2.0) + pow((gbest.y - target.y), 2.0));
        double smooth = acos(num / den);
        return smooth;
    }
    // Overall fitness function
    double fitness(Coord input, Coord target, Coord gbest)
    {
        double distance = shortfit(input, target);
        double smooth = smoothfit(input, target, gbest);
        double alpha1 = 1.0, alpha2 = 0.0;
        double total_fitness = alpha1 * distance + alpha2 * smooth;
        return total_fitness;
    }

    // Implementing fitness function from "Optimization of Dynamic Mobile Robot Path Planning based on Evolutionary Methods"
    double angtheta(Coord input1, Coord input0)
    {
        double theta = atan((input1.y - input0.y) / (input1.x - input0.x));
        return theta;
    }
    // shortest distance from obstacle
    double obsdistance(Coord input1, Coord input0, double center_x, double center_y, double radius)
    {
        double slope = (input1.y - input0.y) / (input1.x - input0.x);
        double num = fabs((slope * center_x) - center_y + input0.y - (slope * input0.x));
        double den = sqrt(1 + pow(slope, 2.0));
        double obsDist = ((num / den) - radius);
        return obsDist;
    }
    // Overall fitness function
    double fitness2(Coord input1, Coord input0, double center_x[], double center_y[], double radius, int num_obstacle)
    {
        double inobstacle[num_obstacle];
        double fsemi = 0, thresh = 0.5, C = 5;
        double alpha = 0.4, beta = 0.2, gamma = 0.5;
        for (int k = 0; k < num_obstacle; k++)
        {
            inobstacle[k] = obsdistance(input1, input0, center_x[k], center_y[k], radius);
            if (inobstacle[k] <= (radius + thresh))
                fsemi = C;
        }
        double ffeas = alpha * shortfit(input1, input0) + beta * angtheta(input1, input0) + gamma * fsemi;

        return ffeas;
    }

    Coord newparticle(Coord input1, Coord input0, double center_x, double center_y, double radius, int seed)
    {
        double thresh = 0.5;
        Coord newPart1, newPart2, newPartFinal;
        double slope = (input1.y - input0.y) / (input1.x - input0.x);
        double invslope = -1 * (1 / slope);
        double obs = obsdistance(input1, input0, center_x, center_y, radius);
        double rn = generate_random(1, 10, 100, seed)[0];
        double safedistance = radius + (rn * thresh);

        double newX1 = (safedistance / sqrt(1 + pow(invslope, 2.0))) + center_x;
        double newY1 = ((safedistance * invslope) / sqrt(1 + pow(invslope, 2.0))) + center_y;
        newPart1.x = newX1;
        newPart1.y = newY1;

        double newX2 = -1 * (safedistance / sqrt(1 + pow(invslope, 2.0))) + center_x;
        double newY2 = -1 * ((safedistance * invslope) / sqrt(1 + pow(invslope, 2.0))) + center_y;
        newPart2.x = newX2;
        newPart2.y = newY2;
        if (shortfit(input0, newPart1) <= shortfit(input0, newPart2))
        {
            newPartFinal = newPart1;
        }
        else
        {
            newPartFinal = newPart2;
        }
        return newPartFinal;
    }
    //===========================================================================================================================//

    // Write to csv
    void write_to_csv(vector<vector<double>> particle_pos_x, vector<vector<double>> particle_pos_y, vector<Coord> global_pos,
                      vector<vector<double>> local_pos_x, vector<vector<double>> local_pos_y, int swarm_size, int counter,
                      double center_x[], double center_y[], double r, double side, double x1[], double y1[], int num_obstacles, vector<Coord> path_coord)
    {
        // create an ofstream for the file output (see the link on streams for
        // more info)
        std::ofstream outputFile;
        ofstream fs;
        // create a name for the file output
        string filename = "graph_data.csv";
        // create and open the .csv file
        outputFile.open(filename.c_str());

        // write data to the file
        // Columns: particles
        // Rows: iterations
        for (int i = 0; i < swarm_size; i++)
        {
            outputFile << "Particle_x" << i + 1 << ","
                       << "Particle_y" << i + 1 << ",";
        }
        outputFile << "global_position_x"
                   << ","
                   << "global_position_y"
                   << ",";
        outputFile << "path_x"
                   << ","
                   << "path_y"
                   << ",";
        for (int i = 0; i < swarm_size; i++)
        {
            outputFile << "local_position_x" << i + 1 << ","
                       << "local_position_y" << i + 1 << ",";
        }
        outputFile << "num_obstacles"
                   << ",";
        for (int i = 0; i < num_obstacles; i++)
        {
            outputFile << "center_x" << i + 1 << ","
                       << "center_y" << i + 1 << ","
                       << "r"
                       << ","
                       << "side"
                       << ","
                       << "x1" << i + 1 << ","
                       << "y1" << i + 1 << ",";
        }

        outputFile << "\n";

        for (int iter = 1; iter <= counter; iter++)
        {
            for (int i = 0; i < swarm_size; i++)
            {
                outputFile << particle_pos_x[iter][i] << "," << particle_pos_y[iter][i] << ",";
            }
            if (iter == 1)
            {
                outputFile << global_pos[0].x << "," << global_pos[0].x << ",";
            }
            else
            {
                outputFile << global_pos[iter].x << "," << global_pos[iter].y << ",";
            }
            // if (iter < k)
            // {
            outputFile << path_coord[iter].x << "," << path_coord[iter].y << ",";
            // }
            // else
            // {
            //     outputFile << ""
            //                << ","
            //                << ""
            //                << ",";
            // }

            for (int i = 0; i < swarm_size; i++)
            {
                outputFile << local_pos_x[iter][i] << "," << local_pos_y[iter][i] << ",";
            }
            outputFile << num_obstacles << ",";
            for (int i = 0; i < num_obstacles; i++)
            {
                outputFile << center_x[i]
                           << ","
                           << center_y[i]
                           << ","
                           << r
                           << ","
                           << side
                           << ","
                           << x1[i]
                           << ","
                           << y1[i]
                           << ",";
            }

            outputFile << "\n";
        }

        // close the output file
        outputFile.close();
        cout << "Values written to csv file" << endl;
    }

    //Function to print all the elements of the input array
    void printArray(double input[], int size)
    {
        for (int i = 0; i < size; ++i)
        {
            cout << input[i] << " ";
        }
        cout << endl;
    }

    // Display at each iteration
    void epochDisplay(int iter_ctr, double global_best_fit, Coord global_best_pos)
    {
        cout << "Iteration :" << iter_ctr << ".........." << endl;
        cout << "New Global Best fitness: " << global_best_fit << endl;
        cout << "New Global Best Position: (" << global_best_pos.x << "," << global_best_pos.y << ")" << endl;
    }

    // ========================================================================================================================================//
    // Implementing obstacles

    // Circular obstacle
    bool inCircle(double x, double y, double center_x, double center_y, double r)
    {
        double radius = sqrt(pow(x - center_x, 2.0) + pow(y - center_y, 2.0));
        if (radius <= r)
            return true;
        else
            return false;
    }

    // Square and rectangular obstacle
    float area(int x1, int y1, int x2, int y2, int x3, int y3)
    {
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1) +
                    x3 * (y1 - y2)) /
                   2.0);
    }

    bool inSquare(double x, double y, double x1, double y1, double side)
    {
        double x2 = x1 + side;
        double y2 = y1;
        double x3 = x1 + side;
        double y3 = y1 + side;
        double x4 = x1;
        double y4 = y1 + side;

        /* Calculate area of rectangle ABCD */
        double A = area(x1, y1, x2, y2, x3, y3) +
                   area(x1, y1, x4, y4, x3, y3);

        /* Calculate area of triangle PAB */
        double A1 = area(x, y, x1, y1, x2, y2);

        /* Calculate area of triangle PBC */
        double A2 = area(x, y, x2, y2, x3, y3);

        /* Calculate area of triangle PCD */
        double A3 = area(x, y, x3, y3, x4, y4);

        /* Calculate area of triangle PAD */
        double A4 = area(x, y, x1, y1, x4, y4);

        /* Check if sum of A1, A2, A3 and A4  
       is same as A */
        if (A == (A1 + A2 + A3 + A4))
            return true;
        else
            return false;
    }

private:
    float originX;
    float originY;
    float resolution;
    unsigned char *costs;
    int width;
    int height;
};
#endif
