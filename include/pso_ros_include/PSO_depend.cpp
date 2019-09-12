//============================================================================
// Name        : PSO.cpp(Ubuntu_18_Ritesh)
// Copyright   : Your copyright notice
// Description : Particle Swarm Implementation in C++, Ansi-style
//============================================================================

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
#include <array>
#include "PSO_depend.h"
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/cost_values.h>

using namespace std;

/**********************************************************/
//Constructors and destructors
/**********************************************************/
PSO_depend::PSO_depend()
{
}
PSO_depend::PSO_depend(float originX, float originY, float resolution, int width, int height, unsigned char *costs) : Pso_set(originX, originY, resolution, width, height, costs)
{
}
PSO_depend::~PSO_depend()
{
}

//callback
void PSO_depend::incomingMap(const nav_msgs::OccupancyGridConstPtr &new_map)
{
    unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
    float res = new_map->info.resolution;
    // cout << "Actual width: " << size_x * res << endl;
}

//===============================================================================================================================================//
vector<int> PSO_depend::PsoPath(double start_x, double start_y, double end_x, double end_y, vector<vector<double>> &particle_pos_x, vector<vector<double>> &particle_pos_y)
{
    Coord start;
    Coord target; // Goal to reach

    start.x = start_x;
    start.y = start_y;
    target.x = end_x;
    target.y = end_y;
    //============================================================================//
    //Experimental
    // to get actual width and height
    std::string map_topic;
    nh.param("map_topic", map_topic, std::string("map"));
    ros::Subscriber map_sub_;
    map_sub_ = g_nh.subscribe(map_topic, 1, &PSO_depend::incomingMap, this);

    //============================================================================//

    //Initial and Control Parameters
    int swarm_size = 30;   //Swarm Size
    int no_of_iters = 200; //Number of Iterations
    double w;              //Inertial Weight
    double wmin = 0.4;
    double wmax = 0.9;
    double c1 = 2, c2 = 2; //acceleration coefficients
    //Variables Initialization
    //Need to be randomly generated
    //These can be changing for every iteration

    // ========================================================================================================================== //
    //Random Numbers used for updating position and velocity of particles
    // Positions // Velocities

    int seed = 1; // Variable to induce different timeseeds in each call for different random number generation
    // Upper and lower bound should not be less than 10, else values after decimal point is not considered.
    double lower_bound = -1;
    double upper_bound = 1;

    double *position_x = generate_random(swarm_size, lower_bound, upper_bound, seed++);
    double *position_y = generate_random(swarm_size, lower_bound, upper_bound, seed++);
    double *velocity_x = generate_random(swarm_size, lower_bound, upper_bound, seed++);
    double *velocity_y = generate_random(swarm_size, lower_bound, upper_bound, seed++);

    // Structure to store random data for position and velocities
    Coord States[swarm_size];
    for (int i = 0; i < swarm_size; ++i)
    {
        States[i].x = position_x[i];
        States[i].y = position_y[i];
        States[i].velx = velocity_x[i];
        States[i].vely = velocity_y[i];
        // cout << "Random numbers (x,y) : " << States[i].x << "   " << States[i].y << "   (velx, vely):" << States[i].velx << " " << States[i].vely << endl;
    }

    // ========================================================================================================================== //

    // New Initializations
    Coord particle_states[swarm_size];   //position and velocities of particles
    double particle_fitness[swarm_size]; //fitness of particles
    Coord local_best_pos[swarm_size];    //local best position of each particle across iterations
    double local_best_fit[swarm_size];   //local best fitness of each particle across iterations
    Coord global_best_pos;               //global best position across all particles across iterations
    double global_best_fit;
    // Boundary conditons for particle initialisations
    double lower_boundary = (-1 * (getoriginX() / getresolution())) - 0.5 * (getwidth() / 7);
    double upper_boundary = (-1 * (getoriginX() / getresolution())) + 0.5 * (getwidth() / 7);

    // double lower_boundary = 0.5 * (getwidth() - (getwidth() / 7));
    // double upper_boundary = (lower_boundary + (getwidth() / 7));

    //Initialize all arrays to 0
    global_best_pos.x = start.x;
    global_best_pos.y = start.y;

    int cols = swarm_size + 1;
    int rows = no_of_iters + 1;
    int initial_value = 0;
    unsigned char lethal_cost_ = costmap_2d::LETHAL_OBSTACLE;
    unsigned char free_cost_ = costmap_2d::FREE_SPACE;

    int counter = 1;
    int flag = 0;
    vector<Coord> global_pos(rows);
    global_pos[0].x = global_best_pos.x;
    global_pos[0].y = global_best_pos.y;

    // vector<vector<double>> particle_pos_x;
    particle_pos_x.resize(rows, vector<double>(cols, initial_value));
    // vector<vector<double>> particle_pos_y;
    particle_pos_y.resize(rows, vector<double>(cols, initial_value));

    vector<Coord> path_coord;

    vector<vector<double>> local_pos_x;
    local_pos_x.resize(rows, vector<double>(cols, initial_value));
    vector<vector<double>> local_pos_y;
    local_pos_y.resize(rows, vector<double>(cols, initial_value));

    const double V_max = (upper_boundary - lower_boundary) / 10;
    global_best_fit = fitness(global_best_pos, target, global_pos[0]);
    double pos_multiple = (upper_boundary - lower_boundary);
    double vel_multiple = (upper_boundary - lower_boundary) / 20;

    // ==================================================================================//
    // Initialize obstacle parameters (Obstacles at random positions)
    // const int num_obstacle = 5;
    // bool incircle[num_obstacle];
    // bool insquare[num_obstacle];
    // double center_x[num_obstacle];
    // double center_y[num_obstacle];
    // double x1[num_obstacle];
    // double y1[num_obstacle];
    // double r = 50.0;
    // double side = 100.0;
    // for (int i = 0; i < num_obstacle; i++)
    // {
    //     center_x[i] = (pos_multiple - 200) * (generate_random(1, -1, 1, seed++)[0]);
    //     center_y[i] = (pos_multiple - 200) * (generate_random(1, -1, 1, seed++)[0]);
    //     x1[i] = (pos_multiple - 200) * (generate_random(1, -1, 1, seed++)[0]);
    //     y1[i] = (pos_multiple - 200) * (generate_random(1, -1, 1, seed++)[0]);
    //     // while ((center_x[i] == target.x && center_y[i] == target.y) || (x1[i] == target.x && y1[i] == target.y) || (center_x[i] == start.x && center_y[i] == start.y) || (x1[i] == start.x && y1[i] == start.y))
    // }
    // double center_x[num_obstacle] = {-300, -300, 150, 0, 0};
    // double center_y[num_obstacle] = {-100, 200, 150, -200, 200};
    // ================================================================================================================================================//

    /**
	 * First Iteration. Initialize swarm with Random Position and Velocity vectors for Each Particle.
	 * Different problems will have different solution space, hence initialization values will change
	 */
    for (int i = 0; i < swarm_size; i++)
    {
        // New
        // array<bool, num_obstacle> validinit;
        bool final_val_init = false;
        do
        {
            double ax = pos_multiple * (States[i].x);
            if (ax > 0)
                particle_states[i].x = (lower_boundary + pos_multiple / 2) + ax;
            else
                particle_states[i].x = (upper_boundary - pos_multiple / 2) + ax;

            double ay = pos_multiple * (States[i].y);
            if (ay > 0)
                particle_states[i].y = (lower_boundary + pos_multiple / 2) + ay;
            else
                particle_states[i].y = (upper_boundary - pos_multiple / 2) + ay;

            // particle_states[i].x = pos_multiple * (States[i].x);
            // particle_states[i].y = pos_multiple * (States[i].y);
            double new_x = generate_random(1, 0, 1, seed++)[0];
            States[i].x = new_x;
            double new_y = generate_random(1, 0, 1, seed++)[0];
            States[i].y = new_y;

            //Obstacle check::
            // int next_i = toIndex(particle_states[i].x, particle_states[i].y);
            // if (costs[next_i] >= lethal_cost_ && !(unknown_ && costs[next_i] == costmap_2d::NO_INFORMATION))
            // {
            //     cout << "Initial point obstacle" << endl;
            //     final_val_init = true;
            // }

        } while (final_val_init);

        particle_states[i].velx = vel_multiple * (States[i].x);
        particle_states[i].vely = vel_multiple * (States[i].y);

        // Representing fitness function as the distance between the particles and local best
        particle_fitness[i] = fitness(particle_states[i], target, global_pos[0]);
        local_best_pos[i].x = particle_states[i].x;
        local_best_pos[i].y = particle_states[i].y;
        local_best_fit[i] = particle_fitness[i];

        // Write to csv data
        local_pos_x[1][i] = local_best_pos[i].x;
        local_pos_y[1][i] = local_best_pos[i].y;

        if (local_best_fit[i] < global_best_fit)
        {
            global_best_pos.x = particle_states[i].x;
            global_best_pos.y = particle_states[i].y;
            global_best_fit = local_best_fit[i];
        }

        // Write to csv data
        global_pos[1].x = global_best_pos.x;
        global_pos[1].y = global_best_pos.y;
        // Write to csv data
        particle_pos_x[1][i] = particle_states[i].x;
        particle_pos_y[1][i] = particle_states[i].y;
    }

    // Initial Display
    epochDisplay(1, global_best_fit, global_best_pos);

    /**
	 * Calculations from Second iteration Onwards
	 */

    for (int iter_ctr = 2; iter_ctr <= no_of_iters; iter_ctr++)
    {
        //Updating Velocity - Applying SocialPsycho Criteria
        for (int i = 0; i < swarm_size; i++)
        {
            // array<bool, num_obstacle> valid;
            bool final_val = false;
            int ct = 0;
            do
            {
                // New
                double rp = generate_random(1, 0, 1, seed++)[0];
                double rg = generate_random(1, 0, 1, seed++)[0];
                // cout << "R1 and R2: " << r1 <<"   "<< r2 << endl;
                // w = wmax - (((wmax - wmin) / (no_of_iters)) * iter_ctr);
                // following update from "Intelligent Vehicle Global Path Planning Based on Improved Particle Swarm Optimization"
                double c = 2.3;
                w = wmin + (wmax - wmin) * exp(-pow(((c * iter_ctr) / no_of_iters), 2.0));
                // cout << "w value : " << w << endl;
                particle_states[i].velx = (w * particle_states[i].velx + c1 * rp * (local_best_pos[i].x - particle_states[i].x)) + c2 * rg * (global_best_pos.x - particle_states[i].x);
                particle_states[i].vely = (w * particle_states[i].vely + c1 * rp * (local_best_pos[i].y - particle_states[i].y)) + c2 * rg * (global_best_pos.y - particle_states[i].y);
                if (particle_states[i].velx > V_max)
                    particle_states[i].velx = V_max;
                if (particle_states[i].vely > V_max)
                    particle_states[i].vely = V_max;
                if (particle_states[i].velx < -V_max)
                    particle_states[i].velx = -V_max;
                if (particle_states[i].vely < -V_max)
                    particle_states[i].vely = -V_max;

                // Calculating resultant velocity and orientation for real world simulation
                // double res_vel = sqrt(pow(particle_states[i].velx, 2.0) + pow(particle_states[i].vely, 2.0));
                // double theta = atan(particle_states[i].vely / particle_states[i].velx);

                particle_states[i].x = particle_states[i].x + particle_states[i].velx;
                particle_states[i].y = particle_states[i].y + particle_states[i].vely;
                if (particle_states[i].x > upper_boundary)
                    particle_states[i].x = upper_boundary;
                if (particle_states[i].y > upper_boundary)
                    particle_states[i].y = upper_boundary;
                if (particle_states[i].x < lower_boundary)
                    particle_states[i].x = lower_boundary;
                if (particle_states[i].y < lower_boundary)
                    particle_states[i].y = lower_boundary;

                // ==============================================================================================================================================================//

                // COSTMAP OBSTACLE CHECK::
                // for (int i = 0; i < getwidth(); i++)
                // {
                //     cout << +getCosts()[i] << " ";
                // }
                // int next_i = toIndex(particle_states[i].x, particle_states[i].y);
                // cout << "problem here: toIndex: " << next_i << endl;
                // cout << "cost: lethal_cost: unknown:" << endl;
                // cout << getCosts()[next_i] << "," << endl;
                // cout << lethal_cost_ << "," << endl;
                // cout << unknown_ << endl;
                // if (getCosts()[next_i] >= lethal_cost_ && !(unknown_ && getCosts()[next_i] == costmap_2d::NO_INFORMATION))
                // {
                //     cout << "In obstacle" << endl;
                //     final_val = true;
                // }

                //     // ==============================================================================================================================================================//
                ct = ct + 1;

                //Check if particles are within boundary
                // if (particle_states[i].x > lower_boundary && particle_states[i].x < upper_boundary && particle_states[i].y > lower_boundary && particle_states[i].y < upper_boundary)
                //     final_val = false;
                // else
                //     final_val = true;

            } while (final_val && (ct < 1000));

            // Write to csv data
            particle_pos_x[iter_ctr][i] = particle_states[i].x;
            particle_pos_y[iter_ctr][i] = particle_states[i].y;

            // Write to csv data
            global_best_pos.velx = particle_states[i].velx;
            global_best_pos.vely = particle_states[i].vely;
        }

        //Assign Local & Global Best position & Fitness
        //For each particle evaluate fitness
        //If fitness(xt) > fitness(gbest) then gbest=xt
        //If fitness(xt) > fitness(pbest) then pbest=xt
        for (int i = 0; i < swarm_size; i++)
        {
            //Set Particle Fitness
            particle_fitness[i] = fitness(particle_states[i], target, global_pos[iter_ctr - 1]);

            // Updating local/particle best position
            if (fitness(particle_states[i], target, global_pos[iter_ctr - 1]) < local_best_fit[i])
            {
                local_best_pos[i].x = particle_states[i].x;
                local_best_pos[i].y = particle_states[i].y;
                // local_best_fit[i] = fitness(local_best_pos[i], global_best_pos, global_pos[iter_ctr - 1]);
                // Updating global best position
                if (fitness(local_best_pos[i], target, global_pos[iter_ctr - 1]) < global_best_fit)
                {
                    global_best_pos.x = local_best_pos[i].x;
                    global_best_pos.y = local_best_pos[i].y;
                    global_best_fit = fitness(global_best_pos, target, global_pos[iter_ctr - 1]);
                }
                else
                {
                    double alpha = 0.01, beta = 0.02; // Arbitrarily generated constants
                    double r3 = generate_random(1, 0, 1, seed++)[0];
                    global_best_pos.velx = global_best_pos.x - particle_states[i].x + (alpha * global_best_pos.velx) + (beta * r3);
                    global_best_pos.vely = global_best_pos.y - particle_states[i].y + (alpha * global_best_pos.vely) + (beta * r3);
                    particle_states[i].x = particle_states[i].x + global_best_pos.velx;
                    particle_states[i].y = particle_states[i].y + global_best_pos.vely;
                    if (particle_states[i].x > lower_boundary && particle_states[i].x < upper_boundary && particle_states[i].y > lower_boundary && particle_states[i].y < upper_boundary)
                    {
                        global_best_pos.x = particle_states[i].x;
                        global_best_pos.y = particle_states[i].y;
                        global_best_fit = fitness(global_best_pos, target, global_pos[iter_ctr - 1]);
                    }
                    // cout << "global distance outside/fitness: " << global_best_fit << endl;
                    // cout << "global positions outside/fitness: " << global_best_pos.x << "," << global_best_pos.y << endl;
                }
            }

            // Write to csv data
            local_pos_x[iter_ctr][i] = local_best_pos[i].x;
            local_pos_y[iter_ctr][i] = local_best_pos[i].y;

            // Write to csv data
            global_pos[iter_ctr].x = global_best_pos.x;
            global_pos[iter_ctr].y = global_best_pos.y;

            // Implementing Shortest Path Planning PSO
            // (Applies a slight perturbation to the Global best particles so that it doesn't get stuck in a local minima.
            //  The particles are updated by the given velocity formula)
            //Also Check if particles are within boundary
            bool final_val = false;
            int cs = 0;
            do
            {
                cs++;
                double alpha = 0.1, beta = 0.2; // Arbitrarily generated constants
                double r3 = generate_random(1, 0, 1, seed++)[0];
                global_best_pos.velx = global_best_pos.x - particle_states[i].x + (alpha * global_best_pos.velx) + (beta * r3);
                global_best_pos.vely = global_best_pos.y - particle_states[i].y + (alpha * global_best_pos.vely) + (beta * r3);
                particle_states[i].x = particle_states[i].x + global_best_pos.velx;
                particle_states[i].y = particle_states[i].y + global_best_pos.vely;
                if (particle_states[i].x > lower_boundary && particle_states[i].x < upper_boundary && particle_states[i].y > lower_boundary && particle_states[i].y < upper_boundary)
                    final_val = false;
                else
                    final_val = true;

            } while (final_val && cs <= 500);
        }

        // All Displays
        epochDisplay(iter_ctr, global_best_fit, global_best_pos);

        // Check if converged on goal
        counter++;
        if (shortfit(global_best_pos, target) <= 60.0)
        {
            cout << "\n\nReached near goal break. SUCCESS " << endl;
            break;
        }
        else if (iter_ctr >= (no_of_iters / 30))
        {
            if (((global_pos[iter_ctr].x - global_pos[iter_ctr - 1].x) < 0.1) && ((global_pos[iter_ctr].x - global_pos[iter_ctr - 2].x) < 0.1) && ((global_pos[iter_ctr].x - global_pos[iter_ctr - 3].x) < 0.1) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 1].y) < 0.1) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 2].y) < 0.1) && ((global_pos[iter_ctr].y - global_pos[iter_ctr - 3].y) < 0.1))
            {
                cout << "\n\nRepeatition break, Converged at local minima/maxima. FAILED" << endl;
                flag = 1;
                break;
            }
        }
    }

    // All Displays
    // display(swarm_size, particle_states, particle_fitness, global_best_fit, local_best_fit, local_best_pos, global_best_pos);

    // Finding Path: 1
    Coord path_cd;
    path_cd.x = start.x;
    path_cd.y = start.y;
    path_coord.insert(path_coord.begin() + path_coord.size(), path_cd);
    for (int iter_ctr = 1; iter_ctr <= counter; iter_ctr++)
    {
        if (iter_ctr > 3)
        {
            if ((shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr - 1], target)) &&
                (shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr - 2], target)) &&
                (shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr - 3], target)))
            {
                // path_coord[iter_ctr] = global_pos[iter_ctr];
                path_coord.insert(path_coord.begin() + path_coord.size(), global_pos[iter_ctr]);
            }
            else
            {
                // path_coord[iter_ctr] = global_pos[iter_ctr - 1];
                path_coord.insert(path_coord.begin() + path_coord.size(), global_pos[iter_ctr - 1]);
            }
        }
        else
        {
            // path_coord[iter_ctr] = global_pos[0];
            path_coord.insert(path_coord.begin() + path_coord.size(), global_pos[0]);
        }
    }

    cout << endl
         << "Final Global Best Position: (" << global_best_pos.x << "," << global_best_pos.y << ")" << endl;

    // Writing to csv all particle positions
    // write_to_csv(particle_pos_x, particle_pos_y, global_pos, local_pos_x, local_pos_x, swarm_size, counter, center_x, center_y, r, side, x1, y1, num_obstacle, path_coord);
    cout << "Compilation Done" << endl;

    // Finding Path: 2
    vector<int> path_idx;
    for (uint iter_ctr = 0; iter_ctr < path_coord.size(); iter_ctr++)
    {
        // cout << "(" << path_coord[iter_ctr].x << "," << path_coord[iter_ctr].y << ")" << endl;
        int idx = path_coord[iter_ctr].x;
        int idy = path_coord[iter_ctr].y;
        path_idx.insert(path_idx.begin() + path_idx.size(), idx);
        path_idx.insert(path_idx.begin() + path_idx.size(), idy);
    }
    path_idx.insert(path_idx.begin() + path_idx.size(), flag);
    return path_idx;
}

// Pseudo code
/*
 Start
 Initialize swarm with Random Position (x0) and velocity vectors (v0)
 for Each Particle
 Evaluate Fitness
 If fitness(xt) > fitness(gbest)
 gbest=xt
 If fitness(xt) > fitness(pbest)
 pbest=xt
 Update Velocity
 v(t+1)=W*vt + c1*rand(0,1)*(pbest-xt)+c2*rand(0,1)*gbest-xt)
 Update Position
 X(t+1) = Xt+V(t+1)

 Go to next particle

 If Terminate
 gbest is the output
 Else goto for Each Particle
 */
