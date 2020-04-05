
#include <iostream>
#include <cmath>
#include <vector>
#include <dlib/all/source.cpp>
#include "vehicle_model.h"
#include <chrono> 

using namespace dlib;
using namespace std;
using namespace std::chrono;


typedef matrix<double, 0, 1> column_vector;

double objective(const column_vector& m);

/* number of horizen point */
#define N 20

/* MPC cost function weights */ 
#define cte_W 2000
#define eth_W 2000
#define v_W 5000
#define st_rate_W  100
#define acc_rate_W 100
#define st_W  10
#define acc_W  10

/* refernce velocity */
#define v_ref 1

/* sample time */
#define dt 0.1

/* Base length */
#define Base_length 0.74

/* coff of refernce polynomial */
double coff[] = { 0.00866424, -0.22994552,  1.56638264, -0.04748659 };

state last_state_g;
state current_state_g;

int main()
{
    /* vector for X, Y, CTE, T for later analysis*/
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> CTE;
    std::vector<double> T;


    inputs act;

    /* init x solution */
    column_vector x = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    /* Upper bounds */
    column_vector ub = { 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43,
                         0.43, 0.43, 0.43, 0.43, 0.43, 0.43,0.43, 0.43, 0.43, 0.43,  
                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

    /* lower bounds */
    column_vector lb = { -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43,
                         -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43, -0.43,
                        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };


    /* simulation Time */
    int ns = 200;

    /* simulation loop */
    for (int i = 0; i < ns; i++)
    {
        /* save start time of optamization */
        auto start = high_resolution_clock::now();

        /* MPC local constrained optamization, the final solutions will be in "column_vector x" */
        find_min_box_constrained(bfgs_search_strategy(),
                                 objective_delta_stop_strategy(1e-6),
                                 objective, derivative(objective), x, lb, ub);

        /* save end time of optamization */
        auto stop = high_resolution_clock::now();

        /* time of optamization */
        auto duration = duration_cast<milliseconds>(stop - start);

        /* Applay output command inputs from optamization to the noisy model of vehicle */
        act.steerangle = x(0);
        act.accelartion = x(N);
        last_state_g = update_state_noise(act, last_state_g, coff, dt, Base_length);


        last_state_g.print_state();
        cout << "Time = " << duration.count() << " MS" << endl;
        cout << "Cost = " << objective(x) << endl;

        /* save data for later analysis */
        X.push_back(last_state_g.x);
        Y.push_back(last_state_g.y);
        
    }

    for (auto i = X.begin(); i != X.end(); ++i)
        cout << *i << ",";

    cout << "\n------------------------------------------------------------------------------------------------------" << endl;

    for (auto i = Y.begin(); i != Y.end(); ++i)
        cout << *i << ",";

    cout << "\n------------------------------------------------------------------------------------------------------" << endl;

    //cout << "Max CTE: "   << *max_element(CTE.begin(), CTE.end()) << endl;

    //cout << "Max value: " << *max_element(T.begin(), T.end()) << endl;


    return 0;
}


/* the cost function */
double objective(const column_vector& m)
{
    inputs u;
    state last_state = last_state_g;
    state current_state = current_state_g;
    double Error = 0;
    for (int i = 0; i < N; i++)
    {
        u.steerangle = m(i);
        u.accelartion = m(i + N);
        current_state = update_state(u, last_state, coff, dt, Base_length);
        if (i == 0)
        {
            Error += cte_W * pow(current_state.cte, 2) + eth_W * pow(current_state.eth, 2) + v_W * pow((v_ref - current_state.v), 2)
                + st_W * pow(u.steerangle, 2) + acc_W * pow(u.accelartion, 2);
        }
        else
        {
            Error += cte_W * pow(current_state.cte, 2) + eth_W * pow(current_state.eth, 2) + v_W * pow((v_ref - current_state.v), 2)
                + st_rate_W * pow(u.steerangle - m(i - 1), 2) + acc_rate_W * pow(u.accelartion - m(i + N - i), 2)
                + st_W * pow(u.steerangle, 2) + acc_W * pow(u.accelartion, 2);
        }


        last_state = current_state;
    }
    return Error;
}