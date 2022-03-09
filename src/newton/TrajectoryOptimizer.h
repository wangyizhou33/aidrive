#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#define TRAJECTORY_PLANNING_HORIZON_DEFAULT 3.0 // seconds

#define TRAJECTORY_OPTIMIZER_HISTORY_LENGTH_DEFAULT 3
#define TRAJECTORY_OPTIMIZER_UPPER_BANDWIDTH                                                                           \
    8 // up to jerk of coordinates, which is a filter with 4 taps times 2 for (x,y)

#include <iomanip>
#include <iostream>
using namespace std;
#include "DampedOptimizer.h"
#include "Frenet.h"
#include "ScalarFunctionWrappers.h"
#include "cost/ActorAvoidanceCost.h"
#include "cost/BumpCost.h"
#include "cost/LateralCost.h"
#include "cost/LimitCosts.h"
#include "cost/PriorCost.h"
#include "cost/ProgressCost.h"
#include "cost/SmoothnessCost.h"

// N is number of time steps.
// HL is history length (how many negative coordinate pairs of trajectory are valid).
// Only the N coordinate pairs from 0 to N-1 will be optimized, but history for HL pairs is needed.
// UB is upper bandwidth.
// FTT is the Frenet Transformation Type
// SWFT is the smootheness weight function type
// LBFT is the limit cost barrier function type
// BFT_? are the barrier function types for actor avoidance cost
// PWFT_LON,LAT are the prior trajectory weight function for longitudinal and lateral
// LAT_FT is the lateral function type for lateral costs
// LAT_WT is the lateral weight type for lateral costs
// BUMP_FT is the two-dimensional bump style function
// VEL_WT is the function of velocity to weigh the bumps by
template <class T, int N, int HL = TRAJECTORY_OPTIMIZER_HISTORY_LENGTH_DEFAULT,
          int UB = TRAJECTORY_OPTIMIZER_UPPER_BANDWIDTH,
          class FTT = frenetTransformationLateralFunction<T, scalarFunctionPolynomial<T, 3>>,
          class SWFT = scalarFunctionWeightedSquare<T>,
          class LBFT = scalarFunctionSquaredBarrier<T>, // class LBFT = scalarFunctionSmoothBarrier<T>, //class LBFT  =
                                                        // scalarFunctionSquaredBarrier<T>,//
          class BFT_F = scalarFunctionSquaredBarrier<T>, class BFT_B = scalarFunctionSquaredBarrier<T>,
          class BFT_L = scalarFunctionSquaredBarrier<T>, class BFT_R = scalarFunctionSquaredBarrier<T>,
          class PWFT_LON = scalarFunctionWeightedSquare<T>, class PWFT_LAT = scalarFunctionWeightedSquare<T>,
          class LAT_FT = scalarFunctionPolynomial<T, 3>,
          class LAT_WT = scalarFunctionWeightedSquare<T>, // scalarFunctionWeightedBump<T>
          class BUMP_FT = TwoDimensionalFunctionBump<T>, class VEL_WT = scalarFunctionWeightedSquare<T>>
class trajectoryOptimizer : public dampedOptimizerBanded<T, 2 * N, UB, 2 * HL>
{
  public:
    void setDefault(T planning_horizon_input = ((T)TRAJECTORY_PLANNING_HORIZON_DEFAULT),
                    T k_p_input = ((T)PROGRESS_WEIGHT_DEFAULT),
                    T set_point_velocity_in = ((T)ACTOR_AVOIDANCE_SET_POINT_VELOCITY_DEFAULT),
                    T set_point_distance_in = ((T)ACTOR_AVOIDANCE_SET_POINT_DISTANCE_DEFAULT),
                    T x_a_input = ((T)ACTOR_AVOIDANCE_LONGITUDINAL_ADD_DEFAULT),
                    T x_s_input = ((T)ACTOR_AVOIDANCE_LONGITUDINAL_SUB_DEFAULT),
                    T y_a_input = ((T)ACTOR_AVOIDANCE_LATERAL_ADD_DEFAULT),
                    T y_s_input = ((T)ACTOR_AVOIDANCE_LATERAL_SUB_DEFAULT))
    {
        int i;

        m_x = m_x_mem + (2 * HL);
        m_x_transformed = m_x_transformed_mem + (2 * HL);
        m_x_prior = m_x_prior_mem + (2 * HL);

        planning_horizon = planning_horizon_input;
        delta_t = ((T)planning_horizon) / ((T)N);
        delta_t_inv = ((T)1.0) / delta_t;

        progressP.setDefault(k_p_input);
        smoothP.setDefault(delta_t_inv);
        limitP.setDefault(delta_t_inv, k_p_input);
        // actorAvoidanceP.setDefaultByWeights(k_p_input, delta_t_inv,
        //	k_b_forward_input, k_b_backward_input, k_b_left_input, k_b_right_input,
        //	x_a_input, x_s_input, y_a_input, y_s_input);
        actorAvoidanceP.setDefaultBySetPoint(k_p_input, delta_t_inv, set_point_velocity_in, set_point_distance_in,
                                             x_a_input, x_s_input, y_a_input, y_s_input);

        priorP.setDefault();

        for (i = -HL; i < N; i++)
        {
            m_x[2 * i] = delta_t * ((T)20.0) * i; // x
            m_x[2 * i + 1] = ((T)0.0);            // y
        }

        m_progress_cost_on = true;
        m_smoothness_cost_on = true;
        m_limit_cost_on = true;
        m_actor_avoidance_cost_on = true;
        m_prior_cost_on = true;
        m_prior_exists = false;
        m_lateral_cost_on = true;
        m_LatFun = NULL;
        m_LatWeight = NULL;
        m_nrLaterals = 0;

        m_bump_cost_on = true;
        m_BumpFun = NULL;
        m_VelWeight = NULL;
        m_nrBumps = NULL;

        m_XC = NULL;
        m_nrXC = NULL;
    }

    trajectoryOptimizer()
    {
        setDefault();
    }

    int getN()
    {
        return (N);
    }
    int getHL()
    {
        return (HL);
    }

    void set_progress_cost(bool ison)
    {
        m_progress_cost_on = ison;
    }
    void set_smoothness_cost(bool ison)
    {
        m_smoothness_cost_on = ison;
    }
    void set_limit_cost(bool ison)
    {
        m_limit_cost_on = ison;
    }
    void set_actor_avoidance_cost(bool ison)
    {
        m_actor_avoidance_cost_on = ison;
    }
    void set_prior_cost(bool ison)
    {
        m_prior_cost_on = ison;
    }
    void set_lateral_cost(bool ison)
    {
        m_lateral_cost_on = ison;
    }
    void set_bump_cost(bool ison)
    {
        m_bump_cost_on = ison;
    }
    void set_all_off()
    {
        m_progress_cost_on = false;
        m_smoothness_cost_on = false;
        m_limit_cost_on = false;
        m_actor_avoidance_cost_on = false;
        m_prior_cost_on = false;
        m_lateral_cost_on = false;
        m_bump_cost_on = false;
    }

    virtual T computeCost(const T *x)
    {
        int i;
        T acc;

        // Zero out
        acc = ((T)0.0);

        //-------------Frenet transformed cost first
        for (i = -HL; i < N; i++)
            m_frenet_transformer.transformPoint(&m_x_transformed[2 * i], &x[2 * i]);

        // Actor Avoidance Costs
        if (m_actor_avoidance_cost_on && m_XC != NULL && m_nrXC != NULL)
        {
            acc += actorAvoidanceCost<T, BFT_F, BFT_B, BFT_L, BFT_R>(m_x_transformed, m_XC, m_nrXC,
                                                                     progressP.stageWeights, N, &actorAvoidanceP);
        }

        //-------------End of costs in Frenet frame. For cost do not have to transform back

        // Progress Cost
        if (m_progress_cost_on)
        {
            acc += trajectoryProgressCost<T, N>(x, &progressP);
        }

        // Smoothness Costs
        if (m_smoothness_cost_on)
        {
            acc += smothnessCost<T, SWFT>(x, &smoothP, progressP.stageWeights, N);
        }

        // Limit Costs
        if (m_limit_cost_on)
        {
            // acc += speedLimitCost<T>(x, &limitP, delta_t_inv, progressP.stageWeights, N);
            acc += allLimitCost<T, LBFT>(x, &limitP, progressP.stageWeights, progressP.probEpisodeEnd, N);
        }

        // Prior Costs
        if (m_prior_cost_on && m_prior_exists)
        {
            acc += priorCost<T, N, PWFT_LON, PWFT_LAT>(x, m_x_prior, &priorP, progressP.stageWeights);
        }

        // Lateral Costs
        if (m_lateral_cost_on && m_LatFun != NULL && m_LatWeight != NULL)
        {
            acc += lateralCost<T, LAT_FT, LAT_WT>(x, m_LatFun, m_LatWeight, m_nrLaterals, progressP.stageWeights, N);
        }

        // Bump Costs
        if (m_bump_cost_on && m_BumpFun != NULL && m_VelWeight != NULL && m_nrBumps != NULL)
        {
            acc += bumpCost<T, BUMP_FT, VEL_WT>(x, m_BumpFun, m_VelWeight, m_nrBumps, progressP.stageWeights, N,
                                                delta_t_inv);
        }

        return (acc);
    }

    virtual T computeHessian(const T *x)
    {
        int i;
        T acc;

        T *g = this->getGradient();
        T *H = this->getHessian();

        // Zero out
        acc = ((T)0.0);
        for (i = 0; i < 2 * N; i++)
        {
            g[i] = ((T)0.0);
        }
        for (i = 0; i < 2 * N * UB; i++)
        {
            H[i] = ((T)0.0);
        }

        //-------------Frenet transformed cost first
        for (i = -HL; i < N; i++)
            m_frenet_transformer.transformPoint(&m_x_transformed[2 * i], &x[2 * i]);

        // Actor Avoidance Costs
        if (m_actor_avoidance_cost_on && m_XC != NULL && m_nrXC != NULL)
        {
            acc += actorAvoidanceHessianContribution<T, BFT_F, BFT_B, BFT_L, BFT_R>(
                H, g, m_x_transformed, m_XC, m_nrXC, progressP.stageWeights, N, UB, &actorAvoidanceP);
        }

        //-------------End of costs in Frenet frame, Transform gradient and Hessian back from Frenet frame
        frenetTransformHessian<T, FTT>(&m_frenet_transformer, H, g, x, N, UB);

        // Progress Cost
        if (m_progress_cost_on)
        {
            acc += trajectoryProgressHessianContribution<T, N>(H, g, x, &progressP);
        }

        // Smoothness Costs
        if (m_smoothness_cost_on)
        {
            acc += smoothnessHessianContribution<T, SWFT>(H, g, x, &smoothP, progressP.stageWeights, N, UB);
        }

        // Limit Costs
        if (m_limit_cost_on)
        {
            // acc += speedLimitHessianContribution<T>(H, g, x, &limitP, delta_t_inv, progressP.stageWeights, N, UB);
            acc += allLimitHessianContribution<T, LBFT>(H, g, x, &limitP, progressP.stageWeights,
                                                        progressP.probEpisodeEnd, N, UB);
        }

        // Prior Costs
        if (m_prior_cost_on && m_prior_exists)
        {
            acc += priorHessianContribution<T, N, PWFT_LON, PWFT_LAT>(H, g, x, m_x_prior, &priorP,
                                                                      progressP.stageWeights, UB);
        }

        // Lateral Cost
        if (m_lateral_cost_on && m_LatFun != NULL && m_LatWeight != NULL)
        {
            acc += lateralHessianContribution<T, LAT_FT, LAT_WT>(H, g, x, m_LatFun, m_LatWeight, m_nrLaterals,
                                                                 progressP.stageWeights, N, UB);
        }

        // Bump Costs
        if (m_bump_cost_on && m_BumpFun != NULL && m_VelWeight != NULL && m_nrBumps != NULL)
        {
            acc += bumpHessianContribution<T, BUMP_FT, VEL_WT>(H, g, x, m_BumpFun, m_VelWeight, m_nrBumps,
                                                               progressP.stageWeights, N, UB, delta_t_inv);
        }

        /*This was moved inside optimizer to be correct when fixed vector is not NULL
        if(this->getVerboseLevel() >= 3)
        {
            cout << "Gradient Magnitude: " << normOfVector<T>(g, 2 * N) << " ";
        }*/

        return (acc);
    }

    int nrTimesteps()
    {
        return (N);
    }

    FTT *getFrenetTransformer()
    {
        return (&m_frenet_transformer);
    }

    void setLaterals(const LAT_FT *LatFun, const LAT_WT *LatWeight, int nrLaterals)
    {
        m_LatFun = LatFun;
        m_LatWeight = LatWeight;
        m_nrLaterals = nrLaterals;
    }

    void setBumps(const BUMP_FT *const *BumpFun, const VEL_WT *const *VelWeight, const int *nrBumps)
    {
        m_BumpFun = BumpFun;
        m_VelWeight = VelWeight;
        m_nrBumps = nrBumps;
    }

    void setContenderData(T *const *XC, const int *nrXC)
    {
        m_XC = XC;
        m_nrXC = nrXC;

        // Transform the data in place
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < nrXC[i]; j++)
            {
                m_frenet_transformer.transformPointAndVelocity(&(XC[i][4 * j]), &(XC[i][4 * j + 2]), &(XC[i][4 * j]),
                                                               &(XC[i][4 * j + 2]));
            }
        }
    }

    void setEgoTrajectory(T *x)
    {
        copyVector<T>(m_x - (2 * HL), x - (2 * HL), 2 * (N + HL));
    }
    T *getEgoTrajectory()
    {
        return (m_x);
    }

    void setPriorTrajectory(T *prior_x)
    {
        // Copy prior trajectory
        copyVector<T>(m_x_prior - (2 * HL), prior_x - (2 * HL), 2 * (N + HL));
        // Flag that now prior can be used
        m_prior_exists = true;
    }

    void setPriorTrajectoryFromCurrent(T timestep_shift = ((T)1.0))
    {
        // Extract integer and fractional part of shift
        int ipart = ((int)(timestep_shift));
        T frac = timestep_shift - ((T)ipart);

        for (int i = -HL; i < N; i++)
        {
            // Use it to find source indexes
            int source_i = i + ipart;
            int source_ipo = source_i + 1;

            // Enforce bounds of interpolation
            if (source_i >= N)
                source_i = N - 1;
            if (source_i < (-HL))
                source_i = (-HL);
            if (source_ipo >= N)
                source_ipo = N - 1;
            if (source_ipo < (-HL))
                source_ipo = (-HL);

            // Do the interpolation
            m_x_prior[2 * i] = (((T)1.0) - frac) * m_x[2 * source_i] + frac * m_x[2 * source_ipo];
            m_x_prior[2 * i + 1] = (((T)1.0) - frac) * m_x[2 * source_i + 1] + frac * m_x[2 * source_ipo + 1];
        }
        // Flag that now prior can be used
        m_prior_exists = true;
    }

    void copyPriorToCurrent()
    {
        copyVector<T>(m_x - 2 * HL, m_x_prior - 2 * HL, 2 * (N + HL));
    }

    void centerTrajectories()
    {
        T center_x, center_y;

        center_x = m_x[-2];
        center_y = m_x[-2 + 1];
        for (int i = -HL; i < N; i++)
        {
            m_x[2 * i] -= center_x;
            m_x[2 * i + 1] -= center_y;
            m_x_prior[2 * i] -= center_x;
            m_x_prior[2 * i + 1] -= center_y;
        }
    }

    void MoveTrajectoryForward(T timestep_shift = ((T)1.0))
    {
        // Update prior trajectory
        this->setPriorTrajectoryFromCurrent(timestep_shift);
        // Copy prior trajectory to current
        this->copyPriorToCurrent();
        // Put the (-1)-th timestep back at origin
        this->centerTrajectories();
    }

    void run()
    {
        // Run the optimization
        this->runOptimization(m_x);

        // Prepare prior based on this outcome for next time
        this->setPriorTrajectoryFromCurrent();
    }

    T getFinalCost()
    {
        return (this->m_current_cost);
    }

    T getPosition(int i)
    {
        if (i < N && i >= (-HL))
        {
            return (m_x[2 * i]);
        }
        return ((T)0.0);
    }

    T getLatPosition(int i)
    {
        if (i < N && i >= (-HL))
        {
            return (m_x[2 * i + 1]);
        }
        return ((T)0.0);
    }

    T getVelocity(int i)
    {
        if (i < N && i > (-HL))
        {
            return ((m_x[2 * i] - m_x[2 * (i - 1)]) * delta_t_inv);
        }
        return ((T)0.0);
    }

    T getLatVelocity(int i)
    {
        if (i < N && i > (-HL))
        {
            return ((m_x[2 * i + 1] - m_x[2 * (i - 1) + 1]) * delta_t_inv);
        }
        return ((T)0.0);
    }

    T getAcceleration(int i)
    {
        if (i < N && i > (-HL + 1))
        {
            return ((m_x[2 * i] - ((T)2.0) * m_x[2 * (i - 1)] + m_x[2 * (i - 2)]) * delta_t_inv * delta_t_inv);
        }
        return ((T)0.0);
    }

    T getLatAcceleration(int i)
    {
        if (i < N && i > (-HL + 1))
        {
            return ((m_x[2 * i + 1] - ((T)2.0) * m_x[2 * (i - 1) + 1] + m_x[2 * (i - 2) + 1]) * delta_t_inv *
                    delta_t_inv);
        }
        return ((T)0.0);
    }

    T getJerk(int i)
    {
        if (i < N && i > (-HL + 2))
        {
            return ((m_x[2 * i] - ((T)3.0) * m_x[2 * (i - 1)] + ((T)3.0) * m_x[2 * (i - 2)] - m_x[2 * (i - 3)]) *
                    delta_t_inv * delta_t_inv * delta_t_inv);
        }
        return ((T)0.0);
    }

    T getLatJerk(int i)
    {
        if (i < N && i > (-HL + 2))
        {
            return ((m_x[2 * i + 1] - ((T)3.0) * m_x[2 * (i - 1) + 1] + ((T)3.0) * m_x[2 * (i - 2) + 1] -
                     m_x[2 * (i - 3) + 1]) *
                    delta_t_inv * delta_t_inv * delta_t_inv);
        }
        return ((T)0.0);
    }

    void printTrajectory()
    {
        int i;

        cout << "Trajectory <i> (x,y) [v]" << endl;
        for (i = -HL; i < N; i++)
        {
            cout << "<" << setw(3) << i << "> ";
            cout << "(" << setw(4) << m_x[2 * i] << "," << setw(4) << m_x[2 * i + 1] << ")";

            if (i > (-HL))
                cout << " V[" << getVelocity(i) << "]";
            if (i > (-HL))
                cout << " LV[" << getLatVelocity(i) << "]";
            if (i > (-HL + 1))
                cout << " A[" << getAcceleration(i) << "]";
            if (i > (-HL + 1))
                cout << " LA[" << getLatAcceleration(i) << "]";
            if (i > (-HL + 2))
                cout << " J[" << getJerk(i) << "]";
            if (i > (-HL + 2))
                cout << " LJ[" << getLatJerk(i) << "]";

            cout << endl;
        }
        cout << endl;
    }

    void printDistanceToVelocityTable()
    {
        actorAvoidanceP.printDistanceToVelocityTable(progressP.k_p);
    }

    T getDeltaT() const {return delta_t;};
    progressCostParameters<T, N>& getProgressCostParameters() {return progressP;};
    smoothnessCostParameters<T, SWFT>& getSmoothnessCostParameters() {return smoothP;};
    limitCostParameters<T, LBFT>& getLimitCostParameters() {return limitP;};
    actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R>& getActorAvoidanceCostParameters() {return actorAvoidanceP;};

    progressCostParameters<T, N> progressP;
    limitCostParameters<T, LBFT> limitP;
    smoothnessCostParameters<T, SWFT> smoothP;
    actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R> actorAvoidanceP;
    priorCostParameters<T, N, PWFT_LON, PWFT_LAT> priorP;

    const LAT_FT *m_LatFun;
    const LAT_WT *m_LatWeight;
    int m_nrLaterals;

    const BUMP_FT *const *m_BumpFun;
    const VEL_WT *const *m_VelWeight;
    const int *m_nrBumps;

    FTT m_frenet_transformer;
    T planning_horizon, delta_t, delta_t_inv;
    T m_x_mem[2 * (N + HL)];
    T *m_x;
    T m_x_transformed_mem[2 * (N + HL)];
    T *m_x_transformed;
    T m_x_prior_mem[2 * (N + HL)];
    T *m_x_prior;

    const T *const *m_XC;
    const int *m_nrXC;
    bool m_progress_cost_on, m_smoothness_cost_on, m_limit_cost_on;
    bool m_actor_avoidance_cost_on, m_prior_cost_on, m_lateral_cost_on;
    bool m_bump_cost_on;
    bool m_prior_exists;
};

#endif /*TRAJECTORY_OPTIMIZER_H*/
