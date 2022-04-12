#include "lec5_hw/visualizer.hpp"
#include "lec5_hw/trajectory.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/LU>

#include <cmath>
#include <iostream>
#include <vector>
#include <math.h>

struct Config
{
    std::string targetTopic;
    double clickHeight;
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;
    int maxPieceNum;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("ClickHeight", clickHeight);
        nh_priv.getParam("InitialVel", initialVel);
        nh_priv.getParam("InitialAcc", initialAcc);
        nh_priv.getParam("TerminalVel", terminalVel);
        nh_priv.getParam("TerminalAcc", terminalAcc);
        nh_priv.getParam("AllocationSpeed", allocationSpeed);
        nh_priv.getParam("AllocationAcc", allocationAcc);
        nh_priv.getParam("MaxPieceNum", maxPieceNum);
    }
};

double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return 2.0 * sqrt(dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix)
{
    // coefficientMatrix is a matrix with 6*piece num rows and 3 columes
    // As for a polynomial c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5,
    // each 6*3 sub-block of coefficientMatrix is
    // --              --
    // | c0_x c0_y c0_z |
    // | c1_x c1_y c1_z |
    // | c2_x c2_y c2_z |
    // | c3_x c3_y c3_z |
    // | c4_x c4_y c4_z |
    // | c5_x c5_y c5_z |
    // --              --
    // Please computed coefficientMatrix of the minimum-jerk trajectory
    // in this function

    // ------------------------ Put your solution below ------------------------
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2*pieceNum*3, 2*pieceNum*3);
    Eigen::MatrixXd Ei = Eigen::MatrixXd::Zero(2*3, 2*3), Fi = Eigen::MatrixXd::Zero(2*3, 2*3);
    std::cout << "M size: " << M.size() << std::endl;
    std::cout << "Ei size: " << Ei.size() << std::endl;
    double t;
    for (int i = 0; i < (pieceNum-1); i++)
    {
        int pos_x = 3 + 2*3*i;
        int pos_y = 2*3*i;
        t = timeAllocationVector[i];
        Ei <<  1,  t,  t*t,   pow(t,3),    pow(t,4),    pow(t,5),
               1,  t,  t*t,   pow(t,3),    pow(t,4),    pow(t,5),
               0,  1,  2*t, 3*pow(t,2),  4*pow(t,3),  5*pow(t,4),
               0,  0,    2,        6*t, 12*pow(t,2), 20*pow(t,3),
               0,  0,    0,          6,        24*t, 60*pow(t,2),
               0,  0,    0,          0,          24,       120*t;
        Fi <<  0,  0,    0,          0,           0,           0,
              -1,  0,    0,          0,           0,           0,
               0, -1,    0,          0,           0,           0,
               0,  0,   -2,          0,           0,           0,
               0,  0,    0,         -6,           0,           0,
               0,  0,    0,          0,         -24,           0;
        M.block(pos_x, pos_y, 6, 6) = Ei;
        M.block(pos_x, pos_y+6, 6, 6) = Fi;
    }
    
    Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd EM = Eigen::MatrixXd::Zero(3, 6);
    F0 <<  1, 0,   0,  0,  0,  0,
           0, 1,   0,  0,  0,  0,
           0, 0,   2,  0,  0,  0;
    t = timeAllocationVector[pieceNum-1];
    EM <<  1,  t,  t*t,   pow(t,3),    pow(t,4),    pow(t,5),
           0,  1,  2*t, 3*pow(t,2),  4*pow(t,3),  5*pow(t,4),
           0,  0,    2,        6*t, 12*pow(t,2), 20*pow(t,3);
    std::cout << "Finished here" << std::endl;
    M.block(0, 0, 3, 6) = F0;
    M.block(3+2*3*(pieceNum-1), 2*3*(pieceNum-1), 3, 6) = EM;
    std::cout << "Finished there" << std::endl;

    std::cout << M << std::endl;

    // Compute M inverse
    Eigen::PartialPivLU<Eigen::MatrixXd> LU(M);

    // Eigen::MatrixXd Mx = M, My = M, Mz = M;
    
    Eigen::VectorXd bx = Eigen::VectorXd::Zero(2*(pieceNum)*3);
    Eigen::VectorXd by = Eigen::VectorXd::Zero(2*(pieceNum)*3);
    Eigen::VectorXd bz = Eigen::VectorXd::Zero(2*(pieceNum)*3);
    
    bx[0] = initialPos[0];
    bx[1] = initialVel[0];
    bx[2] = initialAcc[0];
    by[0] = initialPos[1];
    by[1] = initialVel[1];
    by[2] = initialAcc[1];
    bz[0] = initialPos[2];
    bz[1] = initialVel[2];
    bz[2] = initialAcc[2];

    bx.tail(3)[0] = terminalPos[0];
    bx.tail(3)[1] = terminalVel[0];
    bx.tail(3)[2] = terminalAcc[0];
    by.tail(3)[0] = terminalPos[1];
    by.tail(3)[1] = terminalVel[1];
    by.tail(3)[2] = terminalAcc[1];
    bz.tail(3)[0] = terminalPos[2];
    bz.tail(3)[1] = terminalVel[2];
    bz.tail(3)[2] = terminalAcc[2];

    for (int i = 0; i < intermediatePositions.cols(); i++)
    {
        int location = 3 + i*6;
        bx[location] = intermediatePositions.coeff(0, i);
        by[location] = intermediatePositions.coeff(1, i);
        bz[location] = intermediatePositions.coeff(2, i);
    }

    Eigen::VectorXd sol_x((2*(pieceNum+1)*3));
    Eigen::VectorXd sol_y((2*(pieceNum+1)*3));
    Eigen::VectorXd sol_z((2*(pieceNum+1)*3));

    sol_x = LU.solve(bx);
    sol_y = LU.solve(by);
    sol_z = LU.solve(bz);

    std::cout << sol_x.size() << std::endl;
    for (int i = 0; i < (pieceNum); i++)
    {
        int location = i*6;
        std::cout << "location end: " << location+6 << std::endl;
        coefficientMatrix.block(location, 0, 6, 1) = sol_x.segment(location, 6);
        coefficientMatrix.block(location, 1, 6, 1) = sol_y.segment(location, 6);
        coefficientMatrix.block(location, 2, 6, 1) = sol_z.segment(location, 6);
        
    }

    // ------------------------ Put your solution above ------------------------
}

class ClickGen
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber targetSub;

    Visualizer visualizer;

    Eigen::Matrix3Xd positions;
    Eigen::VectorXd times;
    int positionNum;
    Trajectory<5> traj;

public:
    ClickGen(const Config &conf,
             ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          visualizer(nh),
          positions(3, config.maxPieceNum + 1),
          times(config.maxPieceNum),
          positionNum(0)
    {
        targetSub = nh.subscribe(config.targetTopic, 1,
                                 &ClickGen::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (positionNum > config.maxPieceNum)
        {
            positionNum = 0;
            traj.clear();
        }

        positions(0, positionNum) = msg->pose.position.x;
        positions(1, positionNum) = msg->pose.position.y;
        positions(2, positionNum) = std::fabs(msg->pose.orientation.z) * config.clickHeight;

        if (positionNum > 0)
        {
            const double dist = (positions.col(positionNum) - positions.col(positionNum - 1)).norm();
            times(positionNum - 1) = timeTrapzVel(dist, config.allocationSpeed, config.allocationAcc);
        }

        ++positionNum;

        if (positionNum > 1)
        {
            const int pieceNum = positionNum - 1;
            const Eigen::Vector3d initialPos = positions.col(0);
            const Eigen::Vector3d initialVel(config.initialVel[0], config.initialVel[1], config.initialVel[2]);
            const Eigen::Vector3d initialAcc(config.initialAcc[0], config.initialAcc[1], config.initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions.col(pieceNum);
            const Eigen::Vector3d terminalVel(config.terminalVel[0], config.terminalVel[1], config.terminalVel[2]);
            const Eigen::Vector3d terminalAcc(config.terminalAcc[0], config.terminalAcc[1], config.terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times.head(pieceNum);

            Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

            minimumJerkTrajGen(pieceNum,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);

            traj.clear();
            traj.reserve(pieceNum);
            for (int i = 0; i < pieceNum; i++)
            {
                traj.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
        }

        visualizer.visualize(traj, positions.leftCols(positionNum));

        return;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "click_gen_node");
    ros::NodeHandle nh_;
    ClickGen clickGen(Config(ros::NodeHandle("~")), nh_);
    ros::spin();
    return 0;
}
