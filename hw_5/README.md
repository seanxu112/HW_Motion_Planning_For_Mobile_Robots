# HW_Motion_Planning_For_Mobile_Robots
<!-- ![alt text](RRT*.png) -->

<img src="gif_version.gif"  /> 


```
Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2*pieceNum*3, 2*pieceNum*3);
Eigen::MatrixXd Ei = Eigen::MatrixXd::Zero(2*3, 2*3), Fi = Eigen::MatrixXd::Zero(2*3, 2*3);
```
Initialize M, Ei, and Fi to the right size. Note that M is the same for all x, y, z.

```
    Eigen::MatrixXd F0 = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd EM = Eigen::MatrixXd::Zero(3, 6);
    F0 <<  1, 0,   0,  0,  0,  0,
           0, 1,   0,  0,  0,  0,
           0, 0,   2,  0,  0,  0;
    t = timeAllocationVector[pieceNum-1];
    EM <<  1,  t,  t*t,   pow(t,3),    pow(t,4),    pow(t,5),
           0,  1,  2*t, 3*pow(t,2),  4*pow(t,3),  5*pow(t,4),
           0,  0,    2,        6*t, 12*pow(t,2), 20*pow(t,3);
    M.block(0, 0, 3, 6) = F0;
    M.block(3+2*3*(pieceNum-1), 2*3*(pieceNum-1), 3, 6) = EM;
```
Setup boundary condition for the Coefficient Matrix.

```
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
```
Setup the intermediate point constraints. The first row of Ei, and Fi is setting up the position constraints to make sure the polynomial pass through the intermediate point. The rest of the rows are to ensure the higher order derivatives of polynomials c_i and c_{i+1} to be the same value.

```
Eigen::PartialPivLU<Eigen::MatrixXd> LU(M);
```
Setup PartialLU solver for Coefficient Matrix M.

```
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
```
Setup the actual value of boundary value and intermediate values to the Right Hand Size.

```
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
```
Solve the linear system and allocate them into the right location in the coefficientMatrix.
