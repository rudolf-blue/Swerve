package frc.robot.pose;

import org.apache.commons.math3.*;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.math.*;


public class Kalman {

    //Timescale for integration
    double dt = .1;
    //Acceleration equation
    double accel = .5 * Math.pow(dt, 2);
    //Jerk equation
    double jerk = (1/3) * Math.pow(dt, 3);
 
    //A matrix, state transition matrix:
    //
    //[ 1, 0, 0, 0, 1/2(dt)^2, 0 ]
    //[ 0, 1, dt, 0, 0, 1/2(dt)^2]
    //[ 0, 0, 1, dt, 0, 0]
    //[ 0, 0, 0, 1, dt, 0]
    //[ 0, 0, 0, 0, 1, dt]
    //[ 0, 0, 0, 0, 0, 1]

    RealMatrix A = new Array2DRowRealMatrix(new double[][] { { 1, 0, 0, 0, accel, 0 },
                                                             { 0, 1, dt, 0, 0, 0, accel}, 
                                                             { 0, 0, 1, dt, 0, 0}, 
                                                             { 0, 0, 0, 1, dt, 0},
                                                             { 0, 0, 0, 0, 1, dt},
                                                             { 0, 0, 0, 0, 0, 1}});


    //B matrix, control input matrix:
    //
    //[ 1/3(dt)^3 ]
    //[ 1/3(dt)^3 ]
    //[ 1/2(dt)^2 ]
    //[ 1/2(dt)^2 ]
    //[ dt ]
    //[ dt ]

    RealMatrix B = new Array2DRowRealMatrix(new double[][] { { jerk },
                                                             { jerk }, 
                                                             { accel }, 
                                                             { accel },
                                                             { dt},
                                                             { dt}});

    //H matrix, measurement matrix
    








}