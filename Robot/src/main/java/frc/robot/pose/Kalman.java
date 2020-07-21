package frc.robot.pose;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import frc.robot.pose.State2d;

import frc.robot.Constants;


public class Kalman {

    //Timescale for integration
    double dt = .1;
    //Acceleration equation
    double accel = (1/2) * Math.pow(dt, 2);
    //Jerk equation
    double jerk = (1/3) * Math.pow(dt, 3);
 
    //A - state transition matrix
    private RealMatrix A;
    //B - control input matrix
    private RealMatrix B;
    //H - measurement matrix
    private RealMatrix H;
    //Q - process noise covariance matrix (error in the process)
    private RealMatrix Q;
    //R - measurement noise covariance matrix (error in the measurement)
    private RealMatrix R;
    //PO - error covariance matrix
    private RealMatrix PO;
    //x state
    private RealVector x;
    //Filter
    private KalmanFilter filter;


    public Kalman(){
    //A matrix, state transition matrix:
    //
    //[ 1, 0, 0, 0, 1/2(dt)^2, 0 ]
    //[ 0, 1, dt, 0, 0, 1/2(dt)^2]
    //[ 0, 0, 1, dt, 0, 0]
    //[ 0, 0, 0, 1, dt, 0]
    //[ 0, 0, 0, 0, 1, dt]
    //[ 0, 0, 0, 0, 0, 1]

    RealMatrix A = new Array2DRowRealMatrix(new double[][] { { 1, 0, 0, 0, accel, 0 },
                                                             { 0, 1, dt, 0, 0, accel}, 
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

    //H matrix, measurement matrix:
    //
    //[ 1, 0, 0, 0, 0, 0]
    //[ 0, 1, 0, 0, 0, 0]
    //[ 0, 0, 1, 0, 0, 0]
    //[ 0, 0, 0, 1, 0, 0]
    //[ 0, 0, 0, 0, 1, 0]
    //[ 0, 0, 0, 0, 0, 1]

    RealMatrix H = new Array2DRowRealMatrix(new double[][] { { 1, 0, 0, 0, 0, 0},
                                                             { 0, 1, 0, 0, 0, 0}, 
                                                             { 0, 0, 1, 0, 0, 0}, 
                                                             { 0, 0, 0, 1, 0, 0},
                                                             { 0, 0, 0, 0, 1, 0},
                                                             { 0, 0, 0, 0, 0, 1}});

    // X vector, initial state vector:
    //
    //[ 0, 0, 0, 0, 0, 0]

    RealVector x = new ArrayRealVector(new double[] { 0, 0, 0, 0, 0, 0 });

    //Q matrix, process noise covariance matrix
    //
    //[ 1, 0, 0, 0, 0, 0]
    //[ 0, 1, 0, 0, 0, 0]
    //[ 0, 0, 1, 0, 0, 0]
    //[ 0, 0, 0, 1, 0, 0]
    //[ 0, 0, 0, 0, 1, 0]
    //[ 0, 0, 0, 0, 0, 1]

    RealMatrix Q = new Array2DRowRealMatrix(new double[][] { { 1, 0, 0, 0, 0, 0},
                                                             { 0, 1, 0, 0, 0, 0}, 
                                                             { 0, 0, 1, 0, 0, 0}, 
                                                             { 0, 0, 0, 1, 0, 0},
                                                             { 0, 0, 0, 0, 1, 0},
                                                             { 0, 0, 0, 0, 0, 1}});

    //R matrix, measurement noise covariance matrix
    //
    //[ 1, 0, 0, 0, 0, 0]
    //[ 0, 1, 0, 0, 0, 0]
    //[ 0, 0, 1, 0, 0, 0]
    //[ 0, 0, 0, 1, 0, 0]
    //[ 0, 0, 0, 0, 1, 0]
    //[ 0, 0, 0, 0, 0, 1]

    RealMatrix R = new Array2DRowRealMatrix(new double[][] { { 1, 0, 0, 0, 0, 0},
                                                             { 0, 1, 0, 0, 0, 0}, 
                                                             { 0, 0, 1, 0, 0, 0}, 
                                                             { 0, 0, 0, 1, 0, 0},
                                                             { 0, 0, 0, 0, 1, 0},
                                                             { 0, 0, 0, 0, 0, 1}});

    //P matrix, error covariance matrix
    //
    //[ 1, 1, 1, 1, 1, 1]
    //[ 1, 1, 1, 1, 1, 1]
    //[ 1, 1, 1, 1, 1, 1]
    //[ 1, 1, 1, 1, 1, 1]
    //[ 1, 1, 1, 1, 1, 1]
    //[ 1, 1, 1, 1, 1, 1]

    RealMatrix PO = new Array2DRowRealMatrix(new double[][] { { 1, 1, 1, 1, 1, 1},
                                                              { 1, 1, 1, 1, 1, 1}, 
                                                              { 1, 1, 1, 1, 1, 1}, 
                                                              { 1, 1, 1, 1, 1, 1},
                                                              { 1, 1, 1, 1, 1, 1},
                                                              { 1, 1, 1, 1, 1, 1}});

    ProcessModel pm = new DefaultProcessModel(A, B, Q, x, PO);
    MeasurementModel mm = new DefaultMeasurementModel(H, R);
    filter = new KalmanFilter(pm, mm);
    }

    public State2d estimateState(State2d vision2d, State2d vision3d, State2d IMU, State2d encoderOdometry) {
    
        State2d estimation = new State2d();



    return estimation;

    }


}

