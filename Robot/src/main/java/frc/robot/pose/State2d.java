package frc.robot.pose;

import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class State2d{

private final double xPos;
private final double yPos;
private final double xVel;
private final double yVel;
private final double xAccel;
private final double yAccel;

public State2d(){
xPos = 0;
yPos = 0;
xVel = 0;
yVel = 0;
xAccel = 0;
yAccel = 0;

}

public State2d(double xpos, double ypos, double xvel, double yvel, double xaccel, double yaccel) {
xPos = xpos;
yPos = ypos;
xVel = xvel;
yVel = yvel;
xAccel = xaccel;
yAccel = yaccel;
}

public State2d(double xpos, double ypos, double xvel, double yvel) {
    xPos = xpos;
    yPos = ypos;
    xVel = xvel;
    yVel = yvel;
    xAccel = 0;
    yAccel = 0;
}

public State2d(double xpos, double ypos) {
    xPos = xpos;
    yPos = ypos;
    xVel = 0;
    yVel = 0;
    xAccel = 0;
    yAccel = 0;
}

public double getXPos(){
    return xPos;
}
public double getYPos(){
    return yPos;
}
public double getXVel(){
    return xVel;
}
public double getYVel(){
    return yVel;
}
public double getXAccel(){
    return xAccel;
}
public double getYAccel(){
    return yAccel;
}
}