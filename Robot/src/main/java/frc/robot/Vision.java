package frc.robot;

import java.io.FileOutputStream;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;


public class Vision {

    private NetworkTable table;

    public Vision(String cameraName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("chameleon-vision").getSubTable(cameraName);
    }

    // locate "yaw" in table and return it
    public double getYawAngle() {
        return table.getEntry("targetYaw").getDouble(1.0);
    }
    public double getTargetPitch() {
        return table.getEntry("targetPitch").getDouble(1);
    }
    
    public double getTargetArea() {
        return table.getEntry("targetArea").getDouble(1);
    }

    public double getTargetRectangleHeight() {
        return table.getEntry("targetBoundingHeight").getDouble(1);
    }

    private double getYFOV() {
        return 33;
    }

    public double getNormalizeDistance() {
        final double opticalDistance =this.getDistance();
        return Math.sqrt((opticalDistance * opticalDistance)); 
    }

    private double getDistance(double rectHeight) {
        final double targetHeight = 0.42; // In Meters.
        final int yResolution = 240; // In Pixels.
        return (targetHeight * yResolution) / (2 * Math.tan(Math.toRadians(this.getYFOV()/2)) * rectHeight);
    }
    
    public double getDistance() {
        return this.getDistance(this.getTargetRectangleHeight());
    }

    public double[] getTargetPose() {
        double[] fakePose = {0, 0, 0};
        return table.getEntry("targetPose").getDoubleArray(fakePose);
    }

    public double getActualPitch(){
        return (this.getTargetPitch());
    }
   

} 