package frc.robot;

import java.util.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


public final class Constants {


    public static final int frontLeftDrive = 1;
    public static final int frontLeftSteer = 2;

    public static final int frontRightDrive = 3;
    public static final int frontRightSteer = 4;

    public static final int backLeftDrive = 5;
    public static final int backLeftSteer = 6;

    public static final int backRightDrive = 7;
    public static final int backRightSteer = 8;

    public static final int turret = 9;

    public static final int intake = 10;
    
    public static final int indexer = 11;

    public static final int shooter1 = 12;
    public static final int shooter2 = 13;
    

    //meters, totally fake right now
    public static final double kTrackWidth = 1;
    public static final double kWheelBase = 1;
    public static final double driveEncoderConversion = 1;
    public static final double indexerEncoderConversion = 1;

    //swerve constants
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), 
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) 
    );

    public static final double nominalVoltage = 0;

    //Drive base controller constants, totally fake right now
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 1;
    public static final double kaVoltSecondsSquaredPerMeter = 1;
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
	private static final double kMaxAngularSpeedRadiansPerSecond = 0;
	private static final double kMaxAngularSpeedRadiansPerSecondSquared = 0;
    
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  }








