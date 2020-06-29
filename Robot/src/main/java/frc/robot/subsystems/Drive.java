package frc.robot.subsystems;

import com.revrobotics.*;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Drive extends SubsystemBase {

    private final SwerveModule m_frontLeft = new SwerveModule(Constants.frontLeftDrive, Constants.frontLeftSteer, 1024);

    private final SwerveModule m_frontRight = new SwerveModule(Constants.frontRightDrive, Constants.frontRightSteer, 1024);

    private final SwerveModule m_backRight = new SwerveModule(Constants.backRightDrive, Constants.backRightSteer, 1024);

    private final SwerveModule m_backLeft = new SwerveModule(Constants.backLeftDrive, Constants.backLeftSteer, 1024);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.kDriveKinematics, getAngle());

    public Drive() {

    }

    public Rotation2d getAngle()   {
        //It's really dumb that there are only really 2 different conventions for angles and WPIlib writers and FRC vendors decided to use different ones
        return Rotation2d.fromDegrees(navX.getAngle()+90 *-1);
    }
    
    public Pose2d getBasePose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getAngle());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        
        var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }


    public void setModuleStates(SwerveModuleState[] desiredStates)  {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontLeft.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        m_frontLeft.resetEncoder();
        m_frontRight.resetEncoder();
        m_backLeft.resetEncoder();
        m_backRight.resetEncoder();
    }

    public void zeroHeading() {
        navX.reset();
    }


    public double getHeading() {
        return Math.IEEEremainder(navX.getAngle(), 360) * (-1);
    }

    public double getTurnRate() {
        return navX.getRate()*(-1);

    }


    @Override
    public void periodic() {
        m_odometry.update(

            getAngle(),
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState());
            


        }





    }
