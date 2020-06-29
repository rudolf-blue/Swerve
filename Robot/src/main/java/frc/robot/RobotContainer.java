package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

import frc.robot.subsystems.*;
import frc.robot.Vision;

public class RobotContainer {

        public static Drive drive = new Drive();
        public static Turret turret = new Turret();
        public static Vision vision = new Vision("Chameleon Vision");
        
        public static Joystick m_driverController = new Joystick(0);
        public static Joystick m_operatorController = new Joystick(1);

        public double linear = -m_driverController.getRawAxis(1) * Constants.kMaxSpeedMetersPerSecond;
        public double strafe = -m_driverController.getRawAxis(0) * Constants.kMaxSpeedMetersPerSecond;
        public double rotation = -m_driverController.getRawAxis(4) * Math.PI * 2;
        
        public NetworkTableEntry turretYaw;
        public NetworkTableEntry intakeYaw;
        public NetworkTableEntry visionPose;
        

        public static ProfiledPIDController theta = new ProfiledPIDController(0, 0, 0,
                        Constants.kThetaControllerConstraints);



    public RobotContainer() {

        drive.setDefaultCommand(

        new RunCommand(() -> drive.drive(strafe, linear, rotation, true), drive)
        
        );


    }

    private void configureButtonBindings() {
        // new JoystickButton(m_operatorController, 1).whenPressed(new InstantCommand(intake::extend, intake))
        //                 .whenReleased(intake::retract, intake);
    }


    public InstantCommand getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared)
                                        .setKinematics(Constants.kDriveKinematics);

            

        Trajectory left = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 1, new Rotation2d(-(Math.PI) / 2.)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(0, 0)

                        ),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, -1, new Rotation2d(-(Math.PI) / 2.)), config);

        Trajectory right = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d((Math.PI) / 2.)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(0, 0.5)

                        ),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 1, new Rotation2d((Math.PI) / 2.)), config);

        Trajectory middle = TrajectoryGenerator.generateTrajectory(
                            // Start at the origin facing the +X direction
                            new Pose2d(0, 0, new Rotation2d((Math.PI) / 2.)),
                            // Pass through these two interior waypoints, making an 's' curve path
                            List.of(new Translation2d(0, 0.5)
    
                            ),
                            // End 3 meters straight ahead of where we started, facing forward
                            new Pose2d(0, 1, new Rotation2d((Math.PI) / 2.)), config);

        

        SwerveControllerCommand auto = new SwerveControllerCommand(middle, drive::getBasePose, Constants.kDriveKinematics, new PIDController(1, 0, 0), new PIDController(1, 0, 0), theta, drive::setModuleStates, drive);

        //SwerveControllerCommand auto = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements)

        return new InstantCommand();

            }
}