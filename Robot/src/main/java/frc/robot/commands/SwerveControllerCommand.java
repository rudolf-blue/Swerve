package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

@SuppressWarnings("MemberName")
public class SwerveControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();

  private final Trajectory m_trajectory;
  private final double m_targetRotation;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;



  @SuppressWarnings("ParameterName")
  public SwerveControllerCommand(Trajectory trajectory, double targetRotationRadians, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics, PIDController xController, PIDController yController, ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
    m_targetRotation = targetRotationRadians;
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

    m_xController = requireNonNullParam(xController,
      "xController", "SwerveControllerCommand");
    m_yController = requireNonNullParam(yController,
      "xController", "SwerveControllerCommand");
    m_thetaController = requireNonNullParam(thetaController,
      "thetaController", "SwerveControllerCommand");

    m_outputModuleStates = requireNonNullParam(outputModuleStates,
      "frontLeftOutput", "SwerveControllerCommand");
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();

    var desiredState = m_trajectory.sample(curTime);
    var desiredPose = desiredState.poseMeters;

    double targetXVel = m_xController.calculate(
        m_pose.get().getTranslation().getX(),
        desiredPose.getTranslation().getX());

    double targetYVel = m_yController.calculate(
        m_pose.get().getTranslation().getY(),
        desiredPose.getTranslation().getY());

    // The robot will go to the desired rotation of the final pose in the trajectory,
    // not following the poses at individual states.
    double targetAngularVel = m_thetaController.calculate(
        m_pose.get().getRotation().getRadians(),
        m_targetRotation);

    double vRef = desiredState.velocityMetersPerSecond;

    targetXVel += vRef * desiredPose.getRotation().getCos();
    targetYVel += vRef * desiredPose.getRotation().getSin();

    var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetXVel, targetYVel, targetAngularVel, m_pose.get().getRotation());

    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);

  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
  }
}