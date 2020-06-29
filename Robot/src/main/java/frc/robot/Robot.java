package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  
    private  RobotContainer m_robotContainer;



  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
  m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    RobotContainer.drive.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    //Zero encoders, modules and imu
    //Log open loop motor performance 
    //Generate 1-3 arbitrary trajectories
  }
}
