package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;


public class SwerveModule {

private final CANSparkMax m_driveMotor;
private final CANSparkMax m_turningMotor;

final CANEncoder m_driveEncoder;
final CANEncoder m_turningEncoder;
final CANAnalog m_absoluteEncoder;

private CANPIDController m_pidController;
public double kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput;

public SwerveModule(int driveMotor, int turningMotor, double encoderCPR) {

    m_driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotor, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_driveMotor.setSmartCurrentLimit(50);
    m_turningMotor.setSmartCurrentLimit(30);

    m_driveEncoder = new CANEncoder(m_driveMotor);
    m_turningEncoder = new CANEncoder(m_turningMotor);
    m_absoluteEncoder = new CANAnalog(m_turningMotor, AnalogMode.kAbsolute);

    //distance driven per pulse
    m_driveEncoder.setPositionConversionFactor(1);
    //velocity from pulses
    m_driveEncoder.setVelocityConversionFactor(1/60.0);



    //Azimuth PID
    kP = 1;
    kD = 0;
    kI = 0;
    kFF = 0;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    m_pidController = m_turningMotor.getPIDController();
    m_pidController.setP(kP);
    m_pidController.setD(kD);
    m_pidController.setI(kI);
    m_pidController.setFF(kFF);
    m_pidController.setIZone(kIz);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

}

public SwerveModuleState getState() {
return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
}

public void setDesiredState(SwerveModuleState state) {

double desiredDrive = state.speedMetersPerSecond/1;
double desiredSteering = state.angle.getRadians();
double currentSteering = m_turningEncoder.getPosition();

double steeringError = Math.IEEEremainder(desiredSteering - currentSteering, Math.PI*2);

if (steeringError > Math.PI / 2) {
    steeringError -= Math.PI;
    desiredDrive *= -1;
}else if (steeringError < -Math.PI / 2) {
    steeringError += Math.PI;
    desiredDrive *= -1;
}

double steeringSetpoint = currentSteering + steeringError;

m_driveMotor.set(desiredDrive);

m_pidController.setReference(steeringSetpoint, ControlType.kPosition);
}

public void resetEncoder(){
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
}





}