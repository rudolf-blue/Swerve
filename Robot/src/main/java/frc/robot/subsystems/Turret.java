package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

private final CANSparkMax shooter1, shooter2, turret;

private CANPIDController m_turretPID;
private CANPIDController m_shooterPID;

private CANAnalog m_turretEncoder;
private CANEncoder m_shooterEncoder;

//turret pid constants
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, turretSetPoint, rotations;

//shooter pid constants
public double kSp, kSi, kSd, kSiz, kSff, kSmax, kSmin, maxRPM;

public double desiredSpeed;


public Turret() {
turret = new CANSparkMax(Constants.turret, MotorType.kBrushed);
shooter1 = new CANSparkMax(Constants.shooter1, MotorType.kBrushed);
shooter2 = new CANSparkMax(Constants.shooter2, MotorType.kBrushed);
turret.restoreFactoryDefaults();
shooter1.restoreFactoryDefaults();
shooter2.restoreFactoryDefaults();
turret.setSmartCurrentLimit(30);
shooter1.setSmartCurrentLimit(50);
shooter2.setSmartCurrentLimit(50);
turret.setIdleMode(IdleMode.kBrake);
shooter1.setIdleMode(IdleMode.kCoast);
shooter2.setIdleMode(IdleMode.kCoast);
turret.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
shooter1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
shooter2.follow(shooter1);
turret.burnFlash();
shooter1.burnFlash();
shooter2.burnFlash();

m_turretEncoder = turret.getAnalog(AnalogMode.kAbsolute);
m_shooterEncoder = shooter1.getEncoder();


m_turretPID = turret.getPIDController();

kP = 1;
kI = 0;
kD = 0;
kIz = 0;
kFF = 0;
kMaxOutput = 1;
kMinOutput = -1;

m_turretPID.setD(kD);
m_turretPID.setFF(kFF);
m_turretPID.setP(kP);
m_turretPID.setI(kI);
m_turretPID.setIZone(kIz);
m_turretPID.setOutputRange(kMinOutput, kMaxOutput);


m_shooterPID = shooter1.getPIDController();
kSp = 1;
kSi = 0;
kSd = 0;
kSiz = 0;
kSff = 0;
kSmax = 1;
kSmin = 0;
maxRPM = 20000;

m_shooterPID.setP(kSp);
m_shooterPID.setI(kSi);
m_shooterPID.setD(kSd);
m_shooterPID.setIZone(kSiz);
m_shooterPID.setFF(kSff);
m_shooterPID.setOutputRange(kSmin, kSmax);

}

public void setShooter(double rpm) {
m_shooterPID.setReference(rpm, ControlType.kVelocity);
}

public void setTurret(double error) {
double error2theelectricbugaloo = m_turretEncoder.getPosition()+error; 
m_turretPID.setReference(error2theelectricbugaloo, ControlType.kPosition);
}



}