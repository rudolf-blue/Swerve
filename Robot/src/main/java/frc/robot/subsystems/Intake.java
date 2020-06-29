package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

private final CANSparkMax intake;
private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0, 1);
private boolean extended = false;

private CANPIDController m_intakePID;

private CANEncoder m_intakeEncoder;

public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, setpoint, maxRPM;

public Intake() {
intake = new CANSparkMax(Constants.intake, MotorType.kBrushless);
intake.restoreFactoryDefaults();
intake.setSmartCurrentLimit(50);
intake.setIdleMode(IdleMode.kCoast);
intake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
intake.burnFlash();


 m_intakeEncoder = intake.getEncoder();


m_intakePID = intake.getPIDController();

kP = 1;
kI = 0;
kD = 0;
kIz = 0;
kFF = 0;
kMaxOutput = 1;
kMinOutput = 0;
maxRPM = 5000;

m_intakePID.setP(kP);
m_intakePID.setI(kI);
m_intakePID.setD(kD);
m_intakePID.setIZone(kIz);
m_intakePID.setFF(kFF);
m_intakePID.setOutputRange(kMinOutput, kMaxOutput);

}

public void extend() {
    intakeSolenoid.set(Value.kForward);
    extended = true;
}

public void retract() {
    intakeSolenoid.set(Value.kReverse);
    extended = false;
}

public void setIntake(double rpm) {
m_intakePID.setReference(rpm, ControlType.kVelocity);
}


}