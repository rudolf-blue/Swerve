package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

private final CANSparkMax indexer;

private CANPIDController m_indexerPID;

private CANEncoder m_indexerEncoder;

public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, setpoint, maxRPM;


private static final double slotRatio = 17;

public Indexer() {
indexer = new CANSparkMax(Constants.indexer, MotorType.kBrushless);
indexer.restoreFactoryDefaults();
indexer.setSmartCurrentLimit(30);
indexer.setIdleMode(IdleMode.kBrake);
indexer.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
indexer.burnFlash();

 m_indexerEncoder = indexer.getEncoder();

m_indexerPID = indexer.getPIDController();

kP = 1;
kI = 0;
kD = 0;
kIz = 0;
kFF = 0;
kMaxOutput = 1;
kMinOutput = -1;
maxRPM = 5000;

m_indexerPID.setP(kP);
m_indexerPID.setI(kI);
m_indexerPID.setD(kD);
m_indexerPID.setFF(kFF);
m_indexerPID.setIZone(kIz);
m_indexerPID.setOutputRange(kMinOutput, kMaxOutput);

}

public void rotateIndexer(double numTimes){
    m_indexerPID.setReference(slotRatio*numTimes, ControlType.kPosition);


}

public void setIndexer(double velocity){
    m_indexerPID.setReference(velocity, ControlType.kVelocity);
}

}