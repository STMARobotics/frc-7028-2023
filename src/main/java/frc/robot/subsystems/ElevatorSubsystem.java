package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem for elevator mechanism
 */
public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax elevatorLeader;
  private final CANSparkMax elevatorFollower;
  private final SparkMaxAnalogSensor analogSensor;

  public ElevatorSubsystem() {
    elevatorLeader = new CANSparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
    elevatorFollower = new CANSparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);
    
    elevatorLeader.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();
    
    // Configure potentiometer
    analogSensor = elevatorLeader.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    analogSensor.setInverted(true);

    // Configure soft limits
    elevatorLeader.setSoftLimit(SoftLimitDirection.kForward, 2.38377f);
    elevatorLeader.setSoftLimit(SoftLimitDirection.kReverse, .8f); // real limit was 0.6487
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elevatorLeader.getPIDController().setFeedbackDevice(analogSensor);

    // Configure limit switches, or lack thereof
    elevatorLeader.getForwardLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false);
    elevatorLeader.getReverseLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(true);

    elevatorFollower.follow(elevatorLeader, true);

    // Brake mode helps hold the elevator in place
    elevatorLeader.setIdleMode(IdleMode.kBrake);
    elevatorFollower.setIdleMode(IdleMode.kBrake);

    // Save settings to motor flash, so they persist between power cycle
    elevatorLeader.burnFlash();
    elevatorFollower.burnFlash();
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", analogSensor.getPosition());
  }

  public void moveElevator(double speed) {
    elevatorLeader.set(speed);
  }

  public void stop() {
    elevatorLeader.stopMotor();
  }
  
}