package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorLeader;
  private CANSparkMax elevatorFollower;
  private SparkMaxAnalogSensor analogSensor;

  public ElevatorSubsystem() {
    elevatorLeader = new CANSparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
    elevatorFollower = new CANSparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);
    
    elevatorLeader.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();
    
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kForward, false);
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, false);
    elevatorLeader.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
    elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
    
    elevatorFollower.follow(elevatorLeader, true);
    elevatorLeader.burnFlash();
    elevatorFollower.burnFlash();
    
    analogSensor = elevatorLeader.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", analogSensor.getPosition());
  }

  public void elevatorUp(double speed) {
    speed = MathUtil.clamp(speed, 0, 1);
    speed = MathUtil.applyDeadband(speed, 0.05);
    elevatorLeader.set(speed);
  }

  public void elevatorDown(double speed) {
    speed = MathUtil.clamp(speed, -1, 0);
    speed = MathUtil.applyDeadband(speed, 0.05);
    elevatorLeader.set(speed);
  }

  public void stop() {
    elevatorLeader.stopMotor();
  }
}