package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorMotor1;
  private CANSparkMax elevatorMotor2;
  private SparkMaxAnalogSensor analogSensor;

  public ElevatorSubsystem() {
    elevatorMotor1 = new CANSparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
    elevatorMotor2 = new CANSparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);
    analogSensor = elevatorMotor1.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

    elevatorMotor1.restoreFactoryDefaults();
    elevatorMotor2.restoreFactoryDefaults();

    elevatorMotor2.follow(elevatorMotor1, true);
    elevatorMotor1.getForwardLimitSwitch(Type.kNormallyClosed);
    elevatorMotor1.getReverseLimitSwitch(Type.kNormallyClosed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", analogSensor.getPosition());
  }

  public void elevatorUp(double speed) {
    speed = MathUtil.clamp(speed, 0, 1);
    elevatorMotor1.set(0);
  }

  public void elevatorDown(double speed) {
    speed = MathUtil.clamp(speed, -1, 0);
    elevatorMotor1.set(0);
  }

  public void stop() {
    elevatorMotor1.stopMotor();
  }
}