package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem for elevator mechanism
 */
public class ElevatorSubsystem extends SubsystemBase {

  private static final int SMART_MOTION_SLOT = 0;
  private static final double LIMIT_BOTTOM = 0.646978;
  private static final double LIMIT_TOP = 2.38377;
  // Elevator height, in meters
  private static final double ELEVATOR_HEIGHT = Units.inchesToMeters(16.875);
  // Coefficient in meters per sensor value
  private static final double SENSOR_COEFFICIENT = ELEVATOR_HEIGHT / (LIMIT_TOP - LIMIT_BOTTOM);
  // Offset to add to sensor value - offset from elevator bottom to sensor bottom
  private static final double SENSOR_OFFSET = -LIMIT_BOTTOM * SENSOR_COEFFICIENT;

  private final CANSparkMax elevatorLeader;
  private final CANSparkMax elevatorFollower;
  private final SparkMaxAnalogSensor analogSensor;
  private final SparkMaxPIDController pidController;

  public ElevatorSubsystem() {
    elevatorLeader = new CANSparkMax(ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
    elevatorFollower = new CANSparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);
    
    elevatorLeader.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();

    // Configure potentiometer
    analogSensor = elevatorLeader.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    analogSensor.setInverted(true);
    pidController = elevatorLeader.getPIDController();
    pidController.setFeedbackDevice(analogSensor);

    // Configure closed-loop control
    double kP = .00002; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0;
    double kFF = 0.007;
    double kMaxOutput = 1; // TODO safe values that can be increased when confident
    double kMinOutput = -1;
    double allowedErr = 0.004;

    // Smart Motion Coefficients
    double maxVel = 6500; // rpm
    double maxAcc = 1500;
    double minVel = 0;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput); 

    pidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    pidController.setSmartMotionMinOutputVelocity(minVel, SMART_MOTION_SLOT);
    pidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);

    // Voltage compensation and current limits
    elevatorLeader.enableVoltageCompensation(12);
    elevatorLeader.setSmartCurrentLimit(20);
    elevatorFollower.setSmartCurrentLimit(20);

    // Configure soft limits
    elevatorLeader.setSoftLimit(SoftLimitDirection.kForward, (float) LIMIT_TOP);
    elevatorLeader.setSoftLimit(SoftLimitDirection.kReverse, 0.67f); // Didn't use constant to avoid crashing into limit switch
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);

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
    SmartDashboard.putNumber("Elevator Position Raw", analogSensor.getPosition());
    SmartDashboard.putNumber("Elevator Position Inches",  Units.metersToInches(getElevatorPosition()));
    SmartDashboard.putNumber("Elevator RPM (?)", elevatorLeader.getEncoder().getVelocity());
  }

  /**
   * Moves the elevator using duty cycle
   * @param speed duty cycle [-1, 1]
   */
  public void moveElevator(double speed) {
    elevatorLeader.set(speed);
  }

  /**
   * Moves the elevator to a position
   * @param meters position in meters
   */
  public void moveToPosition(double meters) {
    pidController.setReference(metersToAnalogPosition(meters), ControlType.kSmartMotion);
  }
  
  /**
   * Get the elevator position in meters
   * @return position in meters
   */
  public double getElevatorPosition() {
    return analogPositionToMeters(analogSensor.getPosition());
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    elevatorLeader.stopMotor();
  }

  static double analogPositionToMeters(double analogPosition) {
    return (analogPosition * SENSOR_COEFFICIENT) + SENSOR_OFFSET;
  }

  static double metersToAnalogPosition(double positionMeters) {
    return (positionMeters / SENSOR_COEFFICIENT) - (SENSOR_OFFSET / SENSOR_COEFFICIENT);
  }
  
}