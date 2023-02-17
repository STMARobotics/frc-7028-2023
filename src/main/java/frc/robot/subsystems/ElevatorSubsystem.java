package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem for elevator mechanism
 */
public class ElevatorSubsystem extends SubsystemBase {

  // Elevator travel distance, in meters
  private static final double ELEVATOR_HEIGHT = 1.105 - 0.102;

  // Motor's encoder limits, in encoder ticks
  private static final double MOTOR_BOTTOM = 0;
  private static final double MOTOR_TOP = 40012;

  // Mutiply by sensor position to get meters
  private static final double MOTOR_ENCODER_POSITION_COEFFICIENT = ELEVATOR_HEIGHT / (MOTOR_TOP - MOTOR_BOTTOM);
  // Mutiply by sensor velocity to get meters per second
  private static final double MOTOR_ENCODER_VELOCITY_COEFFICIENT = MOTOR_ENCODER_POSITION_COEFFICIENT * 10;

  private static final int ANALOG_BOTTOM = 758;
  private static final int ANALOG_TOP = 1796;

  // Mutiply by sensor position to get meters
  private static final double ANALOG_SENSOR_COEFFICIENT = ELEVATOR_HEIGHT / (ANALOG_TOP - ANALOG_BOTTOM);

  private final WPI_TalonFX elevatorLeader;
  private final WPI_TalonFX elevatorFollower;
  private final AnalogInput analogSensor;

  public ElevatorSubsystem() {
    elevatorLeader = new WPI_TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID);
    elevatorFollower = new WPI_TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID);
    
    // Configure potentiometer
    analogSensor = new AnalogInput(ElevatorConstants.ANALOG_SENSOR_CHANNEL);

    // Configure closed-loop control
    double kP = .00004; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0;
    double kF = 0.008;
    double kMaxOutput = .5;
    double kMinOutput = -.5;
    double allowedErr = 0.004;

    // Magic Motion Coefficients
    double maxVel = 6500; // rpm // TODO convert to sensor units per 100ms
    double maxAcc = 1500;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kP = kP;
    config.slot0.kI = kI;
    config.slot0.kD = kD;
    config.slot0.integralZone = kIz;
    config.slot0.kF = kF;
    config.slot0.allowableClosedloopError = allowedErr;
    
    config.motionCruiseVelocity = maxVel;
    config.motionAcceleration = maxAcc;

    // Voltage compensation and current limits
    config.voltageCompSaturation = 12;
    // config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(); // TODO research this

    // Configure soft limits
    config.forwardSoftLimitEnable = true;
    config.forwardSoftLimitThreshold = MOTOR_TOP - metersToMotorPosition(0.02);
    config.reverseSoftLimitEnable = true;
    config.reverseSoftLimitThreshold = MOTOR_BOTTOM + metersToMotorPosition(0.02);
    config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;

    elevatorLeader.configAllSettings(config);
    elevatorFollower.configAllSettings(config);

    elevatorLeader.configPeakOutputForward(kMaxOutput);
    elevatorLeader.configPeakOutputReverse(kMinOutput);
    elevatorLeader.enableVoltageCompensation(true);

    elevatorLeader.setInverted(true);
    elevatorFollower.setInverted(true);

    elevatorFollower.follow(elevatorLeader);

    // Brake mode helps hold the elevator in place
    elevatorLeader.setNeutralMode(NeutralMode.Brake);
    elevatorFollower.setNeutralMode(NeutralMode.Brake);

    // Read the absolute analog sensor and seed the talon position
    elevatorLeader.setSelectedSensorPosition(metersToMotorPosition(getElevatorAnalogPositionMeters()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Analog Position Raw", getElevatorAnalogRawPosition());
    SmartDashboard.putNumber("Elevator Analog Position Meters", getElevatorAnalogPositionMeters());
    SmartDashboard.putNumber("Elevator Motor Position Raw", elevatorLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Motor Position Meters", getElevatorPosition());
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
    // TODO should feed forward go in here, or use kF?
    // elevatorLeader.set(TalonFXControlMode.MotionMagic, metersToMotorPosition(meters));
  }
  
  /**
   * Get the elevator position in meters
   * @return position in meters
   */
  public double getElevatorPosition() {
    return motorPositionToMeters(elevatorLeader.getSelectedSensorPosition());
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    elevatorLeader.stopMotor();
  }

  private double getElevatorAnalogRawPosition() {
    return 4096 - analogSensor.getValue(); // Invert sensor so up is positive
  }

  private double getElevatorAnalogPositionMeters() {
    return (getElevatorAnalogRawPosition() - ANALOG_BOTTOM) * ANALOG_SENSOR_COEFFICIENT;
  }

  static double motorPositionToMeters(double motorPosition) {
    return (motorPosition * MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  static double metersToMotorPosition(double positionMeters) {
    return (positionMeters / MOTOR_ENCODER_POSITION_COEFFICIENT);
  }
  
}