package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem for elevator mechanism
 */
public class ElevatorSubsystem extends SubsystemBase {

  // Elevator travel distance, in meters
  private static final double ELEVATOR_HEIGHT = 1.002;

  // Motor's encoder limits, in encoder ticks
  private static final double MOTOR_BOTTOM = 0;
  private static final double MOTOR_TOP = 56530;

  // Mutiply by sensor position to get meters
  private static final double MOTOR_ENCODER_POSITION_COEFFICIENT = ELEVATOR_HEIGHT / (MOTOR_TOP - MOTOR_BOTTOM);

  private static final int ANALOG_BOTTOM = 758;
  private static final int ANALOG_TOP = 1796;

  private static final double GRAVITY_FEED_FORWARD = 0.05;

  // Mutiply by sensor position to get meters
  private static final double ANALOG_SENSOR_COEFFICIENT = ELEVATOR_HEIGHT / (ANALOG_TOP - ANALOG_BOTTOM);

  private final WPI_TalonFX elevatorLeader;
  private final WPI_TalonFX elevatorFollower;
  private final AnalogInput analogSensor;

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(8);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(9);

  private double targetPosition = 0;

  public ElevatorSubsystem() {
    elevatorLeader = new WPI_TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID);
    elevatorFollower = new WPI_TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID);
    
    // Configure potentiometer
    analogSensor = new AnalogInput(ElevatorConstants.ANALOG_SENSOR_CHANNEL);

    // Configure closed-loop control
    double kP = 0.13;
    double kI = 0;
    double kD = 0; 
    double kIz = 0;
    double kF = 0.00;
    double kMaxOutput = 0.7;
    double kMinOutput = -.2;
    double allowedErr = 1;

    // Magic Motion Coefficients
    double maxVel = 10000;
    double maxAcc = 30000;

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
    config.forwardSoftLimitThreshold = MOTOR_TOP - metersToMotorPosition(0.005);
    config.reverseSoftLimitEnable = true;
    config.reverseSoftLimitThreshold = MOTOR_BOTTOM + metersToMotorPosition(0.02);
    config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    config.neutralDeadband = .02;

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
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    layout.addNumber("Position Raw", elevatorLeader::getSelectedSensorPosition).withPosition(0, 0);
    layout.addNumber("Position Meters", this::getElevatorPosition).withPosition(0, 1);
    layout.addNumber("Target Position Meters", () -> targetPosition).withPosition(0, 2);
    var limitsLayout = layout.getLayout("Limits", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 1)).withPosition(0, 3).withSize(2,1);
    limitsLayout.addBoolean("Top Limit", this::isAtTopLimit).withPosition(0, 0);
    limitsLayout.addBoolean("Botton Limit", this::isAtBottomLimit).withPosition(1, 0);
  }

  @Override
  public void periodic() {
    // Handle elevator limit switches
    if (isAtBottomLimit()) {
      elevatorLeader.setSelectedSensorPosition(MOTOR_BOTTOM);
    } else if (isAtTopLimit()) {
      elevatorLeader.setSelectedSensorPosition(MOTOR_TOP);
    }
  }

  /**
   * Moves the elevator using duty cycle
   * @param speed duty cycle [-1, 1]
   */
  public void moveElevator(double speed) {
    targetPosition = 0;
    elevatorLeader.set(speed);
  }

  /**
   * Moves the elevator to a position
   * @param meters position in meters
   */
  public void moveToPosition(double meters) {
    targetPosition = meters;
    elevatorLeader.set(TalonFXControlMode.MotionMagic, metersToMotorPosition(meters),
        DemandType.ArbitraryFeedForward, GRAVITY_FEED_FORWARD);
  }
  
  /**
   * Get the elevator position in meters
   * @return position in meters
   */
  public double getElevatorPosition() {
    return motorPositionToMeters(elevatorLeader.getSelectedSensorPosition());
  }

  /**
   * Gets the position of the elevator top / first stage, where the Limelight is mounted.
   * This moves slower than the wrist / second stage.
   */
  public double getElevatorTopPosition() {
    return getElevatorPosition() * .489;
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    elevatorLeader.stopMotor();
  }

  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public boolean isAtTopLimit() {
    return !topLimitSwitch.get();
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