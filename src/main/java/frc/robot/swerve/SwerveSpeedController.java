package frc.robot.swerve;

import static frc.robot.Constants.DrivetrainConstants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kD;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kI;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kP;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class SwerveSpeedController {

  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private static final int CAN_TIMEOUT_MS = 250;
  private static final double TICKS_PER_ROTATION = 2048.0;

  private final WPI_TalonFX motor;

  private final double sensorPositionCoefficient;
  private final double sensorVelocityCoefficient;
  private final double nominalVoltage = 12.0;

  private final SimpleMotorFeedforward feedforward;

  private double referenceVelocity;

  public SwerveSpeedController(int port, ModuleConfiguration moduleConfiguration, ShuffleboardContainer container,
      double kS, double kV, double kA) {
    
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    
    sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
        * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
    sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.voltageCompSaturation = 12;
    motorConfiguration.supplyCurrLimit.currentLimit = 80;
    motorConfiguration.supplyCurrLimit.enable = true;

    motorConfiguration.slot0.kP = DRIVE_kP;
    motorConfiguration.slot0.kI = DRIVE_kI;
    motorConfiguration.slot0.kD = DRIVE_kD;

    motor = new WPI_TalonFX(port, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");
    motor.enableVoltageCompensation(true);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.setInverted(moduleConfiguration.isDriveInverted());
    motor.setSensorPhase(true);
    motor.setSafetyEnabled(true);

    // Reduce CAN status frame rates
    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS),
        "Failed to configure Falcon status frame period");
    
    addDashboardEntries(container);
  }

  private void addDashboardEntries(ShuffleboardContainer container) {
    if (container != null) {
      container.addNumber("Current Velocity", () -> getStateVelocity()).withPosition(0, 0);
      container.addNumber("Target Velocity", () -> referenceVelocity).withPosition(0, 1);
      container.addNumber("Current Position", () -> getStatePosition()).withPosition(0, 2);
    }
  }

  public void setReferenceVelocity(double velocity) {
    this.referenceVelocity = velocity;
    var arbFeedForward = feedforward.calculate(velocity) / nominalVoltage;
    motor.set(
          TalonFXControlMode.Velocity,
          velocity / sensorVelocityCoefficient,
          DemandType.ArbitraryFeedForward,
          arbFeedForward);
    motor.feed();
  }

  /**
   * Returns velocity in meters per second
   * @return drive velocity in meters per second
   */
  public double getStateVelocity() {
    return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
  }

  /**
   * Returns position in meters
   * @return position in meters
   */
  public double getStatePosition() {
    return motor.getSelectedSensorPosition() * sensorPositionCoefficient;
  }

  /**
   * Sets the neutral mode for the drive motor
   * @param neutralMode neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    motor.setNeutralMode(neutralMode);
  }

}
