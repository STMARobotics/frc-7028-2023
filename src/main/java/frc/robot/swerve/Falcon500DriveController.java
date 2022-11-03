package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Falcon500DriveController {

  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private static final int CAN_TIMEOUT_MS = 250;
  private static final double TICKS_PER_ROTATION = 2048.0;

  private final WPI_TalonFX motor;

  private final double sensorPositionCoefficient;
  private final double sensorVelocityCoefficient;
  private final double nominalVoltage = 12.0;

  /** Voltage needed to overcome the motor’s static friction. kS */
  public static final double kS = 0.6716;
  /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
  public static final double kV = 2.5913;
  /** Voltage needed to induce a given acceleration in the motor shaft. kA */
  public static final double kA = 0.19321;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  public Falcon500DriveController(int port, ModuleConfiguration moduleConfiguration) {
    sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
        * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
    sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.voltageCompSaturation = 12;
    motorConfiguration.supplyCurrLimit.currentLimit = 80;
    motorConfiguration.supplyCurrLimit.enable = true;

    motorConfiguration.slot0.kP = 0.00565;
    motorConfiguration.slot0.kI = 0.0;
    motorConfiguration.slot0.kD = 0.0;

    motor = new WPI_TalonFX(port);
    CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");
    motor.enableVoltageCompensation(true);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.setInverted(
        moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
    motor.setSensorPhase(true);
    motor.setSafetyEnabled(true);

    // Reduce CAN status frame rates
    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS),
        "Failed to configure Falcon status frame period");
  }

  public void setReferenceVoltage(double voltage) {
    motor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
  }

  public void setReferenceVelocity(double velocity) {
    var arbFeedForward = feedforward.calculate(velocity) / nominalVoltage;
    motor.set(TalonFXControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, arbFeedForward);
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
