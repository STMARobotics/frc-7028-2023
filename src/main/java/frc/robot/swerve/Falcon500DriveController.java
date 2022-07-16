package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Falcon500DriveController {

  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private static final int CAN_TIMEOUT_MS = 250;
  private static final double TICKS_PER_ROTATION = 2048.0;

  private final WPI_TalonFX motor;

  private final double sensorPositionCoefficient;
  private final double sensorVelocityCoefficient;
  private final double nominalVoltage = 12.0;

  public Falcon500DriveController(int port, ModuleConfiguration moduleConfiguration) {
    sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
        * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
    sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.voltageCompSaturation = 12;
    motorConfiguration.supplyCurrLimit.currentLimit = 80;
    motorConfiguration.supplyCurrLimit.enable = true;

    motor = new WPI_TalonFX(port);
    CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");
    motor.enableVoltageCompensation(true);
    motor.setNeutralMode(NeutralMode.Brake);
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

  public double getStateVelocity() {
    return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
  }

}
