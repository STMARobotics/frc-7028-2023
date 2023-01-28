package frc.robot.swerve;

import static frc.robot.Constants.DrivetrainConstants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.DrivetrainConstants.STEER_kD;
import static frc.robot.Constants.DrivetrainConstants.STEER_kI;
import static frc.robot.Constants.DrivetrainConstants.STEER_kP;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class SwerveSteerController {

  private static final double ENCODER_RESEED_SECONDS = 10.0;
  private static final double ENCODER_RESEED_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private static final double TICKS_PER_ROTATION = 2048.0;
  private static final int CAN_TIMEOUT_MS = 250;

  private final WPI_TalonFX motor;
  private final double motorEncoderPositionCoefficient;
  private final double motorEncoderVelocityCoefficient;
  private final CANCoder encoder;
  
  // Not sure what these represent, but smaller is faster
  private final double motionMagicVelocityConstant = .125;
  private final double motionMagicAccelerationConstant = .0625;

  private double desiredAngleRadians = 0.0;
  private Timer reseedTimer = new Timer();

  public SwerveSteerController(
      int motorPort,
      int canCoderPort,
      double canCoderOffset,
      ShuffleboardContainer container, 
      ModuleConfiguration moduleConfiguration) {

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = Math.toDegrees(canCoderOffset);
    config.sensorDirection = false;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    encoder = new CANCoder(canCoderPort, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");
    CtreUtils.checkCtreError(encoder.setPositionToAbsolute(250), "Failed to set CANCoder to absolute");

    CtreUtils.checkCtreError(
        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250),
        "Failed to configure CANCoder update rate");

    // Configure Motor
    motorEncoderPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getSteerReduction();
    motorEncoderVelocityCoefficient = motorEncoderPositionCoefficient * 10.0;

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.slot0.kP = STEER_kP;
    motorConfiguration.slot0.kI = STEER_kI;
    motorConfiguration.slot0.kD = STEER_kD;

    motorConfiguration.slot0.kF = (1023.0 * motorEncoderVelocityCoefficient / 12) * motionMagicVelocityConstant;
    motorConfiguration.motionCruiseVelocity = 2.0 / motionMagicVelocityConstant / motorEncoderVelocityCoefficient;
    motorConfiguration.motionAcceleration = (8.0 - 2.0) / motionMagicAccelerationConstant / motorEncoderVelocityCoefficient;

    motorConfiguration.voltageCompSaturation = 12;
    motorConfiguration.supplyCurrLimit.currentLimit = 20;
    motorConfiguration.supplyCurrLimit.enable = true;

    motor = new WPI_TalonFX(motorPort, CANIVORE_BUS_NAME);
    CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS),
        "Failed to configure Falcon 500 settings");

    CtreUtils.checkCtreError(
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
        "Failed to set Falcon 500 feedback sensor");
    motor.enableVoltageCompensation(true);
    motor.setSensorPhase(true);
    motor.setInverted(
        moduleConfiguration.isSteerInverted() ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
    motor.setNeutralMode(NeutralMode.Coast);

    configMotorOffset(true);

    // Reduce CAN status frame rates
    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS),
        "Failed to configure Falcon status frame period");
    
    addDashboardEntries(container);
    reseedTimer.start();

  }

  private void addDashboardEntries(ShuffleboardContainer container) {
    if (container != null) {
      container.addNumber("Current Angle", () -> getStateRotation().getDegrees());
      container.addNumber("Target Angle", () -> Math.toDegrees(desiredAngleRadians));
      container.addNumber("Absolute Encoder Angle", () -> encoder.getAbsolutePosition());
    }
  }

  /**
   * Configures the motor offset from the CANCoder's abosolute position. In an ideal state, this only needs to happen
   * once. However, sometime it fails and we end up with a wheel that isn't in the right position.
   * See https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99
   * @return the absolute angle
   */
  public double configMotorOffset(boolean logErrors) {
    double angle = getAbsoluteAngle();
    var angleErrorCode = encoder.getLastError();

    if (angleErrorCode == ErrorCode.OK) {
      motor.setSelectedSensorPosition(angle / motorEncoderPositionCoefficient, 0, CAN_TIMEOUT_MS);
    }
    return angle;
  }

  private double getAbsoluteAngle() {
    double angle = encoder.getPosition();

    angle = Math.toRadians(angle);
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    return angle;
  }

  public void setDesiredRotation(Rotation2d desiredRotation) {
    var desiredAngleRadians = desiredRotation.getRadians();
    double currentAngleRadians;

    // Reset the Falcon's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    if (reseedTimer.advanceIfElapsed(ENCODER_RESEED_SECONDS) && 
        motor.getSelectedSensorVelocity() * motorEncoderVelocityCoefficient < ENCODER_RESEED_MAX_ANGULAR_VELOCITY) {
      currentAngleRadians = configMotorOffset(false);
    } else {
      currentAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
    }

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
    }

    // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
    // above that
    double adjustedReferenceAngleRadians = desiredAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (desiredAngleRadians - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (desiredAngleRadians - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }
    motor.set(TalonFXControlMode.MotionMagic, adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);

    this.desiredAngleRadians = desiredAngleRadians;
  }

  public Rotation2d getStateRotation() {
    double motorAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
    motorAngleRadians %= 2.0 * Math.PI;
    if (motorAngleRadians < 0.0) {
      motorAngleRadians += 2.0 * Math.PI;
    }

    return new Rotation2d(motorAngleRadians);
  }

  /**
   * Sets the neutral mode for the steer motor
   * @param neutralMode neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    motor.setNeutralMode(neutralMode);
  }

}