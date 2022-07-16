package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class Falcon500SteerController {

  private static final int ENCODER_RESET_ITERATIONS = 500;
  private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private static final double TICKS_PER_ROTATION = 2048.0;
  private static final int CAN_TIMEOUT_MS = 250;

  private final WPI_TalonFX motor;
  private final double motorEncoderPositionCoefficient;
  private final double motorEncoderVelocityCoefficient;
  private final CANCoderAbsoluteEncoder absoluteEncoder;

  // Not sure what these represent, but smaller is faster
  private final double motionMagicVelocityConstant = .125;
  private final double motionMagicAccelerationConstant = .125;

  private double referenceAngleRadians = 0.0;

  private double resetIteration = 0;

  private void addDashboardEntries(ShuffleboardContainer container, Falcon500SteerController controller) {
    container.addNumber("Current Angle", () -> Math.toDegrees(controller.getStateAngle()));
    container.addNumber("Target Angle", () -> Math.toDegrees(controller.getReferenceAngle()));
    container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(absoluteEncoder.getAbsoluteAngle()));
  }

  public Falcon500SteerController(
      ShuffleboardContainer container, 
      Falcon500SteerConfiguration steerConfiguration,
      ModuleConfiguration moduleConfiguration) {
    
    absoluteEncoder = new CANCoderAbsoluteEncoder(steerConfiguration.getEncoderConfiguration());

    motorEncoderPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getSteerReduction();
    motorEncoderVelocityCoefficient = motorEncoderPositionCoefficient * 10.0;

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.slot0.kP = 0.2;
    motorConfiguration.slot0.kI = 0.0;
    motorConfiguration.slot0.kD = 0.1;

    motorConfiguration.slot0.kF = (1023.0 * motorEncoderVelocityCoefficient / 12) * motionMagicVelocityConstant;
    motorConfiguration.motionCruiseVelocity = 2.0 / motionMagicVelocityConstant / motorEncoderVelocityCoefficient;
    motorConfiguration.motionAcceleration = (8.0 - 2.0) / motionMagicAccelerationConstant / motorEncoderVelocityCoefficient;

    motorConfiguration.voltageCompSaturation = 12;
    motorConfiguration.supplyCurrLimit.currentLimit = 20;
    motorConfiguration.supplyCurrLimit.enable = true;

    motor = new WPI_TalonFX(steerConfiguration.getMotorPort());
    CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS),
        "Failed to configure Falcon 500 settings");

    CtreUtils.checkCtreError(
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
        "Failed to set Falcon 500 feedback sensor");
    motor.enableVoltageCompensation(true);
    motor.setSensorPhase(true);
    motor.setInverted(
        moduleConfiguration.isSteerInverted() ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
    motor.setNeutralMode(NeutralMode.Brake);

    CtreUtils.checkCtreError(
      motor.setSelectedSensorPosition(absoluteEncoder.getAbsoluteAngle() / motorEncoderPositionCoefficient, 0, CAN_TIMEOUT_MS), 
      "Failed to set Falcon 500 encoder position");

    // Reduce CAN status frame rates
    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS),
        "Failed to configure Falcon status frame period");
    
    addDashboardEntries(container, this);

  }

  public double getReferenceAngle() {
    return referenceAngleRadians;
  }

  public void setReferenceAngle(double referenceAngleRadians) {
    double currentAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;

    // Reset the Falcon's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    if (motor.getSelectedSensorVelocity() * motorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
      if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        resetIteration = 0;
        double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
        motor.setSelectedSensorPosition(absoluteAngle / motorEncoderPositionCoefficient);
        currentAngleRadians = absoluteAngle;
      }
    } else {
      resetIteration = 0;
    }

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
    }

    // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
    // above that
    double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }
    motor.set(TalonFXControlMode.MotionMagic, adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);

    this.referenceAngleRadians = referenceAngleRadians;
  }

  public double getStateAngle() {
    double motorAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
    motorAngleRadians %= 2.0 * Math.PI;
    if (motorAngleRadians < 0.0) {
      motorAngleRadians += 2.0 * Math.PI;
    }

    return motorAngleRadians;
  }

}