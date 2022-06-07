package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

public class CANCoderAbsoluteEncoder {
  private final CANCoder encoder;

  public CANCoderAbsoluteEncoder(CanCoderAbsoluteConfiguration configuration) {
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
    config.sensorDirection = false;

    encoder = new CANCoder(configuration.getId());
    CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

    CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250), "Failed to configure CANCoder update rate");
  }

  public double getAbsoluteAngle() {
      double angle = Math.toRadians(encoder.getAbsolutePosition());
      angle %= 2.0 * Math.PI;
      if (angle < 0.0) {
          angle += 2.0 * Math.PI;
      }

      return angle;
  }
}

