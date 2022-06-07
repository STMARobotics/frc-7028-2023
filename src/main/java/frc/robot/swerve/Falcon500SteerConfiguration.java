package frc.robot.swerve;

import java.util.Objects;

public class Falcon500SteerConfiguration {
  private final int motorPort;
  private final CanCoderAbsoluteConfiguration encoderConfiguration;

  public Falcon500SteerConfiguration(int motorPort, CanCoderAbsoluteConfiguration encoderConfiguration) {
    this.motorPort = motorPort;
    this.encoderConfiguration = encoderConfiguration;
  }

  public int getMotorPort() {
    return motorPort;
  }

  public CanCoderAbsoluteConfiguration getEncoderConfiguration() {
    return encoderConfiguration;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o)
      return true;
    if (o == null || getClass() != o.getClass())
      return false;
    Falcon500SteerConfiguration that = (Falcon500SteerConfiguration) o;
    return getMotorPort() == that.getMotorPort() && getEncoderConfiguration().equals(that.getEncoderConfiguration());
  }

  @Override
  public int hashCode() {
    return Objects.hash(getMotorPort(), getEncoderConfiguration());
  }

  @Override
  public String toString() {
    return "Falcon500SteerConfiguration{" +
        "motorPort=" + motorPort +
        ", encoderConfiguration=" + encoderConfiguration +
        '}';
  }
}
