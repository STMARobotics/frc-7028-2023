package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_FOLLOWER_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEADER_ID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  private static final double ENCODER_CPR = 2048;

  private final WPI_TalonFX shooterRight;
  private final WPI_TalonFX shooterLeft;

  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.21411, 1.1101 / 10.0, 0.022082);
  private final double VELOCITY_COEFFIENT = 1 / (5.0 * Math.PI * Units.inchesToMeters(4.0)); // gearing * wheel circumfrence

  public ShooterSubsystem() {
    var config = new TalonFXConfiguration();
    config.slot0.kP = 0.03;
    config.slot0.kI = 0;
    config.slot0.kD = 0;
    config.slot1.kP = 0.3;
    config.slot1.kI = 0;
    config.slot1.kD = 0;
    config.slot1.allowableClosedloopError = 0;
    config.neutralDeadband = 0;
    config.voltageCompSaturation = 12;

    shooterRight = new WPI_TalonFX(SHOOTER_LEADER_ID);
    shooterLeft = new WPI_TalonFX(SHOOTER_FOLLOWER_ID);

    shooterRight.configAllSettings(config);
    shooterLeft.configAllSettings(config);
    shooterRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    shooterLeft.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    shooterRight.enableVoltageCompensation(true);
    shooterLeft.enableVoltageCompensation(true);

    shooterLeft.setInverted(true);

    shooterRight.setNeutralMode(NeutralMode.Brake);
    shooterLeft.setNeutralMode(NeutralMode.Brake);
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.addNumber("Right Velocity Raw", shooterRight::getSelectedSensorVelocity);
    layout.addNumber("Left Velocity Raw", shooterLeft::getSelectedSensorVelocity);
    layout.addNumber("Velocity RPS", this::getVelocity);
    layout.addBoolean("Has Cone", this::hasCone);
    layout.addBoolean("Has Cube", this::hasCube);
  }

  public void shootVelocity(double rps) {
    var feedForwardVolts = feedForward.calculate(rps);

    shooterRight.selectProfileSlot(0, 0);
    shooterRight.set(
        ControlMode.Velocity,
        rpsToedgesPerDecisec(rps),
        DemandType.ArbitraryFeedForward,
        feedForwardVolts / 12);

    shooterLeft.selectProfileSlot(0, 0);
    shooterLeft.set(
        ControlMode.Velocity,
        rpsToedgesPerDecisec(rps),
        DemandType.ArbitraryFeedForward,
        feedForwardVolts / 12);
  }

  public void activeStop() {
    shooterRight.selectProfileSlot(1, 0);
    shooterRight.set(ControlMode.Position, shooterRight.getSelectedSensorPosition());
    shooterLeft.selectProfileSlot(1, 0);
    shooterLeft.set(ControlMode.Position, shooterLeft.getSelectedSensorPosition());
  }

  public void shootDutyCycle(double speed) {
    shooterRight.set(speed);
    shooterLeft.set(speed);
  }

  /**
   * Gets the velocity in RPS
   * @return velocity in RPS
   */
  public double getVelocity() {
    return edgesPerDecisecToRPS(shooterRight.getSelectedSensorVelocity());
  }

  /**
   * Get the surface velocity of the shooter wheels in meters per second.
   * @return shooter surface velocity in meters per second
   */
  public double getVelocityInMetersPerSecond() {
    return (getVelocity() * VELOCITY_COEFFIENT);
  }

  public boolean hasCone() {
    return shooterRight.isRevLimitSwitchClosed() == 1;
  }

  public boolean hasCube() {
    return shooterLeft.isRevLimitSwitchClosed() == 1;
  }

  public void stop() {
    shooterRight.stopMotor();
    shooterLeft.stopMotor();
  }

  public static double edgesPerDecisecToRPS(double edgesPerDecisec) {
    var rotationsPerDecisecond = edgesPerDecisec / ENCODER_CPR;
    return rotationsPerDecisecond * 10;
  }

  /**
   * Converts from rotations per second to edges per decisecond.
   * 
   * @param rps
   *          velocity in RPM
   * @return velocity in edges per decisecond (native talon units)
   */
  public static double rpsToedgesPerDecisec(double rps) {
    var edgesPerSecond = rps * ENCODER_CPR;
    return edgesPerSecond / 10;
  }

}