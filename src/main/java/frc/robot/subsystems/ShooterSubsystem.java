package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_FOLLOWER_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEADER_ID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX shooterLeader;
  private final WPI_TalonFX shooterFollower;

  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.025103, 0.55611, 0.31053);

  public ShooterSubsystem() {
    var config = new TalonFXConfiguration();
    config.slot0.kP = .01;
    config.slot0.kI = 0;
    config.slot0.kD = 0;
    config.voltageCompSaturation = 12;

    shooterLeader = new WPI_TalonFX(SHOOTER_LEADER_ID, DrivetrainConstants.CANIVORE_BUS_NAME);
    shooterFollower = new WPI_TalonFX(SHOOTER_FOLLOWER_ID, DrivetrainConstants.CANIVORE_BUS_NAME);

    shooterLeader.configAllSettings(config);
    shooterFollower.configAllSettings(config);
    shooterLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    shooterLeader.enableVoltageCompensation(true);
    shooterFollower.enableVoltageCompensation(true);

    shooterLeader.setInverted(true);
    shooterFollower.follow(shooterLeader);
    shooterFollower.setInverted(InvertType.OpposeMaster);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Leader Speed Raw", shooterLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Follower Speed Raw", shooterFollower.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Leader Speed RPS", getVelocity());
  }

  public void shootVelocity(double rps) {
    var feedForwardVolts = feedForward.calculate(rps);

    shooterLeader.set(
        ControlMode.Velocity,
        rpsToedgesPerDecisec(rps),
        DemandType.ArbitraryFeedForward,
        feedForwardVolts / 12);
  }

  public void shootDutyCycle(double speed) {
    shooterLeader.set(speed);
  }

  /**
   * Gets the velocity in RPS
   * @return velocity in RPS
   */
  public double getVelocity() {
    return edgesPerDecisecToRPS(shooterLeader.getSelectedSensorVelocity());
  }

  public boolean hasCone() {
    return shooterLeader.isFwdLimitSwitchClosed() == 1;
  }

  public void stop() {
    shooterLeader.stopMotor();
  }

  public static double edgesPerDecisecToRPS(double edgesPerDecisec) {
    var rotationsPerDecisecond = edgesPerDecisec / 2048;
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
    var edgesPerSecond = rps * 2048;
    return edgesPerSecond / 10;
  }

}