package frc.robot.subsystems;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;
import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

/**
 * Subsystem for wrist mechanism
 */
public class WristSubsystem extends SubsystemBase {

  private final CANSparkMax wristLeader;
  private final CANSparkMax wristFollower;

  private final SparkMaxAbsoluteEncoder wristEncoder;

  public WristSubsystem() {
    wristLeader = new CANSparkMax(WristConstants.WRIST_LEADER_ID, MotorType.kBrushless);
    wristFollower = new CANSparkMax(WristConstants.WRIST_FOLLOWER_ID, MotorType.kBrushless);

    wristLeader.restoreFactoryDefaults();
    wristFollower.restoreFactoryDefaults();
    
    // Get the through-bore-encoder absolute encoder
    wristEncoder = wristLeader.getAbsoluteEncoder(kDutyCycle);
    
    // Configure soft limits
    wristLeader.setSoftLimit(kForward, 0.76f); // real limit was 0.87317
    wristLeader.setSoftLimit(kReverse, 0.59117f);
    wristLeader.enableSoftLimit(kForward, true);
    wristLeader.enableSoftLimit(kReverse, true);
    wristLeader.getPIDController().setFeedbackDevice(wristEncoder);

    // Disable limit switches, we don't have any
    wristLeader.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    wristLeader.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    
    wristFollower.follow(wristLeader, true);

    // Brake mode helps hold wrist in place
    wristLeader.setIdleMode(IdleMode.kBrake);
    wristFollower.setIdleMode(IdleMode.kBrake);

    // Save settings to motor flash, so they persist between power cycles
    wristLeader.burnFlash();
    wristFollower.burnFlash();
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
  }

  public void moveWrist(double speed){
    wristLeader.set(speed);
  }

  public void stop() {
    wristLeader.stopMotor();
  }

}