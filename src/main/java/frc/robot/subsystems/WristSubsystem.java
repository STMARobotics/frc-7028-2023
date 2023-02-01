package frc.robot.subsystems;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;
import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax wristLeader;
  private final CANSparkMax wristFollower;

  private final SparkMaxAbsoluteEncoder wristEncoder;


  public WristSubsystem() {
    wristLeader = new CANSparkMax(WristConstants.WRIST_LEADER_ID, MotorType.kBrushless);
    wristFollower = new CANSparkMax(WristConstants.WRIST_FOLLOWER_ID, MotorType.kBrushless);

    wristLeader.restoreFactoryDefaults();
    wristFollower.restoreFactoryDefaults();

    wristLeader.enableSoftLimit(kForward, false);
    wristLeader.enableSoftLimit(kReverse, false);
    wristLeader.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    wristLeader.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);

    wristFollower.follow(wristLeader, true);

    wristLeader.burnFlash();
    wristFollower.burnFlash();
    
    wristEncoder = wristLeader.getAbsoluteEncoder(kDutyCycle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
  }

  public void moveWrist(double speed){
    speed = MathUtil.applyDeadband(speed, 0.05);
    wristLeader.set(speed);
  }

  public void stop() {
    wristLeader.stopMotor();
  }

}