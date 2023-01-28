package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax wristMotor1;
  private final CANSparkMax wristMotor2;

  private final SparkMaxAbsoluteEncoder wristEncoder;


  public WristSubsystem() {
    wristMotor1 = new CANSparkMax(WristConstants.WRIST_LEADER_ID, MotorType.kBrushless);
    wristMotor2 = new CANSparkMax(WristConstants.WRIST_FOLLOWER_ID, MotorType.kBrushless);

    wristMotor1.restoreFactoryDefaults();
    wristMotor2.restoreFactoryDefaults();

    wristMotor2.follow(wristMotor1, true);

    wristEncoder = wristMotor1.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
  }

  public void wristUp(double speed){
    speed = MathUtil.clamp(speed, 0, 1);
    wristMotor1.set(0);
  }
  
  public void wristDown(double speed){
    speed = MathUtil.clamp(speed, -1, 0);
    wristMotor1.set(0);
  }

  public void moveWrist(double speed) {
    wristMotor1.set(speed);
  }

  public void stop() {
    wristMotor1.stopMotor();
  }

}