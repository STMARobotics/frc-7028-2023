package frc.robot.subsystems;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;
import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;
import static edu.wpi.first.math.util.Units.radiansToRotations;
import static edu.wpi.first.math.util.Units.rotationsToRadians;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

/**
 * Subsystem for wrist mechanism
 */
public class WristSubsystem extends SubsystemBase {

  private static final int SMART_MOTION_SLOT = 0;
  // Offset in rotations to add to encoder value - offset from arm horizontal to sensor zero
  private static final double ENCODER_OFFSET = -0.59117d; // TODO: Negated position measured when arm is horizontal
  private static final double GRAVITY_FF = 0.07; // TODO find this
  private static final float LIMIT_BOTTOM = 0.59117f;
  private static final float LIMIT_TOP = 0.87317f;

  private final CANSparkMax wristLeader;
  private final CANSparkMax wristFollower;

  private final SparkMaxPIDController pidController;
  private final SparkMaxAbsoluteEncoder wristEncoder;

  double maxGravityFF = 0.07;

  public WristSubsystem() {
    wristLeader = new CANSparkMax(WristConstants.WRIST_LEADER_ID, MotorType.kBrushless);
    wristFollower = new CANSparkMax(WristConstants.WRIST_FOLLOWER_ID, MotorType.kBrushless);

    wristLeader.restoreFactoryDefaults();
    wristFollower.restoreFactoryDefaults();
    
    // Get the through-bore-encoder absolute encoder
    wristEncoder = wristLeader.getAbsoluteEncoder(kDutyCycle);
    pidController = wristLeader.getPIDController();
    pidController.setFeedbackDevice(wristEncoder);

    // Configure closed-loop control
    double kP = .00005; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000156;
    double kMaxOutput = .2;  // TODO safe values that can be increased when confident
    double kMinOutput = -.2;
    double allowedErr = 0;

    // Smart Motion Coefficients
    double maxVel = 2000; // rpm
    double maxAcc = 1500;
    double minVel = 0;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput); 

    pidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    pidController.setSmartMotionMinOutputVelocity(minVel, SMART_MOTION_SLOT);
    pidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);

    // Voltage compensation and current limits
    wristLeader.enableVoltageCompensation(12);
    wristLeader.setSmartCurrentLimit(20);
    wristFollower.setSmartCurrentLimit(20);

    // Configure soft limits
    wristLeader.setSoftLimit(kForward, 0.76f); // LIMIT_TOP);
    wristLeader.setSoftLimit(kReverse, LIMIT_BOTTOM);
    wristLeader.enableSoftLimit(kForward, true);
    wristLeader.enableSoftLimit(kReverse, true);

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
    SmartDashboard.putNumber("Wrist Position Raw", wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Position Radians", getWristPosition());
    SmartDashboard.putNumber("Wrist RPM (?)", wristLeader.getEncoder().getVelocity());
  }

  /**
   * Moves the wrist using duty cycle
   * @param speed duty cycle [-1,1]
   */
  public void moveWrist(double speed){
    wristLeader.set(speed);
  }

  /**
   * Moves the elevator to a position. Zero is horizontal, up is positive
   * @param radians position in radians
   */
  public void moveToPosition(double radians) {
    double positionRotations = getWristPosition();
    double cosineScalar = Math.cos(rotationsToRadians(positionRotations));
    double feedForward = GRAVITY_FF * cosineScalar;

    pidController.setReference(armRadiansToEncoderRotations(radians), ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
  }

  /**
   * Gets the wrist position Zero is horizontal, up is positive
   * @return position in radians
   */
  public double getWristPosition() {
    return wristEncoder.getPosition() + ENCODER_OFFSET;
  }

  /**
   * Convert from arm position in radians to encoder rotations
   * @param armRadians arm position in radians
   * @return equivilant encoder position, in rotations
   */
  static double armRadiansToEncoderRotations(double armRadians) {
    return radiansToRotations(armRadians) - ENCODER_OFFSET;
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    wristLeader.stopMotor();
  }

}