package frc.robot.subsystems;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;
import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;
import static edu.wpi.first.math.util.Units.radiansToRotations;
import static frc.robot.Constants.WristConstants.WRIST_PARK_HEIGHT;

import java.util.Map;
import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

/**
 * Subsystem for wrist mechanism
 */
public class WristSubsystem extends SubsystemBase {

  // Offset in rotations to add to encoder value - offset from arm horizontal to sensor zero
  private static final double ENCODER_OFFSET = -0.53759d;
  
  private static final float LIMIT_BOTTOM = 0.5366f;
  private static final float LIMIT_TOP = 0.77f;
  private static final double LIMIT_TOP_RADIANS = Units.rotationsToRadians(LIMIT_TOP + ENCODER_OFFSET);
  private static final double LIMIT_BOTTOM_RADIANS = Units.rotationsToRadians(LIMIT_BOTTOM + ENCODER_OFFSET);

  // Profile constraints, in radians per second
  private static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(40, 15);
  private static final double GRAVITY_FEED_FORWARD = .04;

  private final CANSparkMax wristLeader;

  private final SparkMaxPIDController pidController;
  private final SparkMaxAbsoluteEncoder wristEncoder;
  
  private TrapezoidProfile.State goal = null;
  private TrapezoidProfile trapezoidProfile;
  private double profileTime = 0.0;
  
  public WristSubsystem() {
    wristLeader = new CANSparkMax(WristConstants.WRIST_LEADER_ID, MotorType.kBrushless);

    wristLeader.restoreFactoryDefaults();
    
    // Get the through-bore-encoder absolute encoder
    wristEncoder = wristLeader.getAbsoluteEncoder(kDutyCycle);
    wristEncoder.setInverted(true);
    wristEncoder.setAverageDepth(64);
    pidController = wristLeader.getPIDController();
    pidController.setFeedbackDevice(wristEncoder);

    // Configure closed-loop control
    double kP = 3.9; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0;
    double kMaxOutput = .4;
    double kMinOutput = -.3;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    pidController.setPositionPIDWrappingMaxInput(1);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingEnabled(true);

    // Voltage compensation and current limits
    wristLeader.enableVoltageCompensation(12);
    // wristLeader.setSmartCurrentLimit(30);

    // Configure soft limits
    wristLeader.setSoftLimit(kForward, LIMIT_TOP);
    wristLeader.setSoftLimit(kReverse, LIMIT_BOTTOM);
    wristLeader.enableSoftLimit(kForward, true);
    wristLeader.enableSoftLimit(kReverse, true);

    // Disable limit switches, we don't have any
    wristLeader.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    wristLeader.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    
    // Brake mode helps hold wrist in place
    wristLeader.setIdleMode(IdleMode.kBrake);

    wristLeader.setInverted(true);

    // Save settings to motor flash, so they persist between power cycles
    wristLeader.burnFlash();
    
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    layout.addNumber("Position Radians", this::getWristPosition).withPosition(0, 0);
    layout.addNumber("Target Position", this::getGoalPosition).withPosition(0, 1);
    layout.addNumber("Position Raw", wristEncoder::getPosition).withPosition(0, 2);
  }
  
  @Override
  public void periodic() {
    if (goal != null) {
      var targetState = trapezoidProfile.calculate(profileTime += 0.02);
      double cosineScalar = Math.cos(getWristPosition());
      double feedForward = GRAVITY_FEED_FORWARD * cosineScalar;

      pidController.setReference(
          armRadiansToEncoderRotations(targetState.position),
          ControlType.kPosition,
          0,
          feedForward,
          ArbFFUnits.kPercentOut);
    }
  }

  private double getGoalPosition() {
    return goal == null ? 0 : goal.position;
  }

  /**
   * Moves the wrist using duty cycle. There is no motor safety, so calling this will continue to move until another
   * method is called.
   * @param speed duty cycle [-1,1]
   */
  public void moveWrist(double speed){
    goal = null;
    wristLeader.set(speed);
  }

  /**
   * Moves the elevator to a position. Zero is horizontal, up is positive. There is no motor safety, so calling this
   * will continue to move to this position, and hold it until another method is called.
   * @param radians position in radians
   */
  public void moveToPosition(double radians) {
    var target = MathUtil.clamp(radians, LIMIT_BOTTOM_RADIANS, LIMIT_TOP_RADIANS);
    // Set the target position, but move in execute() so feed forward keeps updating
    var newGoal = new TrapezoidProfile.State(target, 0.0);
    if (!Objects.equals(newGoal, goal)) {
      var currentState = new TrapezoidProfile.State(getWristPosition(), 0);
      trapezoidProfile = new TrapezoidProfile(PROFILE_CONSTRAINTS, newGoal, currentState);
      profileTime = 0;
    }
    goal = newGoal;
  }

  /**
   * Gets the wrist position Zero is horizontal, up is positive
   * @return position in radians
   */
  public double getWristPosition() {
    return Units.rotationsToRadians(wristEncoder.getPosition() + ENCODER_OFFSET);
  }

  /**
   * Gets the wrist velocity in radians per second
   * @return wrist velocity in radians per second
   */
  public double getWristVelocity() {
    return Units.rotationsToDegrees(wristEncoder.getVelocity());
  }

  /**
   * Moves the wrist to the park/transit position.
   */
  public void parkWrist() {
    moveToPosition(WRIST_PARK_HEIGHT);
  }

  /**
   * Returns true if the wrist is in the park position, within a tolerance.
   * @return true if wrist is parked
   */
  public boolean isParked() {
    return Math.abs(getWristPosition() - WRIST_PARK_HEIGHT) < .2;
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
    goal = null;
    wristLeader.stopMotor();
  }

}