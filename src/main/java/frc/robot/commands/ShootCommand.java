package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Basic command to position elevator and wrist, and then shoot
 */
public class ShootCommand extends CommandBase {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double AIM_TOLERANCE = 1.0;
  private static final double DISTANCE_TOLERANCE = 0.15;
  private static final double SHOOT_TIME = 1.0;
  private static final double TARGET_Y_SETPOINT = 4.2; // degrees in the limelight top target

  private final double elevatorMeters;
  private final double wristRadians;
  private final double shooterRPS;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final Timer shootTimer = new Timer();

  private final TrapezoidProfile.Constraints kThetaControllerConstraints = 
  new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);

  private final ProfiledPIDController aimController = 
      new ProfiledPIDController(2.0, 0, 0, kThetaControllerConstraints);
  
  private final PIDController distanceController = new PIDController(.5, 0, 0);

  private boolean isShooting = false;

  /**
   * Constructor
   * @param elevatorMeters position of elevator in meters
   * @param wristRadians position of wrist in radians
   * @param shooterRPS velocity of shooter in rotations per second
   * @param drivetrainSubsystem drivetrain
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   */
  public ShootCommand(double elevatorMeters, double wristRadians, double shooterRPS,
      DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    isShooting = false;
    aimController.reset(drivetrainSubsystem.getGyroscopeRotation().getRadians());
    distanceController.setSetpoint(TARGET_Y_SETPOINT);
    LimelightHelpers.setPipelineIndex("limelight", 1);
  }

  @Override
  public void execute() {
    // TODO make sure we see a target
    double targetX = LimelightHelpers.getTX("limelight");
    double targetY = LimelightHelpers.getTY("limelight");

    if (!isShooting) {
      var drivetrainHeading = drivetrainSubsystem.getGyroscopeRotation();
      var targetHeadingRadians = drivetrainHeading.getRadians() - Units.degreesToRadians(targetX);
      
      aimController.setGoal(targetHeadingRadians);
      var rotationCorrection = 
          Math.abs(targetX) > AIM_TOLERANCE ? aimController.calculate(drivetrainHeading.getRadians()) : 0;
      var distanceCorrection =
          Math.abs(targetY - TARGET_Y_SETPOINT) > DISTANCE_TOLERANCE ? distanceController.calculate(targetY) : 0;

      drivetrainSubsystem.drive(new ChassisSpeeds(distanceCorrection, 0, rotationCorrection));
    }

    var readyToShoot =
        Math.abs(elevatorSubsystem.getElevatorPosition() - elevatorMeters) < ELEVATOR_TOLERANCE
        && Math.abs(wristSubsystem.getWristPosition() - wristRadians) < WRIST_TOLERANCE
        && Math.abs(targetX) < AIM_TOLERANCE
        && Math.abs(targetY - TARGET_Y_SETPOINT) < DISTANCE_TOLERANCE;

    if (isShooting || readyToShoot) {
      shooterSubsystem.shootVelocity(shooterRPS);
      shootTimer.start();
      isShooting = true;
    } else {
      elevatorSubsystem.moveToPosition(elevatorMeters);
      wristSubsystem.moveToPosition(wristRadians);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
    // return isShooting && shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    wristSubsystem.stop();
    shooterSubsystem.stop();
  }

}
