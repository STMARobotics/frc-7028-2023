package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeLimelightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to drive within range, turn to target, position elevator and wrist, and then shoot
 */
public class ShootCommand extends CommandBase {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double AIM_TOLERANCE = 1.0;
  private static final double DISTANCE_TOLERANCE = 0.15;
  private static final double SHOOT_TIME = 1.0;
  private static final double TARGET_Y_SETPOINT = 4.2; // degrees in the limelight top target

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ConeLimelightSubsystem limelightSubsystem;
  private final Timer shootTimer = new Timer();

  private final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);
  private final ProfiledPIDController aimController = new ProfiledPIDController(2.0, 0, 0, kThetaControllerConstraints);
  private final PIDController distanceController = new PIDController(.5, 0, 0);

  private final Profile limelightProfile;
  protected double elevatorMeters;
  protected double wristRadians;
  protected double shooterRPS;

  private boolean isShooting = false;

  /**
   * Constructor
   * @param elevatorMeters position of elevator in meters
   * @param wristRadians position of wrist in radians
   * @param shooterRPS velocity of shooter in rotations per second
   * @param limelightProfile limelight profile
   * @param drivetrainSubsystem drivetrain
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   * @param limelightSubssystem limelight
   */
  public ShootCommand(double elevatorMeters, double wristRadians, double shooterRPS, Profile limelightProfile,
      DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem, ConeLimelightSubsystem limelightSubsystem) {
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;
    this.limelightProfile = limelightProfile;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, limelightSubsystem);
  }

  protected ShootCommand(
      Profile limeProfile, DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem, ConeLimelightSubsystem limelightSubsystem) {
    this.limelightProfile = limeProfile;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, limelightSubsystem);
    }

  @Override
  public void initialize() {
    shootTimer.reset();
    isShooting = false;
    aimController.reset(drivetrainSubsystem.getGyroscopeRotation().getRadians());
    distanceController.setSetpoint(TARGET_Y_SETPOINT);
    limelightSubsystem.enable();
    limelightSubsystem.setActiveProfile(limelightProfile);
  }

  @Override
  public void execute() {
    var limelightResults = limelightSubsystem.getLatestResults();
    if (limelightResults.valid) {

      var limelightRetroResults = limelightResults.targets_Retro[0];
      var targetX =  limelightRetroResults.tx;
      var targetY = limelightRetroResults.ty;

      elevatorSubsystem.moveToPosition(elevatorMeters);
      wristSubsystem.moveToPosition(wristRadians);

      var drivetrainHeading = drivetrainSubsystem.getGyroscopeRotation();
      var targetHeadingRadians = drivetrainHeading.getRadians() - Units.degreesToRadians(targetX);
      
      aimController.setGoal(targetHeadingRadians);
      var rotationCorrection = 
          Math.abs(targetX) > AIM_TOLERANCE ? aimController.calculate(drivetrainHeading.getRadians()) : 0;
      var distanceCorrection =
          Math.abs(targetY - TARGET_Y_SETPOINT) > DISTANCE_TOLERANCE ? distanceController.calculate(targetY) : 0;

      drivetrainSubsystem.drive(new ChassisSpeeds(distanceCorrection, 0, rotationCorrection));

      var readyToShoot =
          Math.abs(elevatorSubsystem.getElevatorPosition() - elevatorMeters) < ELEVATOR_TOLERANCE
          && Math.abs(wristSubsystem.getWristPosition() - wristRadians) < WRIST_TOLERANCE
          && Math.abs(targetX) < AIM_TOLERANCE
          && Math.abs(targetY - TARGET_Y_SETPOINT) < DISTANCE_TOLERANCE;

      if (isShooting || readyToShoot) {
        shooterSubsystem.shootVelocity(shooterRPS);
        shootTimer.start();
        isShooting = true;
      }
    } else {
      // No target
      elevatorSubsystem.stop();
      wristSubsystem.stop();
      shooterSubsystem.stop();
      drivetrainSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return isShooting && shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    wristSubsystem.stop();
    shooterSubsystem.stop();
    drivetrainSubsystem.stop();
  }

}
