package frc.robot.commands.autonomous;

import static frc.robot.Constants.AutoConstants.THETA_kD;
import static frc.robot.Constants.AutoConstants.THETA_kI;
import static frc.robot.Constants.AutoConstants.THETA_kP;
import static frc.robot.Constants.AutoConstants.X_kD;
import static frc.robot.Constants.AutoConstants.X_kI;
import static frc.robot.Constants.AutoConstants.X_kP;
import static frc.robot.Constants.AutoConstants.Y_kD;
import static frc.robot.Constants.AutoConstants.Y_kI;
import static frc.robot.Constants.AutoConstants.Y_kP;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {
  
  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_VELOCITY_METERS_PER_SECOND * 0.5,
      MAX_VELOCITY_METERS_PER_SECOND);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Pose2d goalPose;
  private final LEDSubsystem ledSubsystem;
  private final boolean useAllianceColor;

  public DriveToPoseCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        LEDSubsystem ledSubsystem,
        boolean useAllianceColor) {
    this(drivetrainSubsystem, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, ledSubsystem, useAllianceColor);
  }

  public DriveToPoseCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        LEDSubsystem ledSubsystem,
        boolean useAllianceColor) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    this.ledSubsystem = ledSubsystem;
    this.useAllianceColor = useAllianceColor;

    xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);

    addRequirements(drivetrainSubsystem);
  }


  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = goalPose;
    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());
    ledSubsystem.setCustomMode(
        leds -> leds.setLEDSegments(Color.kBlue, true, xController.atGoal(), yController.atGoal(), thetaController.atGoal()));
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
