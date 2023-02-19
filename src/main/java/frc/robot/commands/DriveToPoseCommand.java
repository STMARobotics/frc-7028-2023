package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.THETA_kD;
import static frc.robot.Constants.AutoConstants.THETA_kI;
import static frc.robot.Constants.AutoConstants.THETA_kP;
import static frc.robot.Constants.AutoConstants.X_kD;
import static frc.robot.Constants.AutoConstants.X_kI;
import static frc.robot.Constants.AutoConstants.X_kP;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {
  
  private static final double TRANSLATION_TOLERANCE = 0.1;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(0.5);

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);

  private final ProfiledPIDController xController = new ProfiledPIDController(X_kP, X_kI, X_kD, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(X_kP, X_kI, X_kD, Y_CONSTRAINTS);
  private final ProfiledPIDController thetaController = 
      new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, OMEGA_CONSTRAINTS);

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final boolean endAtGoal;
  private Pose2d goalPose;

  public DriveToPoseCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose) {
    this(drivetrainSubsystem, poseProvider, goalPose, true);
  }

  public DriveToPoseCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Pose2d goalPose,
        boolean endAtGoal) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.endAtGoal = endAtGoal;
    this.goalPose = goalPose;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }


  @Override
  public void initialize() {
    resetPIDControllers();

    thetaController.setTolerance(THETA_TOLERANCE);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);

    setGoalPose(goalPose);
  }

  public void setGoalPose(Pose2d goalPose) {
    if (goalPose != null) {
      if (this.goalPose == null) {
        resetPIDControllers();
      } 
      thetaController.setGoal(goalPose.getRotation().getRadians());
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
    }
    this.goalPose = goalPose;
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
    if (goalPose == null) {
      // TODO use slew rate limiter to just stop the robot
    } else {
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
  }

  @Override
  public boolean isFinished() {
    return endAtGoal && atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
