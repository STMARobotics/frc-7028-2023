// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final PhotonCamera photonCamera = new PhotonCamera("LimeLight");

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrainSubsystem);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
  private final ChaseTagCommand chaseTagCommand = 
      new ChaseTagCommand(photonCamera, drivetrainSubsystem, poseEstimator::getCurrentPose);
  
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
      drivetrainSubsystem,
      () -> poseEstimator.getCurrentPose().getRotation(),
      () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2
  );

  private final FieldHeadingDriveCommand fieldHeadingDriveCommand = new FieldHeadingDriveCommand(
      drivetrainSubsystem,
      () -> poseEstimator.getCurrentPose().getRotation(),
      () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, 
      () -> -controller.getRightY(),
      () -> -controller.getRightX());

  private final Timer reseedTimer = new Timer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(
      elevatorSubsystem, operatorController::getRightTriggerAxis, operatorController::getLeftTriggerAxis));
    wristSubsystem.setDefaultCommand(new WristCommand(wristSubsystem, () -> -operatorController.getRightY()));

    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();
    reseedTimer.start();
  }

  private void configureDashboard() {

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button resets the robot pose
    controller.back().onTrue(runOnce(poseEstimator::resetFieldPosition));
    // Start button reseeds the steer motors to fix dead wheel
    controller.start().onTrue(drivetrainSubsystem.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets));
    // B to chase a tag
    controller.b().whileTrue(chaseTagCommand);

    // POV up for field oriented drive
    controller.povUp().onTrue(runOnce(() -> drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand))
        .andThen(new ScheduleCommand(fieldOrientedDriveCommand)));
    // POV down for field heading drive
    controller.povDown().onTrue(runOnce(() -> drivetrainSubsystem.setDefaultCommand(fieldHeadingDriveCommand))
        .andThen(new ScheduleCommand(fieldHeadingDriveCommand)));

    // X to put the wheels in an X until the driver tries to drive
    BooleanSupplier driverDriving = 
        () -> modifyAxis(controller.getLeftY()) != 0.0 
            || modifyAxis(controller.getLeftX()) != 0.0
            || modifyAxis(controller.getRightY()) != 0.0
            || modifyAxis(controller.getRightX()) != 0.0;
    controller.x().onTrue(run(drivetrainSubsystem::setWheelsToX, drivetrainSubsystem).until(driverDriving));

    // Operator
    operatorController.rightBumper().whileTrue(Commands.startEnd(
      ()-> shooterSubsystem.shootDutyCycle(0.9), shooterSubsystem::stop, shooterSubsystem));
    
    operatorController.leftBumper().whileTrue(Commands.startEnd(
      ()-> shooterSubsystem.shootDutyCycle(-0.15), shooterSubsystem::stop, shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var trajectory = PathPlanner.generatePath(new PathConstraints(1, 1), 
      new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(3, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
    );

    return print("Starting auto")
        .andThen(runOnce(
            () -> poseEstimator.setCurrentPose(new Pose2d(0, 0, new Rotation2d(0))), drivetrainSubsystem))
        .andThen(drivetrainSubsystem
            .createCommandForTrajectory(trajectory, poseEstimator::getCurrentPose, true))
        .andThen(runOnce(drivetrainSubsystem::stop, drivetrainSubsystem))
        .andThen(print("Done with auto"));
  }

  public Command getPathPlannerCommand() {
    var testPath = PathPlanner.loadPath("TestPath", PathPlanner.getConstraintsFromPath("TestPath"));

    return print("Starting TestPath")
        .andThen(runOnce(() -> poseEstimator.setCurrentPose(testPath.getInitialHolonomicPose())))
        .andThen(drivetrainSubsystem
            .createCommandForTrajectory(testPath, poseEstimator::getCurrentPose, true))
        .andThen(runOnce(drivetrainSubsystem::stop, drivetrainSubsystem))
        .andThen(print("Done with TestPath"));
  }

  public Command getPathPlannerAutoCommand() {
    var testPath = PathPlanner.loadPath("TestPath", PathPlanner.getConstraintsFromPath("TestPath"));

    var eventMap = new HashMap<String, Command>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        poseEstimator::getCurrentPose,
        poseEstimator::setCurrentPose,
        DrivetrainConstants.KINEMATICS,
        new PIDConstants(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD),
        new PIDConstants(AutoConstants.ROTATION_kP, AutoConstants.ROTATION_kI, AutoConstants.ROTATION_kD),
        drivetrainSubsystem::setModuleStates,
        eventMap,
        true,
        drivetrainSubsystem
    );

    return autoBuilder.fullAuto(testPath);
  }

  public void disabledPeriodic() {
    // Reseed the motor offset continuously when the robot is disabled to help solve dead wheel issue
    if (reseedTimer.advanceIfElapsed(1.0)) {
      drivetrainSubsystem.reseedSteerMotorOffsets();
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
