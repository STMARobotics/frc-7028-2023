// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
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

  private final PhotonCamera photonCamera = new PhotonCamera("LimeLight");

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrainSubsystem);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
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

    // POV up for field oriented drive
    controller.povUp().onTrue(runOnce(() -> drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand))
        .andThen(new ScheduleCommand(fieldOrientedDriveCommand)));
    // POV down for field heading drive
    controller.povDown().onTrue(runOnce(() -> drivetrainSubsystem.setDefaultCommand(fieldHeadingDriveCommand))
        .andThen(new ScheduleCommand(fieldHeadingDriveCommand)));

    // POV Right to put the wheels in an X until the driver tries to drive
    BooleanSupplier driverDriving = 
        () -> modifyAxis(controller.getLeftY()) != 0.0 
            || modifyAxis(controller.getLeftX()) != 0.0
            || modifyAxis(controller.getRightY()) != 0.0
            || modifyAxis(controller.getRightX()) != 0.0;
    controller.povRight().onTrue(run(drivetrainSubsystem::setWheelsToX, drivetrainSubsystem).until(driverDriving));

    // Elevator
    controller.y().whileTrue(
        startEnd(() -> elevatorSubsystem.moveElevator(0.8), elevatorSubsystem::stop, elevatorSubsystem));
    controller.b().whileTrue(
        startEnd(() -> elevatorSubsystem.moveElevator(-0.95), elevatorSubsystem::stop, elevatorSubsystem));

    // Wrist
    controller.x().whileTrue(
        startEnd(() -> wristSubsystem.moveWrist(.3), wristSubsystem::stop, wristSubsystem));
    controller.a().whileTrue(
        startEnd(() -> wristSubsystem.moveWrist(-.3), wristSubsystem::stop, wristSubsystem));

    // Shooter
    controller.rightBumper().whileTrue(startEnd(
      ()-> shooterSubsystem.shootDutyCycle(0.4825), shooterSubsystem::stop, shooterSubsystem));
    
    controller.leftBumper().whileTrue(startEnd(
      ()-> shooterSubsystem.shootDutyCycle(-0.15), shooterSubsystem::stop, shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                1,
                1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these no interior waypoints
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.fromDegrees(90)),
            config);
    
    return print("Starting auto")
        .andThen(runOnce(
            () -> poseEstimator.setCurrentPose(new Pose2d(0, 0, new Rotation2d(0))), drivetrainSubsystem))
        .andThen(drivetrainSubsystem.createCommandForTrajectory(exampleTrajectory, poseEstimator::getCurrentPose))
        .andThen(runOnce(drivetrainSubsystem::stop, drivetrainSubsystem))
        .andThen(print("Done with auto"));
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
