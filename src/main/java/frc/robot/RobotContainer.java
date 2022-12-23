// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);
  private final PhotonCamera photonCamera = new PhotonCamera("gloworm");

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrainSubsystem);
  
  private final ChaseTagCommand chaseTagCommand = 
      new ChaseTagCommand(photonCamera, drivetrainSubsystem, poseEstimator::getCurrentPose);
  
  private final FieldHeadingDriveCommand fieldHeadingDriveCommand = new FieldHeadingDriveCommand(
      drivetrainSubsystem,
      () -> poseEstimator.getCurrentPose().getRotation(),
      () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, 
      () -> -controller.getRightY(),
      () -> -controller.getRightX());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> poseEstimator.getCurrentPose().getRotation(),
        () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2
    ));

    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();
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
    controller.back().onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));
    // Start button reseeds the steer motors to fix dead wheel
    controller.start().onTrue(Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));
    controller.b().whileTrue(chaseTagCommand);
    controller.start().toggleOnTrue(fieldHeadingDriveCommand);
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
    
    return new PrintCommand("Starting auto")
        .andThen(new InstantCommand(
            () -> poseEstimator.setCurrentPose(new Pose2d(0, 0, new Rotation2d(0))), drivetrainSubsystem))
        .andThen(drivetrainSubsystem.createCommandForTrajectory(exampleTrajectory, poseEstimator::getCurrentPose))
        .andThen(new RunCommand(drivetrainSubsystem::stop, drivetrainSubsystem))
        .andThen(new PrintCommand("Done with auto"));
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
