// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultLEDCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultWristCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.ShootConeCommand;
import frc.robot.commands.ShootCubeCommand;
import frc.robot.commands.TeleopConePickupCommand;
import frc.robot.commands.TuneShootCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.limelight.LimelightRetroCalcs;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final ControlBindings controlBindings;;

  private final PhotonCamera photonCamera = null;//new PhotonCamera("OV9281");

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrainSubsystem);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem coneLimelightSubsystem =
      new LimelightSubsystem(VisionConstants.SHOOTER_LIMELIGHT_CONFIG.getNetworkTableName());
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand;
  private final FieldHeadingDriveCommand fieldHeadingDriveCommand;

  private final DefaultWristCommand defaultWristCommand = new DefaultWristCommand(wristSubsystem);
  private final DefaultShooterCommand defaultShooterCommand = new DefaultShooterCommand(shooterSubsystem);

  private final Timer reseedTimer = new Timer();

  private final AutonomousBuilder autoBuilder = new AutonomousBuilder(drivetrainSubsystem, elevatorSubsystem,
      ledSubsystem, shooterSubsystem, wristSubsystem, coneLimelightSubsystem, poseEstimator);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure control binding scheme
    controlBindings = new JoystickControlBindings();
    // controlBindings = new XBoxControlBindings();

    fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        drivetrainSubsystem,
        () -> poseEstimator.getCurrentPose().getRotation(),
        controlBindings.translationX(),
        controlBindings.translationY(),
        controlBindings.omega());

    fieldHeadingDriveCommand = new FieldHeadingDriveCommand(
        drivetrainSubsystem,
        () -> poseEstimator.getCurrentPose().getRotation(),
        controlBindings.translationX(),
        controlBindings.translationY(),
        controlBindings.heading());

    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
    wristSubsystem.setDefaultCommand(defaultWristCommand);
    shooterSubsystem.setDefaultCommand(defaultShooterCommand);
    ledSubsystem.setDefaultCommand(new DefaultLEDCommand(ledSubsystem, shooterSubsystem::hasCone));
    elevatorSubsystem.setDefaultCommand(
        new DefaultElevatorCommand(elevatorSubsystem, () -> Math.abs(wristSubsystem.getWristPosition() - Math.PI/2) < .2));

    configureButtonBindings();
    configureDashboard();
    reseedTimer.start();
  }

  private void configureDashboard() {
    autoBuilder.addDashboardWidgets(Dashboard.driverTab);

    // Shooting tab
    final var limelightCalcs =
        new LimelightRetroCalcs(VisionConstants.SHOOTER_CAMERA_TO_ROBOT, Profile.TOP.targetHeight);
    final var shootingTab = Shuffleboard.getTab("Shooting");
    final var targetLayout = shootingTab.getLayout("Target", BuiltInLayouts.kList);
    targetLayout.addDouble("Top Target Distance", () -> {
        var optResults = coneLimelightSubsystem.getLatestRetroTarget();
        if (optResults.isPresent()) {
          return limelightCalcs.getRobotRelativeTargetInfo(optResults.get()).distance;
        }
        return 0;
    });

  }

  private void configureButtonBindings() {
    // reset the robot pose
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(poseEstimator::resetFieldPosition)));
    // Start button reseeds the steer motors to fix dead wheel
    controlBindings.reseedSteerMotors()
        .ifPresent(trigger -> trigger.onTrue(drivetrainSubsystem.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets)));

    // POV up for field oriented drive
    controlBindings.fieldOrientedDrive().ifPresent(
        trigger -> trigger.onTrue(runOnce(() -> drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand))
            .andThen(new ScheduleCommand(fieldOrientedDriveCommand))));

    // POV down for field heading drive
    controlBindings.fieldHeadingDrive().ifPresent(
        trigger -> trigger.onTrue(runOnce(() -> drivetrainSubsystem.setDefaultCommand(fieldHeadingDriveCommand))
            .andThen(new ScheduleCommand(fieldHeadingDriveCommand))));

    // POV Right to put the wheels in an X until the driver tries to drive
    controlBindings.wheelsToX()
        .ifPresent(trigger -> trigger.onTrue(run(drivetrainSubsystem::setWheelsToX, drivetrainSubsystem)
            .until(controlBindings.driverWantsControl())));

    // Elevator
    controlBindings.elevatorUp().ifPresent(trigger -> trigger.whileTrue(
        startEnd(() -> elevatorSubsystem.moveElevator(0.1), elevatorSubsystem::stop, elevatorSubsystem)));
    controlBindings.elevatorDown().ifPresent(trigger -> trigger.whileTrue(
        startEnd(() -> elevatorSubsystem.moveElevator(-0.05), elevatorSubsystem::stop, elevatorSubsystem)));

    // Wrist
    controlBindings.wristUp().ifPresent(trigger -> trigger.whileTrue(
        startEnd(() -> wristSubsystem.moveWrist(.2), wristSubsystem::stop, wristSubsystem)));
    controlBindings.wristDown().ifPresent(trigger -> trigger.whileTrue(
        startEnd(() -> wristSubsystem.moveWrist(-.1), wristSubsystem::stop, wristSubsystem)));

    // Shooter
    controlBindings.shooterOut().ifPresent(trigger -> trigger.whileTrue(startEnd(
      ()-> shooterSubsystem.shootDutyCycle(0.4825), shooterSubsystem::stop, shooterSubsystem)));
    controlBindings.shooterIn().ifPresent(trigger -> trigger.whileTrue(startEnd(
      ()-> shooterSubsystem.shootDutyCycle(-0.15), shooterSubsystem::stop, shooterSubsystem)));

    // Intake
    controlBindings.intakeCone().ifPresent(trigger -> trigger.whileTrue(new TeleopConePickupCommand(
        0.058, 0.0, -0.1, 0.2, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
        () -> controlBindings.translationX().getAsDouble() * 0.25,
        () -> controlBindings.translationY().getAsDouble() * 0.25,
        () -> controlBindings.omega().getAsDouble() / 6.0)
        .andThen(new DefaultWristCommand(wristSubsystem))));

    // Shoot
    controlBindings.tuneShoot().ifPresent(trigger -> trigger.whileTrue(
        new TuneShootCommand(elevatorSubsystem, wristSubsystem, shooterSubsystem, coneLimelightSubsystem, Profile.TOP)));

    controlBindings.shootConeHigh().ifPresent(trigger -> trigger.whileTrue(new ShootConeCommand(
        Profile.TOP, drivetrainSubsystem, elevatorSubsystem, wristSubsystem,
        shooterSubsystem, coneLimelightSubsystem, ledSubsystem)));

    // Drive to cone node to the left of tag 1, then just shoot
    // controller.rightTrigger().whileTrue(new DriveToPoseCommand(
    //     drivetrainSubsystem, poseEstimator::getCurrentPose, new Pose2d(14.59, 1.67, Rotation2d.fromDegrees(0.0)))
    //         .andThen(new JustShootCommand(0.4064, 1.05, 34.5, elevatorSubsystem, wristSubsystem, shooterSubsystem)));
    
    controlBindings.shootCubeHigh().ifPresent(trigger -> trigger.whileTrue(new ShootCubeCommand(
      inchesToMeters(17), 1, 60, elevatorSubsystem, wristSubsystem, shooterSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoBuilder.getAutonomousCommand();
  }

  public void disabledPeriodic() {
    // Reseed the motor offset continuously when the robot is disabled to help solve dead wheel issue
    if (reseedTimer.advanceIfElapsed(1.0)) {
      drivetrainSubsystem.reseedSteerMotorOffsets();
    }
  }

  /**
   * Called when the alliance reported by the driverstation/FMS changes.
   * @param alliance new alliance value
   */
  public void onAllianceChanged(Alliance alliance) {
    poseEstimator.setAlliance(alliance);
  }

}
