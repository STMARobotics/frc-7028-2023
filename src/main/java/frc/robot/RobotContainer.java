// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts.kGrid;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static frc.robot.Constants.VisionConstants.HIGH_LIMELIGHT_TO_ROBOT;
import static frc.robot.limelight.LimelightProfile.PICKUP_CONE_DOUBLE_STATION;
import static frc.robot.limelight.LimelightProfile.PICKUP_CONE_FLOOR;
import static frc.robot.limelight.LimelightProfile.PICKUP_CUBE_FLOOR;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AfterDoubleStationCommand;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultHighLimelightCommand;
import frc.robot.commands.DefaultLEDCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.DefaultWristCommand;
import frc.robot.commands.DoubleStationCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.LEDBootAnimationCommand;
import frc.robot.commands.ShootConeCommand;
import frc.robot.commands.ShootCubeCommand;
import frc.robot.commands.TeleopConePickupCommand;
import frc.robot.commands.TuneShootCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterProfile;
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

  private final PhotonCamera photonCamera = new PhotonCamera("OV9281");

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrainSubsystem);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem lowLimelightSubsystem = new LimelightSubsystem(VisionConstants.LOW_LIMELIGHT_NAME);
  private final LimelightSubsystem highLimelightSubsystem = new LimelightSubsystem(VisionConstants.HIGH_LIMELIGHT_NAME);
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand;
  private final FieldHeadingDriveCommand fieldHeadingDriveCommand;

  private final DefaultWristCommand defaultWristCommand = new DefaultWristCommand(wristSubsystem);
  private final DefaultShooterCommand defaultShooterCommand = new DefaultShooterCommand(shooterSubsystem);

  private final Timer reseedTimer = new Timer();

  private final AutonomousBuilder autoBuilder = new AutonomousBuilder(drivetrainSubsystem, elevatorSubsystem,
      ledSubsystem, shooterSubsystem, wristSubsystem, lowLimelightSubsystem, poseEstimator);

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
    ledSubsystem.setDefaultCommand(
        new DefaultLEDCommand(ledSubsystem, shooterSubsystem::hasCone, shooterSubsystem::hasCube));
    elevatorSubsystem.setDefaultCommand(
        new DefaultElevatorCommand(elevatorSubsystem, () -> Math.abs(wristSubsystem.getWristPosition() - 1.4) < .2));
    highLimelightSubsystem.setDefaultCommand(
        new DefaultHighLimelightCommand(shooterSubsystem::hasCone, shooterSubsystem::hasCube, highLimelightSubsystem));

    configureButtonBindings();
    configureDashboard();
    reseedTimer.start();
    new LEDBootAnimationCommand(ledSubsystem).schedule();
  }

  private void configureDashboard() {
    /**** Driver tab ****/
    var driverTab = Shuffleboard.getTab("Driver");
    autoBuilder.addDashboardWidgets(driverTab);

    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");

    // Pose estimation
    poseEstimator.addDashboardWidgets(visionTab);

    // Top target
    final var topLimelightCalcs = new LimelightCalcs(
        LimelightProfile.SCORE_CONE_TOP.cameraToRobot, LimelightProfile.SCORE_CONE_TOP.targetHeight);
    final var topTargetLayout = visionTab.getLayout("Top Target", kGrid).withPosition(6, 0).withSize(1, 2);
    lowLimelightSubsystem.addTargetDashboardWidgets(topTargetLayout, topLimelightCalcs);

    // Mid target
    final var midLimelightCalcs = new LimelightCalcs(
        LimelightProfile.SCORE_CONE_MIDDLE.cameraToRobot, LimelightProfile.SCORE_CONE_MIDDLE.targetHeight,
        elevatorSubsystem::getElevatorTopPosition);
    final var midTargetLayout = visionTab.getLayout("Mid Target", kGrid).withPosition(7, 0).withSize(1, 2);
    highLimelightSubsystem.addTargetDashboardWidgets(midTargetLayout, midLimelightCalcs);
    
    // Pickup game piece
    final var pickupLimelightCalcs = new LimelightCalcs(
        PICKUP_CONE_FLOOR.cameraToRobot, PICKUP_CONE_FLOOR.targetHeight,
        elevatorSubsystem::getElevatorTopPosition);
    final var pickupLayout = visionTab.getLayout("Pickup", kGrid).withPosition(8, 0).withSize(1, 3);
    highLimelightSubsystem.addDetectorDashboardWidgets(pickupLayout, pickupLimelightCalcs);
    pickupLayout.addDouble(
        "Camera Height", () -> -HIGH_LIMELIGHT_TO_ROBOT.getTranslation().getZ() + elevatorSubsystem.getElevatorTopPosition())
        .withPosition(0, 4);

    /**** Subsystems tab ****/
    final var subsystemsTab = Shuffleboard.getTab("Subsystems");
    elevatorSubsystem.addDashboardWidgets(subsystemsTab.getLayout("Elevator", kGrid).withPosition(0, 0).withSize(2, 3));
    shooterSubsystem.addDashboardWidgets(subsystemsTab.getLayout("Shooter", kGrid).withPosition(2, 0).withSize(2, 3));
    wristSubsystem.addDashboardWidgets(subsystemsTab.getLayout("Wrist", kGrid).withPosition(4, 0).withSize(2, 2));

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
    controlBindings.teleopIntakeCone().ifPresent(trigger -> trigger.whileTrue(new TeleopConePickupCommand(
        0.058, 0.0, -0.1, 0.2, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
        () -> controlBindings.translationX().getAsDouble() * 0.25,
        () -> controlBindings.translationY().getAsDouble() * 0.25,
        () -> controlBindings.omega().getAsDouble() / 6.0)
        .andThen(new DefaultWristCommand(wristSubsystem))));

    controlBindings.intakeCone().ifPresent(trigger -> trigger.whileTrue(new AutoPickupCommand(
        0.049, 0.0, -0.1, 0.2, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
        poseEstimator::getCurrentPose, highLimelightSubsystem, PICKUP_CONE_FLOOR,
        shooterSubsystem::hasCone, "Cone")));

    controlBindings.intakeCube().ifPresent(trigger -> trigger.whileTrue(new AutoPickupCommand(
        0.058, 0.0, -0.1, 0.2, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
        poseEstimator::getCurrentPose, highLimelightSubsystem, PICKUP_CUBE_FLOOR,
        shooterSubsystem::hasCube, "Cube")));
    
    // Double sub-station intake
    final var doublePickupHeight = 1.0;
    controlBindings.doubleStationCone().ifPresent(trigger -> trigger.whileTrue(new DoubleStationCommand(
      doublePickupHeight, 0.0, -0.1, 0.2, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
        poseEstimator::getCurrentPose, highLimelightSubsystem, PICKUP_CONE_DOUBLE_STATION,
        shooterSubsystem::hasCone))
            .onFalse(new AfterDoubleStationCommand(elevatorSubsystem, wristSubsystem, drivetrainSubsystem, doublePickupHeight)));
    
    controlBindings.doubleStationCube().ifPresent(trigger -> trigger.whileTrue(new DoubleStationCommand(
      doublePickupHeight, 0.0, -0.1, 0.2, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
        poseEstimator::getCurrentPose, highLimelightSubsystem, PICKUP_CONE_DOUBLE_STATION,
        shooterSubsystem::hasCube))
            .onFalse(new AfterDoubleStationCommand(elevatorSubsystem, wristSubsystem, drivetrainSubsystem, doublePickupHeight)));

    // Tune Shoot
    controlBindings.tuneShoot().ifPresent(trigger -> trigger.whileTrue(
        new TuneShootCommand(elevatorSubsystem, wristSubsystem, shooterSubsystem)));

    // Shoot Cone
    controlBindings.shootConeHigh().ifPresent(trigger -> trigger.whileTrue(new ShootConeCommand(
      ShooterProfile.SCORE_CONE_TOP, LimelightProfile.SCORE_CONE_TOP, drivetrainSubsystem, elevatorSubsystem,
          wristSubsystem, shooterSubsystem, lowLimelightSubsystem, ledSubsystem)));

    controlBindings.shootConeMid().ifPresent(trigger -> trigger.whileTrue(new ShootConeCommand(
        ShooterProfile.SCORE_CONE_MIDDLE, LimelightProfile.SCORE_CONE_MIDDLE, drivetrainSubsystem, elevatorSubsystem,
        wristSubsystem, shooterSubsystem, highLimelightSubsystem, ledSubsystem)));
    
    // controlBindings.shootConeLow().ifPresent(trigger -> trigger.whileTrue(new ShootConeCommand(
    //     ShooterProfile.SCORE_CONE_LOW, LimelightProfile.SCORE_CONE_LOW, drivetrainSubsystem, elevatorSubsystem,
    //     wristSubsystem, shooterSubsystem, highLimelightSubsystem, ledSubsystem)));
    controlBindings.shootConeLow().ifPresent(trigger -> trigger.whileTrue(new ShootCubeCommand(
        0.06, 0.1, 16.0, elevatorSubsystem, wristSubsystem, shooterSubsystem)));
    
    // Shoot cube
    controlBindings.shootCubeHigh().ifPresent(trigger -> trigger.whileTrue(new ShootCubeCommand(
        0.8, 0.5, 25.0, elevatorSubsystem, wristSubsystem, shooterSubsystem)));
    
    controlBindings.shootCubeMid().ifPresent(trigger -> trigger.whileTrue(new ShootCubeCommand(
        0.55, 0.4, 23.0, elevatorSubsystem, wristSubsystem, shooterSubsystem)));
    
    controlBindings.shootCubeLow().ifPresent(trigger -> trigger.whileTrue(new ShootCubeCommand(
        0.06, 0.1, 15.0, elevatorSubsystem, wristSubsystem, shooterSubsystem)));
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
