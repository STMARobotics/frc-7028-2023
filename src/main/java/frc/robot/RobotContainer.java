// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts.kGrid;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;
import static frc.robot.Constants.PickupConstants.CUBE_ELEVATOR_HEIGHT;
import static frc.robot.Constants.PickupConstants.CUBE_FORWARD_SPEED;
import static frc.robot.Constants.PickupConstants.CUBE_INTAKE_DUTY_CYCLE;
import static frc.robot.Constants.PickupConstants.CUBE_WRIST_ANGLE;
import static frc.robot.Constants.PickupConstants.DOUBLE_ELEVATOR_HEIGHT;
import static frc.robot.Constants.PickupConstants.DOUBLE_INTAKE_DUTY_CYCLE;
import static frc.robot.Constants.PickupConstants.DOUBLE_WRIST_ANGLE;
import static frc.robot.Constants.VisionConstants.HIGH_LIMELIGHT_TO_ROBOT;
import static frc.robot.GamePiece.CONE;
import static frc.robot.controls.OperatorButtons.ENTER_SELECTION;
import static frc.robot.controls.OperatorButtons.GRID_CENTER;
import static frc.robot.controls.OperatorButtons.GRID_LEFT;
import static frc.robot.controls.OperatorButtons.GRID_RIGHT;
import static frc.robot.limelight.LimelightProfile.PICKUP_CONE_FLOOR;
import static frc.robot.subsystems.LEDSubsystem.CUBE_COLOR;

import java.util.Arrays;
import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultHighLimelightCommand;
import frc.robot.commands.DefaultLEDCommand;
import frc.robot.commands.DefaultWristCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.HighPickupCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.LEDBootAnimationCommand;
import frc.robot.commands.LEDMarqueeCommand;
import frc.robot.commands.TeleopConePickupCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.OperatorNodeButtons;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoreLocationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final ControlBindings controlBindings;
  private final CommandJoystick operatorJoysticks[] =
      new CommandJoystick[] {new CommandJoystick(2), new CommandJoystick(3)};

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator =
      new PoseEstimatorSubsystem(drivetrainSubsystem::getGyroscopeRotation, drivetrainSubsystem::getModulePositions);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem lowLimelightSubsystem = new LimelightSubsystem(VisionConstants.LOW_LIMELIGHT_NAME);
  private final LimelightSubsystem highLimelightSubsystem = new LimelightSubsystem(VisionConstants.HIGH_LIMELIGHT_NAME);
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final ScoreLocationSubsystem scoreLocation =
      new ScoreLocationSubsystem(shooterSubsystem::hasCone, shooterSubsystem::hasCube);
  
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand;
  private final FieldHeadingDriveCommand fieldHeadingDriveCommand;

  private final DefaultWristCommand defaultWristCommand = new DefaultWristCommand(wristSubsystem);

  private final Timer reseedTimer = new Timer();

  private final AutonomousBuilder autoBuilder = new AutonomousBuilder(drivetrainSubsystem, elevatorSubsystem,
      ledSubsystem, shooterSubsystem, wristSubsystem, lowLimelightSubsystem, highLimelightSubsystem, poseEstimator);

  private final AutoScoreCommand autoScoreCommand =
      new AutoScoreCommand(scoreLocation, shooterSubsystem::hasCone, autoBuilder, drivetrainSubsystem, elevatorSubsystem,
          ledSubsystem, shooterSubsystem, wristSubsystem, lowLimelightSubsystem, highLimelightSubsystem);

  private boolean pickingUp = false;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0)) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

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
    ledSubsystem.setDefaultCommand(
        new DefaultLEDCommand(ledSubsystem, shooterSubsystem::hasCone, shooterSubsystem::hasCube));
    elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(
        elevatorSubsystem, wristSubsystem::isParked));
    highLimelightSubsystem.setDefaultCommand(new DefaultHighLimelightCommand(
        shooterSubsystem::hasCone, shooterSubsystem::hasCube, highLimelightSubsystem, scoreLocation::getSelectedGamePiece));


    configureButtonBindings();
    configureDashboard();
    reseedTimer.start();
    new LEDBootAnimationCommand(ledSubsystem).schedule();
  }

  private void configureDashboard() {
    /**** Driver tab ****/
    var driverTab = Shuffleboard.getTab("Driver");
    autoBuilder.addDashboardWidgets(driverTab);
    driverTab.addBoolean("Pickup", () -> pickingUp).withPosition(11, 3).withSize(2, 2);

    driverTab.add(new HttpCamera("limelight-high", "http://10.70.28.13:1181"))
        .withWidget(BuiltInWidgets.kCameraStream)
        .withProperties(Map.of("showCrosshair", true, "showControls", false, "rotation", "QUARTER_CCW"))
        .withSize(4, 6).withPosition(0, 0);

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
        .ifPresent(trigger -> trigger.onTrue(drivetrainSubsystem.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

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
      ()-> shooterSubsystem.shootDutyCycle(-0.15), shooterSubsystem::activeStop, shooterSubsystem)));

    // Intake
    controlBindings.autoIntake().ifPresent(trigger -> trigger.whileTrue(runOnce(() -> pickingUp = true)
        .alongWith(either(
            autoBuilder.pickupCone(),
            autoBuilder.pickupCube(),
            () -> scoreLocation.getSelectedGamePiece() == CONE)))
      .onFalse(runOnce(() -> pickingUp = false)));

    // Double sub-station pickup
    controlBindings.doubleStationPickup().ifPresent(trigger -> trigger.toggleOnTrue(runOnce(() -> pickingUp = true)
        .alongWith(new HighPickupCommand(controlBindings.translationX(), controlBindings.translationY(),            
        controlBindings.omega(), DOUBLE_WRIST_ANGLE, DOUBLE_ELEVATOR_HEIGHT, DOUBLE_INTAKE_DUTY_CYCLE,
        elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem))));

    // Manual Cube
    controlBindings.manualCube().ifPresent(trigger -> trigger.whileTrue(
      new TeleopConePickupCommand(CUBE_ELEVATOR_HEIGHT, CUBE_WRIST_ANGLE, CUBE_INTAKE_DUTY_CYCLE,
      CUBE_FORWARD_SPEED, elevatorSubsystem, wristSubsystem, drivetrainSubsystem,
      shooterSubsystem, () -> controlBindings.translationX().getAsDouble() * 0.25,
      () -> controlBindings.translationY().getAsDouble() * 0.25,
      () -> controlBindings.omega().getAsDouble() / 6.0,
      shooterSubsystem::hasCube)));

    // Manual Cone
    controlBindings.manualCone().ifPresent(trigger -> trigger.whileTrue(
        new TeleopConePickupCommand(Constants.PickupConstants.CONE_ELEVATOR_HEIGHT, 
        Constants.PickupConstants.CONE_WRIST_ANGLE,
        Constants.PickupConstants.CONE_INTAKE_DUTY_CYCLE,
        Constants.PickupConstants.CONE_FORWARD_SPEED, elevatorSubsystem, wristSubsystem, drivetrainSubsystem,
        shooterSubsystem, () -> controlBindings.translationX().getAsDouble() * 0.25,
        () -> controlBindings.translationY().getAsDouble() * 0.25,
        () -> controlBindings.omega().getAsDouble() / 6.0,
        shooterSubsystem::hasCone)));

    // Drive to human player station
    controlBindings.driveSingleSubstation().ifPresent(trigger -> trigger.whileTrue(
        autoBuilder.driveToPose(new Pose2d(14.22, 7.0, Rotation2d.fromDegrees(90)), true)));

    // Shoot Top
    controlBindings.shootHigh().ifPresent(trigger -> trigger.whileTrue(
        either(autoBuilder.shootConeTop(), autoBuilder.shootCubeTop(), shooterSubsystem::hasCone)));

    // Shoot Mid
    controlBindings.shootMid().ifPresent(trigger -> trigger.whileTrue(
        either(autoBuilder.shootConeMid(), autoBuilder.shootCubeMid(), shooterSubsystem::hasCone)));
    
    // Shoot Low
    controlBindings.shootLow().ifPresent(trigger -> trigger.whileTrue(
        either(autoBuilder.shootConeBottom(), autoBuilder.shootCubeBottom(), shooterSubsystem::hasCone)));
    
    // Auto score
    controlBindings.shootAutomatically().ifPresent(trigger -> trigger.whileTrue(autoScoreCommand));

    // Launch cube
    controlBindings.launchCube().ifPresent(trigger -> trigger.onTrue(new JustShootCommand(
        0.0, 0.78, 130.0, CUBE_COLOR, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem)));
    
    //////////// Operator \\\\\\\\\\\\
    // Enter location selection
    operatorJoysticks[ENTER_SELECTION.joystickId].button(ENTER_SELECTION.buttonId)
        .onTrue(Commands.print("Enter not cable side")).onFalse(Commands.print("Enter cable side"));
    
    // Grid selection
    operatorJoysticks[GRID_LEFT.joystickId].button(GRID_LEFT.buttonId).onTrue(runOnce(() -> scoreLocation.selectGrid(0)));
    operatorJoysticks[GRID_CENTER.joystickId].button(GRID_CENTER.buttonId).onTrue(runOnce(() -> scoreLocation.selectGrid(1)));
    operatorJoysticks[GRID_RIGHT.joystickId].button(GRID_RIGHT.buttonId).onTrue(runOnce(() -> scoreLocation.selectGrid(2)));

    // Node selection
    Arrays.stream(OperatorNodeButtons.values()).forEach(button -> 
      operatorJoysticks[button.joystickId].button(button.buttonId).onTrue(
          runOnce(() -> scoreLocation.selectColumn(button.column).selectRow(button.row))
              .andThen(
                  Commands.either(new LEDMarqueeCommand(ledSubsystem, 20, 255, 0, 15, .07),
                      new LEDMarqueeCommand(ledSubsystem, 130, 255, 0, 15, .07),
                      () -> scoreLocation.getSelectedGamePiece() == CONE)))
    );

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
    autoScoreCommand.setAlliance(alliance);
  }

}
