package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Command to pick up a game piece from the double station - and hold elevator and wrist until interrupted */
public class DoubleStationCommand extends SequentialCommandGroup {

  public DoubleStationCommand(double elevatorMeters, double wristRadians, double intakeDutyCycle, double forwardSpeed, 
  ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, DrivetrainSubsystem drivetrainSubsystem, 
  ShooterSubsystem shooterSubsystem, Supplier<Pose2d> poseSupplier, LimelightSubsystem limelightSubsystem,
  LimelightProfile profile, BooleanSupplier finishedSuppiler) {

    var pickup = new AutoPickupCommand(
      elevatorMeters, 0.0, -0.1, 0.2, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
      poseSupplier, limelightSubsystem, profile, finishedSuppiler);

    addCommands(pickup,
        run(() -> elevatorSubsystem.moveToPosition(elevatorMeters), elevatorSubsystem)
            .alongWith(run(() -> wristSubsystem.moveToPosition(.3), wristSubsystem)));

  }
  
}
