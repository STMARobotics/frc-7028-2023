package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Command to back away and drop the elevator safely after pickup from double station */
public class AfterDoubleStationCommand extends SequentialCommandGroup {
  
  public AfterDoubleStationCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      DrivetrainSubsystem drivetrainSubsystem, double doublePickupHeight) {
    
    addCommands(
        run(() -> elevatorSubsystem.moveToPosition(doublePickupHeight), elevatorSubsystem)
            .alongWith(run(() -> wristSubsystem.moveToPosition(.3), wristSubsystem))
            .alongWith(run(() -> drivetrainSubsystem.drive(new ChassisSpeeds(-0.3, 0.0, 0.0)), drivetrainSubsystem)).withTimeout(2.0),
        run(() -> elevatorSubsystem.moveToPosition(.5), elevatorSubsystem).withTimeout(.5));
  }

}
