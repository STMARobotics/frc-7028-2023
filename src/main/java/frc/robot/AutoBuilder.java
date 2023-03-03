package frc.robot;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * PathPlanner auto builder that handles running our default commands while driving a path.
 */
public class AutoBuilder extends SwerveAutoBuilder {

  private final Command[] commandsDuringPath;
  
  public AutoBuilder( Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      SwerveDriveKinematics kinematics,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      Command[] commandsDuringPath,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, kinematics, translationConstants, rotationConstants, outputModuleStates, eventMap, useAllianceColor, driveRequirements);
    this.commandsDuringPath = commandsDuringPath;
  }

  @Override
  public CommandBase followPathWithEvents(PathPlannerTrajectory trajectory) {
    var wrappedCommands = Arrays.stream(commandsDuringPath).map(c -> wrappedEventCommand(c)).toArray(Command[]::new);
    return super.followPathWithEvents(trajectory).deadlineWith(wrappedCommands);
  }

}
