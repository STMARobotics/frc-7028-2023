package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command for the high limelight to proactively switch pipelines.
 */
public class DefaultHighLimelightCommand extends CommandBase {
  
  public final BooleanSupplier hasCone;
  public final BooleanSupplier hasCube;
  public final LimelightSubsystem limelightSubsystem;
  public DefaultHighLimelightCommand(BooleanSupplier hasCone, BooleanSupplier hasCube,
      LimelightSubsystem limelightSubsystem) {
    this.hasCone = hasCone;
    this.hasCube = hasCube;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(limelightSubsystem);
  }

  @Override
  public void execute() {
    if(hasCone.getAsBoolean()) {
      limelightSubsystem.setPipelineId(LimelightProfile.SCORE_CONE_MIDDLE.pipelineId);
    } else if (hasCube.getAsBoolean()) {
      limelightSubsystem.setPipelineId(LimelightProfile.PICKUP_CUBE_FLOOR.pipelineId);
    } else {
      limelightSubsystem.setPipelineId(LimelightProfile.PICKUP_CONE_FLOOR.pipelineId);
    }
  }

}
