package frc.robot.swerve;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class Mk4SwerveModuleFactory {
    private Mk4SwerveModuleFactory() {
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param moduleConfiguration        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static Falcon500SwerveModule createFalcon500(
            ShuffleboardLayout container,
            ModuleConfiguration moduleConfiguration,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset) {
      
      var driveController = new Falcon500DriveController(driveMotorPort, moduleConfiguration, container);
      var steerController = 
          new Falcon500SteerController(steerMotorPort, steerEncoderPort, steerOffset, container, moduleConfiguration);

      return new Falcon500SwerveModule(driveController, steerController);
    }
}
