package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.InstantWhenDisabledCommand;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {
  private static final double TARGET_HEIGHT = Units.inchesToMeters(16);
  
  //valid keys - https://docs.limelightvision.io/en/latest/networktables_api.html
  private final static String ntTargetValid = "tv";
  private final static String ntTargetX = "tx";
  private final static String ntTargetY = "ty";
  private final static String ntSkew = "ts";

  private final NetworkTable limelightNetworkTable;
  private final String networkTableName;
  private final LimelightConfig limelightConfig;

  private boolean takeSnapshot = false;

  private boolean targetValid = false;
  private long targetLastSeen = 0;
  private double targetX = 0;
  private double targetY = 0;
  private double skew = 0;

  private final HashMap<String, MedianFilter> updateFilterMap = new HashMap<>();
  
  private boolean enabled;
  private boolean driverMode;

  public LimelightSubsystem(LimelightConfig limelightConfig) {
    
    this.limelightConfig = limelightConfig;

    networkTableName = limelightConfig.getNetworkTableName();
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);

    //this adds listeners on an explicit list
    addLimelightUpdateListeners(limelightNetworkTable, ntTargetValid, ntTargetX, ntTargetY, ntSkew);

    limelightNetworkTable.getEntry("snapshot").setDouble(0.0);

    new Trigger(RobotState::isEnabled)
        .whenActive(new InstantCommand(this::enable))
        .whenInactive(new InstantWhenDisabledCommand(this::disable, this));
  }

  private void addLimelightUpdateListeners(NetworkTable limelightTable, String... keys) {
    for (String key : keys) {
      limelightNetworkTable.addEntryListener(key, this::update, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
      updateFilterMap.putIfAbsent(key, new MedianFilter(20));
    }
  }

  private void update(final NetworkTable table, final String key, final NetworkTableEntry entry,
      final NetworkTableValue value, final int flags) {

    switch(key) {

      case ntTargetX:
        targetX = value.getDouble();
        break;

      case ntTargetY:
        targetY = value.getDouble();
        break;

      case ntTargetValid:
        targetValid = value.getDouble() == 1.0;
        break;

      case ntSkew:
        skew = value.getDouble();
        break;

    }
  }

  @Override
  public void periodic() {
    // Flush NetworkTable to send LED mode and pipeline updates immediately
    var shouldFlush = limelightNetworkTable.getEntry("ledMode").getDouble(0.0) != (enabled ? 0.0 : 1.0);
  
    limelightNetworkTable.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
    limelightNetworkTable.getEntry("camMode").setDouble(driverMode ? 1.0 : 0.0);
  
    if (shouldFlush)  {
      NetworkTableInstance.getDefault().flush();
    }

    if(takeSnapshot) {
      limelightNetworkTable.getEntry("snapshot").setDouble(1.0);
      takeSnapshot = false;
    } else {
      limelightNetworkTable.getEntry("snapshot").setDouble(0.0);
    }
  }

  public ShuffleboardLayout addDashboardWidgets(ShuffleboardTab dashboard) {    
    var detailDashboard = dashboard.getLayout("Target", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 2)).withPosition(0, 0);
    
    detailDashboard.addBoolean("Acquired", this::getTargetAcquired).withPosition(0, 0);
    detailDashboard.addNumber("X", this::getTargetX).withPosition(0, 1);
    detailDashboard.addNumber("Y", this::getTargetY).withPosition(1, 1);
    detailDashboard.addNumber("Distance", () -> Units.metersToInches(getDistanceToTarget())).withPosition(0, 2);
    detailDashboard.addNumber("Skew", this::getSkew).withPosition(1, 2);
    
    dashboard.add(this).withPosition(0, 1);
    return detailDashboard;
  }

  /**
   * Gets the distance from the Limelight to the target. If no target is acquired this will return zero. This value is
   * the distance straight ahead from the limelight to the target, (parallel to the floor.) Use
   * {@link #getDistanceToTarget()} to get the distance from the robot to the target.
   * 
   * @return distance from the Limelight to the target
   */
  private double getLimelightDistanceToTarget() {
    if (getTargetAcquired()) {
      return (TARGET_HEIGHT - limelightConfig.getMountHeight())
          / Math.tan(Units.degreesToRadians(limelightConfig.getMountAngle() + getTargetY()));
    }
    return 0.0;
  }

  /**
  * Gets the distance from the center of the robot front bumper to the base of the target. If no target is acquired this
  * will return zero. This method accounts for the limelight being mounted off-center and behind the front bumper.
  * 
  * @return distance from the center of the robot front bumper to the base of the target
  */
  public double getDistanceToTarget() {
    return Math.sqrt(Math.pow(getLimelightDistanceToTarget(), 2)
        + Math.pow(limelightConfig.getMountDistanceFromCenter(), 2)) - limelightConfig.getMountDepth();
  }

  public boolean getTargetAcquired() {
    return targetValid;
  }

  public long getTargetLastSeen() {
    return targetLastSeen;
  }

  public double getTargetX() {
    return targetX;
  }

  public double getTargetY() {
    return targetY;
  }

  public double getSkew() {
    return skew;
  }

  /**
   * Turns the LEDS off and switches the camera mode to vision processor.
   */
  public void disable() {
    enabled = false;
    driverMode = false;
  }

  /**
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode
   * to vision processor.
   */
  public void enable() {
    enabled = true;
    driverMode = false;
  }

  /**
   * Sets the LEDs to off and switches the camera to driver mode.
   */
  public void driverMode() {
    enabled = false;
    driverMode = true;
  }

  public String getNetworkTableName() {
    return networkTableName;
  }

  public void takeSnapshot() {
    takeSnapshot = true;
  }

}