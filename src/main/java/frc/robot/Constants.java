package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose.
 */
public final class Constants {
  private Constants() {
  }

  /**
   * Contains physical measurements and CAN IDs for the robot hardware.
   */
  public static final class RobotConstants {
    private RobotConstants() {
    }
    
    public static final double MAX_SPEED = 15.0;
  }

  public static final class SubsystemConfig {
    public static final CameraConfig[] cameraConfigs = {
      new CameraConfig("test", new Transform3d(0, 0, 0.1524, new Rotation3d())),
    };
  }

  /**
   * Constants for operator interface (OI) configuration.
   */
  public static final class OIConstants {
    private OIConstants() {
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DRIVER_DEADBAND = 0.05;
  }
}
