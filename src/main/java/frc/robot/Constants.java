package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

  /**
   * Constants for field positions and points of interest
   */
  public static final class FieldConstants {
    private FieldConstants() {
    }

    /** Field dimensions for 2025 Reefscape */
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.21;

    /**
     * Point of Interest on the field
     * Contains a pose, alliance designation, and a descriptive tag
     */
    public static final class POI {
      private final Pose2d pose;
      private final String tag;

      public POI(Pose2d pose, String tag) {
        this.pose = pose;
        this.tag = tag;
      }

      public Pose2d get(Alliance alliance) {
        if (alliance == Alliance.Blue) {
          return pose;
        } else {
          return new Pose2d(
              FIELD_LENGTH_METERS - pose.getTranslation().getX(),
              FIELD_WIDTH_METERS - pose.getTranslation().getY(),
              pose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)));
        }
      }

      /**
       * Gets the descriptive tag for this POI
       * 
       * @return The tag string
       */
      public String getTag() {
        return tag;
      }
    }

    /** All consolidated points of interest on the field */
    public static final POI[] ALL_POIS = {
        // Intake stations
        new POI(new Pose2d(1.25, 7.0, Rotation2d.fromDegrees(126.0)), "INTAKE_STATION"),
        new POI(new Pose2d(1.25, FIELD_WIDTH_METERS - 7.0, Rotation2d.fromDegrees(-126.0)), "INTAKE_STATION"),

        // Coral reef bars
        new POI(new Pose2d(5.9, 4.2, Rotation2d.fromDegrees(0)), "CORAL_REEF"),
        new POI(new Pose2d(5.3, 5.15, Rotation2d.fromDegrees(60.0)), "CORAL_REEF"),
        new POI(new Pose2d(5.0, 5.3, Rotation2d.fromDegrees(60.0)), "CORAL_REEF"),
        new POI(new Pose2d(4.0, 5.3, Rotation2d.fromDegrees(120.0)), "CORAL_REEF"),
        new POI(new Pose2d(3.7, 5.15, Rotation2d.fromDegrees(120.0)), "CORAL_REEF"),
        new POI(new Pose2d(3.1, 4.2, Rotation2d.fromDegrees(180.0)), "CORAL_REEF"),
        new POI(new Pose2d(5.9, FIELD_WIDTH_METERS - 4.2, Rotation2d.fromDegrees(0)), "CORAL_REEF"),
        new POI(new Pose2d(5.3, FIELD_WIDTH_METERS - 5.15, Rotation2d.fromDegrees(300.0)), "CORAL_REEF"),
        new POI(new Pose2d(5.0, FIELD_WIDTH_METERS - 5.3, Rotation2d.fromDegrees(300.0)), "CORAL_REEF"),
        new POI(new Pose2d(4.0, FIELD_WIDTH_METERS - 5.3, Rotation2d.fromDegrees(230.0)), "CORAL_REEF"),
        new POI(new Pose2d(3.7, FIELD_WIDTH_METERS - 5.15, Rotation2d.fromDegrees(240.0)), "CORAL_REEF"),
        new POI(new Pose2d(3.1, FIELD_WIDTH_METERS - 4.2, Rotation2d.fromDegrees(180)), "CORAL_REEF"),

        // Alga stations
        new POI(new Pose2d(6.0, 0.75, Rotation2d.fromDegrees(180.0)), "ALGA_STATION"),

        // Cages
        new POI(new Pose2d(FIELD_LENGTH_METERS / 2, 8.765, Rotation2d.fromDegrees(0.0)), "CAGE"),
        new POI(new Pose2d(FIELD_LENGTH_METERS / 2, 6.165, Rotation2d.fromDegrees(0.0)), "CAGE"),
        new POI(new Pose2d(FIELD_LENGTH_METERS / 2, 3.565, Rotation2d.fromDegrees(0.0)), "CAGE")
    };

    /**
     * Legacy POI lists for backward compatibility - reference the consolidated list
     * by tag
     */
    public static final POI[] INTAKE_STATIONS = filterPoisByTag(ALL_POIS, "INTAKE_STATION");
    public static final POI[] CORAL_REEF_BARS = filterPoisByTag(ALL_POIS, "CORAL_REEF");
    public static final POI[] ALGA_STATIONS = filterPoisByTag(ALL_POIS, "ALGA_STATION");
    public static final POI[] CAGES = filterPoisByTag(ALL_POIS, "CAGE");

    /**
     * Filters the master POI list by tag
     * 
     * @param pois The array of POIs to filter
     * @param tag  The tag to match
     * @return Array of POIs with the matching tag
     */
    private static POI[] filterPoisByTag(POI[] pois, String tag) {
      return java.util.Arrays.stream(pois)
          .filter(poi -> poi.getTag().equals(tag))
          .toArray(POI[]::new);
    }
  }
}
