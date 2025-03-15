package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Swerve subsystem manages the robot's swerve drive functionality.
 * It handles drive control, path following, and odometry management.
 */
public class Swerve extends SubsystemBase {
    // Singleton instance
    private static Swerve instance;

    // Hardware
    private final SwerveDrive swerveDrive;

    /**
     * Creates a new Swerve subsystem and configures the drive system.
     * Initializes telemetry, drive configuration, and control parameters.
     */
    public Swerve() {
        // Configure telemetry
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Initialize swerve drive
        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(
                            RobotConstants.MAX_SPEED,
                            new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
                                    Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        // Configure drive parameters
        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
        swerveDrive.pushOffsetsToEncoders();

        // Set up path planner
        setupPathPlanner();
    }

    // Singleton access
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    // Drive control methods
    /**
     * Creates a command for field-oriented driving using the provided velocity
     * supplier.
     * 
     * @param velocity Supplier for chassis speeds
     * @return Command for field-oriented drive control
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }

    /**
     * Locks the swerve modules in an X pattern to prevent movement.
     */
    public void lockWheels() {
        swerveDrive.lockPose();
    }

    /**
     * Resets the robot's odometry to the origin (0,0,0).
     */
    public void resetOdometry() {
        swerveDrive.resetOdometry(new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
                Rotation2d.fromDegrees(0)));
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return The current pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Resets the robot's odometry to the specified pose.
     * 
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Gets the robot's current velocity.
     * 
     * @return The robot's velocity as ChassisSpeeds
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Gets the underlying SwerveDrive object for direct access.
     * 
     * @return The SwerveDrive instance
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    // Autonomous configuration
    /**
     * Configures PathPlanner autonomous functionality.
     * Sets up path following, controllers, and alliance-specific behavior.
     */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose,
                    // Robot pose supplier
                    this::resetOdometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built in path following controller for holonomic
                            // drive trains
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)
                    // Rotation PID constants
                    ),
                    config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), 4.0,
                swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
    }

    /**
     * Finds the closest point from an array of positions to the robot's current
     * position.
     * Uses squared distance for optimization to avoid expensive square root
     * operations.
     * 
     * @param points Array of Pose2d positions to check
     * @return The closest Pose2d from the array, or null if the array is empty
     */
    public Pose2d getClosestPoint(Pose2d[] points) {
        if (points == null || points.length == 0) {
            return null;
        }

        Pose2d currentPose = getPose();
        double minDistSq = Double.MAX_VALUE;
        Pose2d closest = null;

        for (Pose2d point : points) {
            // Calculate squared distance to avoid square root operation
            double dx = currentPose.getX() - point.getX();
            double dy = currentPose.getY() - point.getY();
            double distSq = dx * dx + dy * dy;

            if (distSq < minDistSq) {
                minDistSq = distSq;
                closest = point;
            }
        }

        return closest;
    }

    /**
     * Finds the closest point from an array of field POIs to the robot's current
     * position.
     * Takes alliance into account and optimizes calculations for frequent calls.
     * 
     * @param points Array of POI objects to check
     * @return The Pose2d of the closest POI, or null if the array is empty
     */
    public Pose2d getClosestPOI(FieldConstants.POI[] points) {
        if (points == null || points.length == 0) {
            return null;
        }

        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d currentPose = getPose();
        double minDistSq = Double.MAX_VALUE;
        int closestIndex = -1;

        // Cache current position for optimization
        final double robotX = currentPose.getX();
        final double robotY = currentPose.getY();

        for (int i = 0; i < points.length; i++) {
            Pose2d pointPose = points[i].get(alliance);

            // Calculate squared distance to avoid square root operation
            double dx = robotX - pointPose.getX();
            double dy = robotY - pointPose.getY();
            double distSq = dx * dx + dy * dy;

            if (distSq < minDistSq) {
                minDistSq = distSq;
                closestIndex = i;
            }
        }

        return closestIndex >= 0 ? points[closestIndex].get(alliance) : null;
    }

    /**
     * Finds the closest POI with a specific tag to the robot's current position.
     * Highly optimized for frequent calls in control loops.
     * 
     * @param points Array of POI objects to check
     * @param tag    The tag to filter by, or null to check all POIs
     * @return The Pose2d of the closest matching POI, or null if none found
     */
    public Pose2d getClosestPOIByTag(FieldConstants.POI[] points, String tag) {
        if (points == null || points.length == 0) {
            return null;
        }

        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Pose2d currentPose = getPose();
        double minDistSq = Double.MAX_VALUE;
        int closestIndex = -1;

        // Cache current position for optimization
        final double robotX = currentPose.getX();
        final double robotY = currentPose.getY();

        for (int i = 0; i < points.length; i++) {
            // Skip POIs that don't match the requested tag
            if (tag != null && !points[i].getTag().equals(tag)) {
                continue;
            }

            Pose2d pointPose = points[i].get(alliance);

            // Calculate squared distance to avoid square root operation
            double dx = robotX - pointPose.getX();
            double dy = robotY - pointPose.getY();
            double distSq = dx * dx + dy * dy;

            if (distSq < minDistSq) {
                minDistSq = distSq;
                closestIndex = i;
            }
        }

        return closestIndex >= 0 ? points[closestIndex].get(alliance) : null;
    }

    /**
     * Creates a supplier that returns a Rotation2d pointing toward the closest POI
     * with a specific tag.
     * 
     * @param points Array of POIs to target
     * @param tag    The tag to filter by, or null to consider all POIs
     * @return A supplier that provides the rotation toward the closest matching POI
     *         when called
     */
    public Supplier<Rotation2d> createPointToClosestSupplier(FieldConstants.POI[] points, String tag) {
        return () -> {
            Pose2d closestPose = getClosestPOIByTag(points, tag);
            if (closestPose == null) {
                return new Rotation2d(); // Default to 0 if no points available
            }
            return closestPose.getRotation();
        };
    }

    // For backward compatibility
    public Supplier<Rotation2d> createPointToClosestSupplier(FieldConstants.POI[] points) {
        return createPointToClosestSupplier(points, null);
    }
}
