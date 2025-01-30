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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}