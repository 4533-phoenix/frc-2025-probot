package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
<<<<<<< HEAD
import frc.robot.subsystems.Apriltag;
=======
import frc.robot.helpers.CustomSwerveInput;
>>>>>>> origin/feat/chalkydri
import frc.robot.subsystems.Swerve;

/**
 * This class contains the robot's subsystems, commands, and button mappings.
 * It serves as the main robot configuration class and manages:
 * - Controller setup and bindings
 * - Subsystem initialization
 * - Default commands
 * - Autonomous selection
 */
public class RobotContainer {
    // Controller configuration
    private final CommandXboxController driver = new CommandXboxController(0);

    // Subsystem instances
    private final Swerve swerveDrive = Swerve.getInstance();
    private final Apriltag apriltag = Apriltag.getInstance();

    // Drive control configuration
    private final CustomSwerveInput swerveInputStream = CustomSwerveInput.of(swerveDrive.getSwerveDrive(),
            () -> driver.getLeftY() * -1,
            () -> driver.getLeftX() * -1)
            .cubeTranslationControllerAxis(true)
            .scaleTranslation(0.75)
            .cubeRotationControllerAxis(true)
            .withControllerHeadingAxis(() -> driver.getRightX() * -1, () -> driver.getRightY() * -1)
            .cubeRotationControllerAxis(true)
            .deadband(OIConstants.DRIVER_DEADBAND)
            .allianceRelativeControl(true)
            .headingWhile(true);

    /**
     * Creates the robot container and configures all robot systems.
     */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        driver.setRumble(RumbleType.kBothRumble, 0.0);

        configureBindings();
    }

    /**
     * Configures button bindings and default commands.
     * - Left stick: Translation control
     * - Right stick: Rotation control
     * - X button: Lock wheels
     * - Start button: Reset odometry
     */
    private void configureBindings() {
        // Set default drive command
        Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(swerveInputStream.copy()
                .withHeading(swerveDrive.createPointToClosestSupplier(Constants.FieldConstants.ALL_POIS))
                .allianceRelativeControl(false));
        swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

        driver.leftBumper()
                .whileTrue(swerveDrive.driveFieldOriented(swerveInputStream));

        // Configure button bindings
        driver.x().whileTrue(Commands.runOnce(swerveDrive::lockWheels, swerveDrive).repeatedly());
        driver.start().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));
        driver.back().whileTrue(
                swerveDrive.driveToPose(
                        new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
                                Rotation2d.fromDegrees(0))));
    }

    /**
     * @return Command to run during autonomous phase
     */
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
