package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

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

    // Drive control configuration
    private final SwerveInputStream driveDirectAngle = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
            () -> driver.getLeftY() * -1,
            () -> driver.getLeftX() * -1)
            .cubeTranslationControllerAxis(true)
            .scaleTranslation(0.5)
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
        Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(driveDirectAngle);
        swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

        // Configure button bindings
        driver.x().whileTrue(Commands.runOnce(swerveDrive::lockWheels, swerveDrive).repeatedly());
        driver.start().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));
        driver.back().whileTrue(
                swerveDrive.driveToPose(
                        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0))));
    }

    /**
     * @return Command to run during autonomous phase
     */
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
