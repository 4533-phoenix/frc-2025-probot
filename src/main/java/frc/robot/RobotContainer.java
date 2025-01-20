package frc.robot;

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
 * It serves as the main robot configuration class.
 */
public class RobotContainer {
    // Controls
    private final CommandXboxController driver = new CommandXboxController(0);

    // Subsystems
    private final Swerve swerveDrive = Swerve.getInstance();

    // Streams
    private final SwerveInputStream driveDirectAngle = SwerveInputStream.of(swerveDrive.getSwerveDrive(), driver::getLeftY, driver::getLeftX)
            .cubeTranslationControllerAxis(true)
            .scaleTranslation(0.5)
            .withControllerHeadingAxis(driver::getRightX, driver::getRightY)
            .cubeRotationControllerAxis(true)
            // .scaleRotation(0.5)
            .deadband(OIConstants.DRIVER_DEADBAND)
            .allianceRelativeControl(true)
            .headingWhile(true);

    /**
     * Creates the robot container and configures button bindings.
     * Initializes all enabled subsystems and their default commands.
     */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        driver.setRumble(RumbleType.kBothRumble, 0.0);

        configureBindings();
    }

    /**
     * Configures button bindings for all controllers.
     */
    private void configureBindings() {
        Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(driveDirectAngle);
        swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

        driver.x().whileTrue(Commands.runOnce(swerveDrive::lockWheels, swerveDrive).repeatedly());
        driver.start().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));
    }

    /**
     * @return Command to run during autonomous phase
     */
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
