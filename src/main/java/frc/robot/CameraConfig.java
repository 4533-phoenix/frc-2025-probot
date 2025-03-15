package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * The CameraConfig class holds the configuration for a camera,
 * including its name and its transformation relative to the robot.
 */
public class CameraConfig {
  // The name of the camera
  String name;
  
  // The transformation from the camera to the robot
  Transform3d camToRobot;

  /**
   * Constructs a new CameraConfig with the specified name and transformation.
   *
   * @param name the name of the camera
   * @param camToRobot the transformation from the camera to the robot
   */
  public CameraConfig(String name, Transform3d camToRobot) {
    this.name = name;
    this.camToRobot = camToRobot;
  }

  /**
   * Returns the name of the camera.
   *
   * @return the name of the camera
   */
  public String getName() {
    return name;
  }

  /**
   * Returns the transformation from the camera to the robot.
   *
   * @return the transformation from the camera to the robot
   */
  public Transform3d getCamToRobot() {
    return camToRobot;
  }
}
