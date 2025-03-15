package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation3d;
import swervelib.SwerveDrive;

public final class Apriltag extends SubsystemBase {
    private static Apriltag apriltag = null;
    private static SwerveDrive swerveDrive = null;

    private class Camera {
        String name;
        DoubleArraySubscriber translationSub;
        DoubleArraySubscriber rotationSub;
        DoubleSubscriber delaySub;
        BooleanSubscriber tagDetectedSub;

        private Camera(String name) {
            this.name = name;
            NetworkTableInstance nt = NetworkTableInstance.getDefault();

            NetworkTable table = nt.getTable("/chalkydri/robot_pose/" + name);
            DoubleArrayTopic translation = table.getDoubleArrayTopic("translation");
            DoubleArrayTopic rotation = table.getDoubleArrayTopic("rotation");
            BooleanTopic tagDetected = table.getBooleanTopic("tag_detected");
            DoubleTopic delay = table.getDoubleTopic("delay");

            translationSub = translation.subscribe(null);
            rotationSub = rotation.subscribe(null);
            tagDetectedSub = tagDetected.subscribe(false);
            delaySub = delay.subscribe(0.0);
        }

        public String getName() {
            return name;
        }

        public double[] getTranslationSub() {
            return translationSub.get();
        }

        public double[] getRotationSub() {
            return rotationSub.get();
        }

        public double getDelaySub() {
            return delaySub.get();
        }

        public boolean getTagDetectedSub() {
            return tagDetectedSub.get();
        }
    }

    // Array to hold pairs of camera names and their corresponding pose estimators
    private static Camera[] cameras;

    public static Apriltag getInstance() {
        if (apriltag == null)
            apriltag = new Apriltag();
        if (swerveDrive == null)
            swerveDrive = Swerve.getInstance().getSwerveDrive();

        return apriltag;
    }

    @Override
    public void periodic() {
        // Periodically update the pose estimate
        for (Camera camera : cameras) {
            String cameraName = camera.name;
            
            boolean tagDetected = camera.getTagDetectedSub();
            double[] translation = camera.getTranslationSub();
            double[] rotation = camera.getRotationSub();
            double delay = camera.getDelaySub();

            if (tagDetected && translation != null && rotation != null) {
                // Update the pose estimator with the new data
                Pose3d pose = new Pose3d(translation[0], translation[1], translation[2], new Rotation3d(rotation[0], rotation[1], rotation[2]));
                swerveDrive.addVisionMeasurement(pose.toPose2d(), Timer.getFPGATimestamp() - delay);
            }
        }
    }
}
