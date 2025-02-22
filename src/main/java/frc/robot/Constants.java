package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Elevator.ElevatorLocation;

public class Constants {
    public static class Drivetrain {
        // This constraint is solely used in the driveToPose() function by PPLib.
        public static final PathConstraints DRIVE_TO_POSE_CONSTRAINTS = new PathConstraints(3, 3, 540, 720, 12, false);
    }

    public static class Vision {
        public static final String CAMERA_NAME = "BR_Cam";
        // TU12 says that the Taiwan regional will use the AndyMark field
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.26, 0, 0.50),
            new Rotation3d(Degrees.of(0), Degrees.of(40), Degrees.of(0)));

        public static class Simulated {
            public static final int WIDTH = 1280;
            public static final int HEIGHT = 800;
            public static final Rotation2d FOV = Rotation2d.fromDegrees(100);
            public static final int FPS = 30;
        }
    }

    public static class ReefSelector {
        // Apriltag-relative offsets used to calculate the robot's desired Pose when scoring from its AprilTag.
        // X_OFFSET should always be negative (away from the tag).
        public static final Distance X_OFFSET = Meters.of(-0.5);
        // Y_OFFSET is assumed to be positive (LEFT reef). The reason for this is that A1, the default scoring location, is on the left.
        public static final Distance Y_OFFSET = Meters.of(0.16);

        // These are the AprilTag IDs of the desired scoring locations.
        // A-B shares one AprilTag, C-D shares one AprilTag, and so on.
        public static final int[] BLUE_ALLIANCE_TAGS = {18, 17, 22, 21, 20, 19};
        public static final int[] RED_ALLIANCE_TAGS = {7, 8, 9, 10, 11, 6};

        // The elevator locations for levels 1 to 4. Level 0 does not exist, so null is placed instead.
        // This array MUST have 5 elements (0, 1, 2, 3, 4).
        public static final ElevatorLocation[] ELEVATOR_LOCATIONS = {null, ElevatorLocation.BOTTOM, ElevatorLocation.BOTTOM, ElevatorLocation.MID, ElevatorLocation.TOP};
    }
}
