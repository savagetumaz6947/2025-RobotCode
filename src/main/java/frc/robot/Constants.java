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
        public static final PathConstraints DRIVE_TO_POSE_CONSTRAINTS = new PathConstraints(1, 1.5, 180,
            240, 12, false);//1.1.540.720
    }

    public static class VisionDownCam {
        public static final String CAMERA_NAME = "DownCam";
        // TU12 says that the Taiwan regional will use the AndyMark field
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(-0.255,    0.175, 0.815),
            new Rotation3d(Degrees.of(0), Degrees.of(30), Degrees.of(180)));

        public static class Simulated {
            public static final int WIDTH = 1280;
            public static final int HEIGHT = 800;
            public static final Rotation2d FOV = Rotation2d.fromDegrees(100);
            public static final int FPS = 30;
        }
    }

    public static class VisionUpCam {
        public static final String CAMERA_NAME = "UpCam";
        // TU12 says that the Taiwan regional will use the AndyMark field
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(-0.265, 0.175, 0.895),
            new Rotation3d(Degrees.of(0), Degrees.of(-25), Degrees.of(180)));

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
        public static final Distance X_OFFSET = Meters.of(-0.60);
        // Y_OFFSET is assumed to be positive (LEFT reef). The reason for this is that A1, the default scoring location, is on the left.
        public static final Distance Y_OFFSET = Meters.of(0.16);

        // These are the AprilTag IDs of the desired scoring locations.
        // A-B shares one AprilTag, C-D shares one AprilTag, and so on.
        public static final int[] BLUE_ALLIANCE_TAGS = {18, 17, 22, 21, 20, 19};
        public static final int[] RED_ALLIANCE_TAGS = {7, 8, 9, 10, 11, 6};

        // The elevator locations for levels 1 to 4. Level 0 does not exist, so null is placed instead.
        // This array MUST have 5 elements (0, 1, 2, 3, 4).
        public static final ElevatorLocation[] ELEVATOR_LOCATIONS = {null, null, ElevatorLocation.BOTTOM, ElevatorLocation.MID, ElevatorLocation.TOP};
    }
}
