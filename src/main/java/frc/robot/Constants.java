package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.Elevator.ElevatorLocation;

public class Constants {
    public static class Drivetrain {
        public static final LinearVelocity JOYSTICK_POV_VELOCITY = MetersPerSecond.of(0.2);

        public static final PPHolonomicDriveController PP_HOLONOMIC_DRIVE_CONTROLLER = new PPHolonomicDriveController(
            // PPHolonomicController is the built in path following controller for holonomic drive trains.
            // This does not affect DriveToPose.
            new PIDConstants(5, 0.0, 0.0), // Translation PID constants500
            new PIDConstants(2.5, 0.0, 0.0) // Rotation PID constants500
        );

        public static class DriveToPose {
            // Whether to use PPLib in the driveToPose() function.
            public static final boolean USE_PPLIB = false;// maybe try once!
            // This constraint is used in the driveToPose() function by PPLib AND PIDControl.
            public static final PathConstraints CONSTRAINTS = new PathConstraints(2.5, 1.5, 360,
                    360, 12, false);// 2.2

            // These constraints are solely used in the driveToPose() function by PIDControl.
            public static final double TRANSLATION_kP = 2.3;//10
            public static final double TRANSLATION_kI = 0.0;//.5
            public static final double TRANSLATION_kD = 0.0;//0
            public static final double TRANSLATION_TOLERANCE = 0.04;//0.02

            public static final double ROTATION_kP = 1;//3
            public static final double ROTATION_kI = 0.0;
            public static final double ROTATION_kD = 0.0;//0
            public static final double ROTATION_TOLERANCE = Units.degreesToRadians(2.5);//2
        }
    }

    public static class VisionDown1Cam {
        public static final String CAMERA_NAME = "DownCam1";
        // TU12 says that the Taiwan regional will use the AndyMark field
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(-0.300, -0.075, 0.202),
                new Rotation3d(Degrees.of(0), Degrees.of(-12), Degrees.of(160)));
        // Distance from camera to tag
        public static final Distance MAX_DISTANCE = Meters.of(1);
        public static final double MAX_AMBIGUITY = 0.15;

        public static class Simulated {
            public static final int WIDTH = 1280;
            public static final int HEIGHT = 800;
            public static final Rotation2d FOV = Rotation2d.fromDegrees(60);
            public static final int FPS = 30;
        }
    }

    public static class VisionDown2Cam {
        public static final String CAMERA_NAME = "DownCam2";
        // TU12 says that the Taiwan regional will use the AndyMark field
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(-0.300, -0.145, 0.202),
                new Rotation3d(Degrees.of(0), Degrees.of(-12), Degrees.of(200)));//angle0to-12
        // Distance from camera to tag
        public static final Distance MAX_DISTANCE = Meters.of(1);
        public static final double MAX_AMBIGUITY = 0.15;

        public static class Simulated {
            public static final int WIDTH = 1280;
            public static final int HEIGHT = 800;
            public static final Rotation2d FOV = Rotation2d.fromDegrees(60);
            public static final int FPS = 30;
        }
    }

    public static class ReefSelector {
        // Apriltag-relative offsets used to calculate the robot's desired Pose when scoring from its AprilTag.
        // X_OFFSET should always be negative (away from the tag).
        public static Distance X_OFFSET = Meters.of(-0.55);//-0.60
   
        // LEFT_Y_OFFSET is assumed to be positive (LEFT reef). The reason for this is that A1, the default scoring location, is on the left.
        public static Distance LEFT_Y_OFFSET = Meters.of(0.08);//16
        public static Distance RIGHT_Y_OFFSET = Meters.of(-0.29);//-16

        // These are the AprilTag IDs of the desired scoring locations.
        // A-B shares one AprilTag, C-D shares one AprilTag, and so on.
        public static final int[] BLUE_ALLIANCE_TAGS = { 18, 17, 22, 21, 20, 19 };
        public static final int[] RED_ALLIANCE_TAGS = { 7, 8, 9, 10, 11, 6 };

        // The elevator locations for levels 1 to 4. Level 0 does not exist, so null is placed instead.
        // This array MUST have 5 elements (0, 1, 2, 3, 4).
        public static final ElevatorLocation[] ELEVATOR_LOCATIONS = { null, null, ElevatorLocation.BOTTOM, ElevatorLocation.MID, ElevatorLocation.TOP };
    }
}
