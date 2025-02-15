package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefSelector {
    // Positive doubles of apriltag-relative offsets used to calculate the robot's desired Pose when scoring from its AprilTag.
    protected static final double X_OFFSET = 0.2;
    protected static final double Y_OFFSET = 0.2;
    protected static final int[] blueAllianceTags = {18, 17, 22, 21, 20, 19};
    protected static final int[] redAllianceTags = {7, 8, 9, 10, 11, 6};
    protected static final AprilTagFieldLayout fieldLayout = CommandSwerveDrivetrain.aprilTagFieldLayout;

    // Reefs are labeled A-L based on the annotated PathPlanner grid.
    private char selectedReef = 'A';
    private int selectedLevel = 1;
    private Pose2d selectedPose = new Pose2d();

    public ReefSelector() {
        calculatePose();
    }

    private void calculatePose() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Capital A has the ASCII code 65. int 65/2 = 32.
        Pose2d aprilTagPose = alliance == Alliance.Red ?
            fieldLayout.getTagPose(redAllianceTags[(this.selectedReef - 1) / 2 - 32]).get().toPose2d() :
            fieldLayout.getTagPose(blueAllianceTags[(this.selectedReef - 1) / 2 - 32]).get().toPose2d();

        this.selectedPose = aprilTagPose.plus(new Transform2d(Meters.of(X_OFFSET), Meters.of((this.selectedReef - 1) % 2 == 0 ? -Y_OFFSET : Y_OFFSET), Rotation2d.fromDegrees(0)));

        CommandSwerveDrivetrain.setSelectedReef(this.selectedPose);
        SmartDashboard.putNumber("ReefSelector/Level", this.selectedLevel);
    }

    public void goRight() {
        this.selectedReef = (char)(this.selectedReef + 1) > 'L' ? 'A' : (char)(this.selectedReef + 1);
        calculatePose();
    }

    public void goLeft() {
        this.selectedReef = (char)(this.selectedReef - 1) < 'A' ? 'L' : (char)(this.selectedReef - 1);
        calculatePose();
    }

    public void goUp() {
        this.selectedLevel = this.selectedLevel + 1 > 4 ? this.selectedLevel : this.selectedLevel + 1;
        calculatePose();
    }

    public void goDown() {
        this.selectedLevel = this.selectedLevel - 1 < 1 ? this.selectedLevel : this.selectedLevel - 1;
        calculatePose();
    }

    public void setReef(char reef, int level) {
        this.selectedReef = reef;
        this.selectedLevel = level;
        calculatePose();
    }

    public void setReef(char reef) {
        this.selectedReef = reef;
        calculatePose();
    }

    public void setReef(int level) {
        this.selectedLevel = level;
        calculatePose();
    }
}
