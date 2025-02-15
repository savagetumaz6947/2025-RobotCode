package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorLocation;

public class ReefSelector {
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
            Constants.Vision.APRIL_TAG_FIELD_LAYOUT.getTagPose(Constants.ReefSelector.RED_ALLIANCE_TAGS[(this.selectedReef - 1) / 2 - 32]).get().toPose2d() :
            Constants.Vision.APRIL_TAG_FIELD_LAYOUT.getTagPose(Constants.ReefSelector.BLUE_ALLIANCE_TAGS[(this.selectedReef - 1) / 2 - 32]).get().toPose2d();

        this.selectedPose = aprilTagPose.rotateAround(aprilTagPose.getTranslation(), Rotation2d.fromDegrees(180))
            .plus(new Transform2d(Constants.ReefSelector.X_OFFSET,
                (this.selectedReef - 1) % 2 == 0 ? Constants.ReefSelector.Y_OFFSET : Constants.ReefSelector.Y_OFFSET.unaryMinus(),
                Rotation2d.fromDegrees(0)));

        CommandSwerveDrivetrain.setSelectedReef(this.selectedPose);
        SmartDashboard.putString("ReefSelector/Selected", "" + this.selectedReef + this.selectedLevel);
    }

    public Pose2d getSelectedPose() {
        return selectedPose;
    }

    public ElevatorLocation getElevatorLocation() {
        return Constants.ReefSelector.ELEVATOR_LOCATIONS[selectedLevel];
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
