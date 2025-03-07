package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ExtendedController extends CommandXboxController {
    double deadband = 0.1;

    public ExtendedController(int port) {
        super(port);
    }

    public ExtendedController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    /**
     * Get the X axis value of left side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), deadband);
    }

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), deadband);
    }

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), deadband);
    }

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), deadband);
    }

    /**
     * Get the left trigger axis value of the controller. Note that this axis is
     * bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(super.getLeftTriggerAxis(), deadband);
    }

    /**
     * Get the right trigger axis value of the controller. Note that this axis is
     * bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(super.getRightTriggerAxis(), deadband);
    }

    // To construct new button triggers, use the example below. You can then use the button as normal.

    /**
     * Constructs a Trigger instance around the BackRight (13) button's digital signal.
     */
    public Trigger menu() {
        return menu(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the BackRight (13) button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the A button's digital signal attached
     *     to the given loop.
     */
    public Trigger menu(EventLoop loop) {
        // Put the button ID here.
        return button(8, loop);
    }

}
