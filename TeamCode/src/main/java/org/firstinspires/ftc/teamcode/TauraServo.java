package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.Range;

@ServoType(flavor = ServoFlavor.CUSTOM, usPulseLower = 500, usPulseUpper = 2500, xmlTag = "TaurusServo")
@DeviceProperties(name = "Taurus Servo", xmlTag = "TaurusServo", compatibleControlSystems = ControlSystem.REV_HUB)
public class TauraServo implements HardwareDevice {

    private final double MIN_POSITION = 0.0;
    private final double MAX_POSITION = 1.0;
    enum Direction { FORWARD, REVERSE }
    //------------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------------

    protected ServoController controller;
    protected int             portNumber;

    private Direction direction        = Direction.FORWARD;
    protected double          limitPositionMin = MIN_POSITION;
    protected double          limitPositionMax = MAX_POSITION;

    public TauraServo(ServoControllerEx controller, int portNumber) {
        this.controller = controller;
        this.portNumber = portNumber;
    }

    public double getPositionWithFeedback(AnalogInput analogInput) {
        return analogInput.getVoltage();
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return "Taurus Servo";
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo()
    {
        return controller.getConnectionInfo() + "; port " + portNumber;
    }

    @Override
    public int getVersion()
    {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.limitPositionMin = MIN_POSITION;
        this.limitPositionMax = MAX_POSITION;
        this.direction = Direction.FORWARD;
    }

    @Override
    public void close() { }

    //------------------------------------------------------------------------------------------------
    // Servo operations
    //------------------------------------------------------------------------------------------------

    /**
     * Get Servo Controller
     * @return servo controller
     */
    public ServoController getController() {
        return controller;
    }

    /**
     * Set the direction
     * @param direction direction
     */
    synchronized public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Get the direction
     * @return direction
     */
    public Direction getDirection() {
        return direction;
    }

    /**
     * Get Channel
     * @return channel
     */
    public int getPortNumber() {
        return portNumber;
    }

    /**
     * Commands the servo to move to a designated position. This method initiates the movement;
     * the servo will arrive at the commanded position at some later time.
     *
     * @param position the commanded servo position. Should be in the range [0.0, 1.0].
     * @see #getPosition()
     */
    synchronized public void setPosition(double position) {
        position = Range.clip(position, MIN_POSITION, MAX_POSITION);
        if (direction == Direction.REVERSE) position = reverse(position);
        double scaled = Range.scale(position, MIN_POSITION, MAX_POSITION, limitPositionMin, limitPositionMax);
        internalSetPosition(scaled);
    }

    protected void internalSetPosition(double position) {
        controller.setServoPosition(portNumber, position);
    }

    /**
     * Returns the position to which the servo was last commanded, or Double.NaN if that is
     * unavailable.
     * @return the last commanded position
     * @see #setPosition(double)
     * @see Double#isNaN(double)
     */
    synchronized public double getPosition() {
        double position = controller.getServoPosition(portNumber);
        double scaled = Range.scale(position, limitPositionMin, limitPositionMax, MIN_POSITION, MAX_POSITION);
        if (direction == Direction.REVERSE) scaled = reverse(scaled);
        return Range.clip(scaled, MIN_POSITION, MAX_POSITION);
    }

    /**
     * Automatically scales the position of the servo irrespective of whether or not reverse()
     * is called.
     * For example, if you set the scale range to [0.0, 0.5] and the servo is reversed,
     * it will be from 0.5 to 0.0, NOT 1.0 to 0.5
     */
    synchronized public void scaleRange(double min, double max) {
        min = Range.clip(min, MIN_POSITION, MAX_POSITION);
        max = Range.clip(max, MIN_POSITION, MAX_POSITION);

        if (min >= max) {
            throw new IllegalArgumentException("min must be less than max");
        }

        limitPositionMin = min;
        limitPositionMax = max;
    }

    private double reverse(double position) {
        return MAX_POSITION - position + MIN_POSITION;
    }
}
