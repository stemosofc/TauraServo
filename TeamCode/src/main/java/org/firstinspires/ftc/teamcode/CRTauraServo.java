package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * CRTauraServo
 * <p>
 * A wrapper for an FTC {@link CRServo} (Continuous Rotation Servo) that emulates
 * standard servo position control by using an external {@link AnalogInput} sensor
 * for closed-loop feedback.
 * <p>
 * A standard {@link CRServo} only accepts power values (-1.0 to 1.0).
 * This class adds PIDF-based position control ({@link #setPosition(double)})
 * by reading the servo's current angle from the feedback sensor.
 * <p>
 * To use position control, you MUST:
 * 1. Use a constructor that initializes the {@link AnalogInput} sensor.
 * 2. Call {@link #setPIDFCoefficients(PIDFCoefficients)} to enable the controller.
 */
public class CRTauraServo {

    // --- Class Fields ---

    /** Linear calibration coefficients that map sensor voltage to a normalized "universal" position in [0..1]. */
    private static final double A = -0.06637714;
    private static final double B =  0.34132813;

    /** Maximum mechanical/angular travel (in degrees) used to convert universal [0..1] into raw degrees. */
    private static final double MAX_ANGLE_DEGREES_ANGULAR = 315;

    /**
     * Effective span (in degrees) used for "unwrapping" the incremental measurement.
     * Choose the span that best represents one full cycle of your analog sensor (e.g., ~330° for many pots).
     */
    private static final double MAX_ANGLE_DEGREES_CONTINUOUS = 330;

    /** Last sampled raw angle (in degrees) used to compute deltas between samples. */
    private double lastRawAngleInDegrees = 0.0;

    /** Accumulated continuous displacement (in degrees) since start (or since last implicit reset). */
    private double deltaPositionInDegrees = 0.0;

    /** Optional analog input sensor used as a feedback source; if null, read functions return NaN. */
    private AnalogInput analogFeedbackSensor;

    /** The underlying servo instance obtained from the hardware map. */
    private final CRServo servo;

    /**
     * The internal PIDF controller used for closed-loop position control.
     * Remains null until {@link #setPIDFCoefficients(PIDFCoefficients)} is called.
     */
    private PIDFController pidf;

    /**
     * Constructor that initializes the servo using an existing CRServo instance.
     * @param servo The CRServo instance to be used.
     */
    public CRTauraServo(CRServo servo) { this.servo = servo; }

    /**
     * Constructor that initializes the servo and feedback sensor using the HardwareMap.
     * @param hMap The robot's hardware map.
     * @param deviceNameServo The name of the configured CRServo in the hardware.
     * @param deviceNameAnalogInput The name of the configured AnalogInput in the hardware.
     */
    public CRTauraServo(HardwareMap hMap, String deviceNameServo, String deviceNameAnalogInput) {
        servo = hMap.get(CRServo.class, deviceNameServo);
        analogFeedbackSensor = hMap.get(AnalogInput.class, deviceNameAnalogInput);
    }

    /**
     * Constructor that initializes the servo using the HardwareMap.
     * Note: This constructor does not configure an analog feedback sensor.
     * @param hMap The robot's hardware map.
     * @param deviceName The name of the configured CRServo in the hardware.
     */
    public CRTauraServo(HardwareMap hMap, String deviceName) {
        servo = hMap.get(CRServo.class, deviceName);
    }

    /**
     * Sets the raw power of the servo, bypassing the PIDF controller.
     * This is the default behavior of a {@link CRServo}.
     *
     * @param power The desired power for the servo, from -1.0 to 1.0.
     */
    public void setPower(double power){
        servo.setPower(power);
    }

    /**
     * Gets the current power being applied to the servo.
     * @return The current power, from -1.0 to 1.0.
     */
    public double getPower() {
        return servo.getPower();
    }

    /**
     * Sets the servo direction (FORWARD/REVERSE).
     * @param direction The direction the servo should rotate.
     */
    public void setDirection(DcMotorSimple.Direction direction) { servo.setDirection(direction); }

    /**
     * Sets the PIDF coefficients for the position controller.
     * This method MUST be called before using {@link #setPosition(double)} or
     * {@link #setPIDFTolerance(double)}.
     *
     * @param coefficients The PIDF coefficients to use.
     */
    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        pidf = new PIDFController(coefficients);
    }

    /**
     * Sets the tolerance for the PIDF controller.
     *
     * @param tolerance The position tolerance (in degrees) for the controller.
     * @throws IllegalStateException if {@link #setPIDFCoefficients(PIDFCoefficients)}
     * has not been called first.
     */
    public void setPIDFTolerance(double tolerance) {
        assertPIDFIsInitialized("setPIDFTolerance");
        pidf.setTolerance(tolerance);
    }

    /**
     * Commands the servo to move to a target position (in degrees) using
     * the internal PIDF controller.
     * <p>
     * This method provides position-based control for a {@link CRServo}, which
     * normally only accepts power-based control.
     *
     * @param position The target position in degrees (based on the accumulated value from
     * {@link #getIncrementalPositionInDegrees()}).
     * @throws IllegalStateException if {@link #setPIDFCoefficients(PIDFCoefficients)}
     * has not been called, or if no {@link AnalogInput} sensor was configured.
     */
    public void setPosition(double position) {
        assertPIDFIsInitialized("setPosition");

        double currentPosition = getIncrementalPositionInDegrees();

        // Safety check: If sensor reading is NaN (e.g., unplugged), stop the servo
        if (Double.isNaN(currentPosition)) {
            servo.setPower(0);
            return;
        }

        double power = pidf.calculate(currentPosition, position);
        power = Math.max(-1, Math.min(1, power));
        servo.setPower(power);
    }

    /**
     * Returns the continuous incremental displacement in degrees.
     * Applies "unwrap" across the 0° boundary so small real movements near wrap-around are accumulated correctly.
     * If the analog reading is unavailable (NaN), the accumulator is preserved and returned unchanged.
     * <p>
     * Use for Servo in Continous mode.
     */
    public double getIncrementalPositionInDegrees() {
        double actualAngleInDegrees = getRawPositionInDegrees();

        double delta = actualAngleInDegrees - lastRawAngleInDegrees;
        double halfRange = MAX_ANGLE_DEGREES_CONTINUOUS / 2.0;

        // Unwrap: if the jump crosses the wrap boundary, choose the shortest equivalent delta
        if (delta >  halfRange) delta -= MAX_ANGLE_DEGREES_CONTINUOUS;
        else if (delta < -halfRange) delta += MAX_ANGLE_DEGREES_CONTINUOUS;

        deltaPositionInDegrees += delta;
        lastRawAngleInDegrees = actualAngleInDegrees;

        return deltaPositionInDegrees;
    }

    /**
     * Returns the current raw angle (in degrees) within the expected angular span.
     * Converts the normalized universal position [0..1] to degrees using MAX_ANGLE_DEGREES_ANGULAR.
     * If the universal position is NaN, returns NaN.
     * @return The raw angle in degrees, or Double.NaN.
     */
    public double getRawPositionInDegrees() {
        return getUniversalPosition() * MAX_ANGLE_DEGREES_ANGULAR;
    }

    /**
     * Returns the normalized "universal" position in [0..1] computed from the analog voltage
     * using the linear calibration model A + B * voltage. If no sensor is set, returns NaN.
     * @return Normalized position [0..1] or Double.NaN if the sensor is null.
     */
    public double getUniversalPosition() {
        return A + B * getVoltageFromAnalogSensor();
    }

    /**
     * Reads the analog sensor voltage. If no sensor is configured, returns NaN instead of throwing.
     * @return The sensor voltage, or Double.NaN if the sensor is null.
     */
    private double getVoltageFromAnalogSensor() {
        try {
            return analogFeedbackSensor.getVoltage();
        } catch(NullPointerException e) {
            return Double.NaN;
        }
    }

    /**
     * Checks if the PIDF controller has been initialized.
     * @param methodName The name of the calling method, for a descriptive error message.
     * @throws IllegalStateException if {@link #pidf} is null.
     */
    private void assertPIDFIsInitialized(String methodName) {
        if (pidf == null)
            throw new IllegalStateException("CRTauraServo Error: " + methodName + "() was called, " +
                    "but PIDF coefficients have not been set. " +
                    "You must call setPIDFCoefficients() first in your OpMode.");
    }
}