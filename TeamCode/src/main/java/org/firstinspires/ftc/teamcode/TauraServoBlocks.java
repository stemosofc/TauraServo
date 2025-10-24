package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import java.util.concurrent.ConcurrentHashMap;
import java.util.Map;

public class TauraServoBlocks extends BlocksOpModeCompanion {

    private static final int GREEN = 147;
    private static final int PURPLE = 289;
    private static final Map<String, TauraServo> SERVOS = new ConcurrentHashMap<>();

    private static TauraServo getOrCreate(String servoName) {
        if (servoName == null || servoName.isEmpty()) return null;
        return SERVOS.computeIfAbsent(servoName, name -> {
            Servo base = hardwareMap.get(Servo.class, name);
            return new TauraServo(base);
        });
    }

    @ExportToBlocks(
            heading = "set",
            comment = "Bind an AnalogInput (potentiometer) to a specific TauraServo by names.",
            tooltip = "Enables universal/raw/incremental readings for that servo.",
            parameterLabels = {"servoName", "analogInputName"},
            color = PURPLE
    )
    public static void setAnalogFeedbackSensor(String servoName, String analogInputName) {
        TauraServo taura = getOrCreate(servoName);
        AnalogInput ai = hardwareMap.get(AnalogInput.class, analogInputName);
    }

    @ExportToBlocks(
            heading = "set",
            comment = "Set direction (FORWARD/REVERSE) for the specified servo.",
            tooltip = "Mirrors logical [0..1] range.",
            parameterLabels = {"servoName", "direction"},
            color = GREEN
    )
    public static void setDirection(String servoName, Servo.Direction direction) {
        TauraServo taura = getOrCreate(servoName);
        taura.setDirection(direction);
    }

    @ExportToBlocks(
            heading = "set",
            comment = "Scale logical motion range for the specified servo.",
            tooltip = "Constrain movement without changing ServoType.",
            parameterLabels = {"servoName", "min", "max"},
            color = GREEN
    )
    public static void scaleRange(String servoName, double min, double max) {
        TauraServo taura = getOrCreate(servoName);
        taura.scaleRange(min, max);
    }

    @ExportToBlocks(
            heading = "set",
            comment = "Set target position in [0..1] for the specified servo.",
            tooltip = "0.0 = min pulse, 1.0 = max pulse (per ServoType).",
            parameterLabels = {"servoName", "position"},
            color = GREEN
    )
    public static void setPosition(String servoName, double position) {
        TauraServo taura = getOrCreate(servoName);
        taura.setPosition(position);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Get last commanded position [0..1] for the specified servo.",
            tooltip = "Returns the target position previously sent.",
            parameterLabels = {"servoName"},
            color = GREEN
    )
    public static double getLastSetPosition(String servoName) {
        TauraServo taura = getOrCreate(servoName);
        return taura.getPosition();
    }

    @ExportToBlocks(
            heading = "",
            comment = "Get continuous incremental angle (deg, unwrapped) for the specified servo.",
            tooltip = "Accumulates motion over time.",
            parameterLabels = {"servoName"},
            color = GREEN
    )
    public static double getIncrementalPositionInDegrees(String servoName) {
        TauraServo taura = getOrCreate(servoName);
        return taura.getIncrementalPositionInDegrees();
    }

    @ExportToBlocks(
            heading = "",
            comment = "Get raw angle (deg) within sensor span for the specified servo.",
            tooltip = "Derived from analog feedback mapping.",
            parameterLabels = {"servoName"},
            color = GREEN
    )
    public static double getRawPositionInDegrees(String servoName) {
        TauraServo taura = getOrCreate(servoName);
        return taura.getRawPositionInDegrees();
    }

    @ExportToBlocks(
            heading = "",
            comment = "Get normalized universal position [0..1] for the specified servo.",
            tooltip = "Computed from analog voltage (A + B*V).",
            parameterLabels = {"servoName"},
            color = GREEN
    )
    public static double getUniversalPosition(String servoName) {
        TauraServo taura = getOrCreate(servoName);
        return taura.getUniversalPosition();
    }
}
