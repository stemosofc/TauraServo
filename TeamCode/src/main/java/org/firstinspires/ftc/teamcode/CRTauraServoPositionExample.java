package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * This OpMode demonstrates how to move a CRTauraServo to specific positions
 * using PID control.
 *
 * Press:
 *  - (A) to move the servo to position 500
 *  - (B) to move it back to position 0
 *
 * The encoder position is displayed on the Driver Hub.
 */
@TeleOp(name="CRTaura Position Control", group="Example")
public class CRTauraServoPositionExample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the target position variable.
        double position = 0;

        // Initialize the Taura Servo and its associated encoder.
        CRTauraServo tauraServo = new CRTauraServo(hardwareMap,"CRServo","absoluteEncoder");

        // Set PIDF coefficients for the position control.
        // These values can be tuned for smoother or more responsive movement.
        tauraServo.setPIDFCoefficients(new PIDFCoefficients(0.02,0,0,0.0));



        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            // Command the servo to move to the target position.
            tauraServo.setPosition(position);

            // Press A to move to position 500
            if(gamepad1.a) {
                position = 500;
            }

            // Press B to return to position 0
            else if(gamepad1.b) {
                position = 0;
            }

            //encoder telemetry that will appear on the driver hub
            telemetry.addData("Pos lida", "%.3f", tauraServo.getUniversalPosition());
            telemetry.addData("Pos graus absoluto", "%.3f", tauraServo.getRawPositionInDegrees());
            telemetry.addData("Pos graus incremental", "%.3f", tauraServo.getIncrementalPositionInDegrees());
            telemetry.update();
        }
    }

}