package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/*
 * This OpMode demonstrates basic control of the CRTauraServo using power.
 *
 * Press:
 *  - (A) to spin the servo forward at full power
 *  - (B) to stop the servo
 *
 * The encoder position is displayed on the Driver Hub.
 */

@TeleOp(name="CRTaura Power Control", group="Examples")
public class CRTauraServoExample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the Taura Servo using its configured name.
        CRTauraServo tauraServo = new CRTauraServo(hardwareMap,"CRServo");


        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            // Press A to spin the servo forward at full power
            if(gamepad1.a) {
                tauraServo.setPower(1);
            }

            // Press B to stop the servo
            else if(gamepad1.b) {
                tauraServo.setPower(0);
            }

            //encoder telemetry that will appear on the driver hub
            telemetry.addData("Pos lida", "%.3f", tauraServo.getUniversalPosition());
            telemetry.addData("Pos graus absoluto", "%.3f", tauraServo.getRawPositionInDegrees());
            telemetry.addData("Pos graus incremental", "%.3f", tauraServo.getIncrementalPositionInDegrees());
            telemetry.update();
        }
    }

}