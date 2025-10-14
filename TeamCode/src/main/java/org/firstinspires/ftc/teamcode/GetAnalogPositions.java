package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name="Get Analog Positions", group="Linear OpMode")
public class GetAnalogPositions extends LinearOpMode {

    private static final String FILE_NAME = "analogFeedback.csv"; // pode usar .txt se preferir

    @Override
    public void runOpMode() throws InterruptedException {
        TauraServo tauraServo = hardwareMap.get(TauraServo.class, "servoTeste");
        AnalogInput potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        // Arquivo na pasta de settings do FTC: /sdcard/FIRST/settings/analogFeedback.csv
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
        // Garante que a pasta existe
        if (file.getParentFile() != null && !file.getParentFile().exists()) {
            file.getParentFile().mkdirs();
        }

        telemetry.addLine("Pronto. Arquivo: " + file.getAbsolutePath());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file, true))) {
            if (file.length() == 0) {
                writer.write("timestamp_ms,pose_comandada,pose_atual,voltagem_V\n");
                writer.flush();
            }

            ElapsedTime timer = new ElapsedTime();

            for (double pose = 0.0; opModeIsActive() && pose <= 1.0001; pose += 0.05) {
                tauraServo.setPosition(pose);
                sleep(300);

                double voltage = potentiometer.getVoltage();
                double actual  = tauraServo.getPosition();
                long ts        = System.currentTimeMillis();

                String line = String.format(Locale.US, "%d,%.3f,%.3f,%.5f\n", ts, pose, actual, voltage);
                writer.write(line);
                writer.flush();

                telemetry.addData("Pose cmd", "%.3f", pose);
                telemetry.addData("Pose atual", "%.3f", actual);
                telemetry.addData("V (pot)", "%.5f V", voltage);
                telemetry.addData("Arquivo", file.getName());
                telemetry.update();
            }

            for (double pose = 1.0; opModeIsActive() && pose >= -0.0001; pose -= 0.05) {
                tauraServo.setPosition(pose);
                sleep(300);

                double voltage = potentiometer.getVoltage();
                double actual  = tauraServo.getPosition();
                long ts        = System.currentTimeMillis();

                String line = String.format(Locale.US, "%d,%.3f,%.3f,%.5f\n", ts, pose, actual, voltage);
                writer.write(line);
                writer.flush();

                telemetry.addData("Pose cmd", "%.3f", pose);
                telemetry.addData("Pose atual", "%.3f", actual);
                telemetry.addData("V (pot)", "%.5f V", voltage);
                telemetry.addData("Arquivo", file.getName());
                telemetry.update();
            }

            telemetry.addLine("Coleta concluída e salva com sucesso.");
            telemetry.update();

        } catch (IOException e) {
            telemetry.addData("Erro ao escrever arquivo", e.getMessage());
            telemetry.update();
        }
    }
}
