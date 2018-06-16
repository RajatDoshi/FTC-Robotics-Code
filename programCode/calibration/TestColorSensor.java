package net.kno3.season.relicrecovery.nerva.program.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by robotics on 11/9/2017.
 */

@TeleOp(group = "calibration", name = "test color sensor")
public class TestColorSensor extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor color = hardwareMap.colorSensor.get("Left Jewel Color");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }

    }
}
