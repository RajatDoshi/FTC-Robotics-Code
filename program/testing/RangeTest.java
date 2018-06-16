package net.kno3.season.relicrecovery.nerva.program.testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "testing", name = "RangeTest")
public class RangeTest extends LinearOpMode {

    ModernRoboticsI2cRangeSensor range, range2;
    @Override
    public void runOpMode() throws InterruptedException {

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        range2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("distance", range.getDistance(DistanceUnit.INCH));
            telemetry.addData("distance2", range2.getDistance(DistanceUnit.INCH));
            telemetry.addData("ultra1", range.cmUltrasonic()*0.393701);
            telemetry.addData("ultra2", range2.cmUltrasonic()*0.393701);
            telemetry.addData("optic1", range.cmOptical()*0.393701);
            telemetry.addData("optic2", range2.cmOptical()*0.393701);
            telemetry.update();
        }
    }
}