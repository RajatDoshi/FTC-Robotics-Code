package net.kno3.season.relicrecovery.nerva.program.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;
import net.kno3.util.Threading;

/**
 * @author Jaxon A Brown
 */
@TeleOp(group = "calibration", name = "Calibrate Intake Arm Left")
public class CalibrateIntakeArmLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("1", "Press start");
        telemetry.update();
        waitForStart();

        RobotSettings settings = new RobotSettings("Nerva");

        ServoDefaults.resetAllServos(hardwareMap, settings);

        while(opModeIsActive()) {
            Servo leftIntakeArm = hardwareMap.servo.get(Nerva.INTAKE_LEFT_ARM_KEY);
            double initSetpoint = settings.getDouble("intake_arm_left_init");
            while(opModeIsActive() && !gamepad1.a) {
                if(Math.abs(gamepad1.left_stick_x) > 0.05) {
                    initSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if(Math.abs(gamepad1.right_stick_x) > 0.05) {
                    initSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                leftIntakeArm.setPosition(initSetpoint);
                telemetry.addData("Init Setpoint: ", initSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            double outSetpoint = settings.getDouble("intake_arm_left_out");
            while(opModeIsActive() && !gamepad1.a) {
                if(Math.abs(gamepad1.left_stick_x) > 0.05) {
                    outSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if(Math.abs(gamepad1.right_stick_x) > 0.05) {
                    outSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                leftIntakeArm.setPosition(outSetpoint);
                telemetry.addData("Out Setpoint: ", outSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            double backSetpoint = settings.getDouble("intake_arm_left_back");
            while(opModeIsActive() && !gamepad1.a) {
                if(Math.abs(gamepad1.left_stick_x) > 0.05) {
                    backSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if(Math.abs(gamepad1.right_stick_x) > 0.05) {
                    backSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                leftIntakeArm.setPosition(backSetpoint);
                telemetry.addData("Back Setpoint: ", backSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            double senseSetpoint = settings.getDouble("intake_arm_left_sense");
            while(opModeIsActive() && !gamepad1.a) {
                if(Math.abs(gamepad1.left_stick_x) > 0.05) {
                    senseSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if(Math.abs(gamepad1.right_stick_x) > 0.05) {
                    senseSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                leftIntakeArm.setPosition(senseSetpoint);
                telemetry.addData("Sense Setpoint: ", senseSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            telemetry.addData("3", "press B to test");
            telemetry.update();
            while(opModeIsActive() && !gamepad1.b);

            leftIntakeArm.setPosition(initSetpoint);
            Threading.delay(4);
            leftIntakeArm.setPosition(backSetpoint);
            Threading.delay(4);
            leftIntakeArm.setPosition(outSetpoint);
            Threading.delay(4);

            settings.setDouble("intake_arm_left_out", outSetpoint);
            settings.setDouble("intake_arm_left_init", initSetpoint);
            settings.setDouble("intake_arm_left_back", backSetpoint);
            settings.setDouble("intake_arm_left_sense", senseSetpoint);
            settings.save();

            telemetry.addData("5", "Saved! Press Y to try again.");
            telemetry.update();
            while(opModeIsActive() && !gamepad1.y);
        }
    }
}