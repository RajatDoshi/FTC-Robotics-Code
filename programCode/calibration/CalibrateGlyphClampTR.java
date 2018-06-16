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
@TeleOp(group = "calibration", name = "Calibrate Glyph Clamp TR")
public class CalibrateGlyphClampTR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("1", "Press start");
        telemetry.update();
        waitForStart();

        RobotSettings settings = new RobotSettings("Nerva");

        ServoDefaults.resetAllServos(hardwareMap, settings);

        while(opModeIsActive()) {
            Servo topRightGlyphClamp = hardwareMap.servo.get(Nerva.GLYPH_CLAMP_TOP_RIGHT_KEY);
            double idleSetpoint = settings.getDouble("glyph_clamp_top_right_idle");
            while(opModeIsActive() && !gamepad1.a) {
                if(Math.abs(gamepad1.left_stick_x) > 0.05) {
                    idleSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if(Math.abs(gamepad1.right_stick_x) > 0.05) {
                    idleSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                topRightGlyphClamp.setPosition(idleSetpoint);
                telemetry.addData("Idle Setpoint: ", idleSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            double openSetpoint = settings.getDouble("glyph_clamp_top_right_open");
            while(opModeIsActive() && !gamepad1.a) {
                if(Math.abs(gamepad1.left_stick_x) > 0.05) {
                    openSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if(Math.abs(gamepad1.right_stick_x) > 0.05) {
                    openSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                topRightGlyphClamp.setPosition(openSetpoint);
                telemetry.addData("Open Setpoint: ", openSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            double clampedSetpoint = settings.getDouble("glyph_clamp_top_right_closed");
            while(opModeIsActive() && !gamepad1.a) {
                if(Math.abs(gamepad1.left_stick_x) > 0.05) {
                    clampedSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if(Math.abs(gamepad1.right_stick_x) > 0.05) {
                    clampedSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                topRightGlyphClamp.setPosition(clampedSetpoint);
                telemetry.addData("Clamped Setpoint: ", clampedSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            double halfSetpoint = settings.getDouble("glyph_clamp_top_right_half");
            while(opModeIsActive() && !gamepad1.a) {
                if (Math.abs(gamepad1.left_stick_x) > 0.05) {
                    halfSetpoint += gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) / 3000;
                }
                if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                    halfSetpoint += gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) / 15000;
                }
                topRightGlyphClamp.setPosition(halfSetpoint);
                telemetry.addData("Half Clamped Setpoint: ", halfSetpoint);
                telemetry.addData("2", "Press A to continue");
                telemetry.update();
            }

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            telemetry.addData("3", "press B to test");
            telemetry.update();
            while(opModeIsActive() && !gamepad1.b);

            topRightGlyphClamp.setPosition(idleSetpoint);
            Threading.delay(4);
            topRightGlyphClamp.setPosition(openSetpoint);
            Threading.delay(4);
            topRightGlyphClamp.setPosition(clampedSetpoint);
            Threading.delay(4);
            topRightGlyphClamp.setPosition(halfSetpoint);
            Threading.delay(4);

            settings.setDouble("glyph_clamp_top_right_idle", idleSetpoint);
            settings.setDouble("glyph_clamp_top_right_closed", clampedSetpoint);
            settings.setDouble("glyph_clamp_top_right_open", openSetpoint);
            settings.setDouble("glyph_clamp_top_right_half", halfSetpoint);
            settings.save();

            telemetry.addData("5", "Saved! Press Y to try again.");
            telemetry.update();
            while(opModeIsActive() && !gamepad1.y);
        }
    }
}