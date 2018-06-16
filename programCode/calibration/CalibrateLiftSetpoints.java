package net.kno3.season.relicrecovery.nerva.program.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;
import net.kno3.util.Threading;

/**
 * @author Jaxon A Brown
 */
@TeleOp(group = "calibration", name = "Calibrate Lift Setpoints")
public class CalibrateLiftSetpoints extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("1", "Press start");
        telemetry.update();

        waitForStart();

        //renamed from "lift"
        DcMotor lift = hardwareMap.dcMotor.get("glyft motor");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotSettings settings = new RobotSettings("Nerva");

        while(opModeIsActive()) {
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            int pos0, pos1, pos11, pos2, pos3;

            int liftRst = lift.getCurrentPosition();
            while (opModeIsActive() && !gamepad1.a) {
                lift.setPower(gamepad1.right_stick_y);
                telemetry.addData("1", "Press A to set Bottom Position");
                telemetry.addData("Pos", lift.getCurrentPosition() - liftRst);
                telemetry.update();
            }
            lift.setPower(0);

            pos0 = lift.getCurrentPosition() - liftRst;

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            while (opModeIsActive() && !gamepad1.a) {
                lift.setPower(-gamepad1.right_stick_y);
                telemetry.addData("3", "Press A to set Position Up 1");
                telemetry.addData("Pos", lift.getCurrentPosition() - liftRst);
                telemetry.update();
            }
            lift.setPower(0);

            pos1 = lift.getCurrentPosition() - liftRst;

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            while (opModeIsActive() && !gamepad1.a) {
                lift.setPower(-gamepad1.right_stick_y);
                telemetry.addData("3", "Press A to set Position Up 1.1");
                telemetry.addData("Pos", lift.getCurrentPosition() - liftRst);
                telemetry.update();
            }
            lift.setPower(0);

            pos11 = lift.getCurrentPosition() - liftRst;

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            while (opModeIsActive() && !gamepad1.a) {
                lift.setPower(-gamepad1.right_stick_y);
                telemetry.addData("3", "Press A to set Position Up 2");
                telemetry.addData("Pos", lift.getCurrentPosition() - liftRst);
                telemetry.update();
            }
            lift.setPower(0);

            pos2 = lift.getCurrentPosition() - liftRst;

            telemetry.addData("2", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            while (opModeIsActive() && !gamepad1.a) {
                lift.setPower(-gamepad1.right_stick_y);
                telemetry.addData("3", "Press A to set Position Up 3");
                telemetry.addData("Pos", lift.getCurrentPosition() - liftRst);
                telemetry.update();
            }
            lift.setPower(0);

            pos3 = lift.getCurrentPosition() - liftRst;

            telemetry.addData("4", "Release A");
            telemetry.update();
            while(opModeIsActive() && gamepad1.a);

            telemetry.addData("5", "Lower the lift to starting position.");
            telemetry.addData("Pos0", pos0);
            telemetry.addData("Pos1", pos1);
            telemetry.addData("Pos1.1", pos11);
            telemetry.addData("Pos2", pos2);
            telemetry.addData("Pos3", pos3);
            settings.setInt("glyph_lift_setpoint_bottom", pos0);
            settings.setInt("glyph_lift_setpoint_1", pos1);
            settings.setInt("glyph_lift_setpoint_1_1", pos11);
            settings.setInt("glyph_lift_setpoint_2", pos2);
            settings.setInt("glyph_lift_setpoint_3", pos3);
            settings.save();
            telemetry.addData("6", "Saved! Press Y to try again!");
            telemetry.update();
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            while(opModeIsActive() && !gamepad1.y);
        }
    }
}