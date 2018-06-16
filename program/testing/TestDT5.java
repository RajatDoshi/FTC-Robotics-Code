package net.kno3.season.relicrecovery.nerva.program.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import net.kno3.season.relicrecovery.nerva.robot.Nerva;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "testing", name = "TestDT5")
public class TestDT5 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.dcMotor.get(Nerva.DRIVE_FL_KEY);
        DcMotor fr = hardwareMap.dcMotor.get(Nerva.DRIVE_FR_KEY);
        DcMotor rl = hardwareMap.dcMotor.get(Nerva.DRIVE_RL_KEY);
        DcMotor rr = hardwareMap.dcMotor.get(Nerva.DRIVE_RR_KEY);
        waitForStart();
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setPower(1);
        fr.setPower(1);
        rl.setPower(1);
        rr.setPower(1);

        while (opModeIsActive()) {
            telemetry.addData("FLP", fl.getCurrentPosition());
            telemetry.addData("FRP", fr.getCurrentPosition());
            telemetry.addData("RLP", rl.getCurrentPosition());
            telemetry.addData("RRP", rr.getCurrentPosition());
            telemetry.update();
        }
    }
}
