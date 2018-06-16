package net.kno3.season.relicrecovery.nerva.program.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import net.kno3.season.relicrecovery.nerva.robot.Nerva;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "testing", name = "EncTest")
public class EncTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.dcMotor.get(Nerva.DRIVE_FL_KEY);
        DcMotor fr = hardwareMap.dcMotor.get(Nerva.DRIVE_FR_KEY);
        DcMotor rl = hardwareMap.dcMotor.get(Nerva.DRIVE_RL_KEY);
        DcMotor rr = hardwareMap.dcMotor.get(Nerva.DRIVE_RR_KEY);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotor encoder_y = hardwareMap.dcMotor.get(Nerva.ENCODER_Y_KEY), encoder_x = hardwareMap.dcMotor.get(Nerva.ENCODER_X_KEY);
        int encZeroX = encoder_x.getCurrentPosition(), encZeroY = encoder_y.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("encX", -(encoder_x.getCurrentPosition() - encZeroX));
            telemetry.addData("encY", -(encoder_y.getCurrentPosition() - encZeroY));
            telemetry.update();
        }

    }
}
