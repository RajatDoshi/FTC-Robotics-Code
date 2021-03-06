package net.kno3.season.relicrecovery.nerva.program.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.program.calibration.ServoDefaults;
import net.kno3.season.relicrecovery.nerva.program.calibration.TestingNervaDT;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "testing", name = "TestDT2")
public class TestDT2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.dcMotor.get(Nerva.DRIVE_FL_KEY);
        DcMotor fr = hardwareMap.dcMotor.get(Nerva.DRIVE_FR_KEY);
        DcMotor rl = hardwareMap.dcMotor.get(Nerva.DRIVE_RL_KEY);
        DcMotor rr = hardwareMap.dcMotor.get(Nerva.DRIVE_RR_KEY);
        waitForStart();
        fl.setPower(1);
        fr.setPower(1);
        rl.setPower(1);
        rr.setPower(1);
    }
}
