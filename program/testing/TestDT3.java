package net.kno3.season.relicrecovery.nerva.program.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import net.kno3.robot.Robot;
import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.program.calibration.ServoDefaults;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;
import net.kno3.season.relicrecovery.nerva.robot.NervaDriveSystem;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "testing", name = "TestDT3")
public class TestDT3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcOpModeRegister.opModeManager = (OpModeManagerImpl) internalOpModeServices;

        Robot robot = new Robot(this);
        NervaDriveSystem drive = new NervaDriveSystem(robot);
        drive.init();
        drive.modeSpeed();
        telemetry.addData("1", "Press start when imu is ready");
        telemetry.update();
        waitForStart();

        RobotSettings settings = new RobotSettings("Nerva");
        ServoDefaults.resetAllServos(hardwareMap, settings);

        while (opModeIsActive()) {
            telemetry.addData("FLP", drive.frontLeft.getCurrentPosition());
            telemetry.addData("FRP", drive.frontRight.getCurrentPosition());
            telemetry.addData("RLP", drive.rearLeft.getCurrentPosition());
            telemetry.addData("RRP", drive.rearRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
