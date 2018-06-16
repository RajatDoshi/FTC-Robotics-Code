package net.kno3.season.relicrecovery.nerva.program.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.program.calibration.ServoDefaults;
import net.kno3.season.relicrecovery.nerva.program.calibration.TestingNervaDT;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "testing", name = "TestDT")
public class TestDT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcOpModeRegister.opModeManager = (OpModeManagerImpl) internalOpModeServices;

        TestingNervaDT drive = new TestingNervaDT(hardwareMap, telemetry);
        drive.modeSpeed();
        telemetry.addData("1", "Press start when imu is ready");
        telemetry.update();
        waitForStart();

        RobotSettings settings = new RobotSettings("Nerva");
        ServoDefaults.resetAllServos(hardwareMap, settings);


        drive.driveRobotOriented(0, 1, 0);
        Thread.sleep(1000);
        drive.stop();
    }
}
