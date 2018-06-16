package net.kno3.season.relicrecovery.nerva.program.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.kno3.robot.Robot;
import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.robot.NervaDriveSystem;
import net.kno3.util.ValuesAdjuster;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "calibration", name = "Calibrate TurnPIDSlow")
public class CalibrateTurnPIDslow extends LinearOpMode {
    public double kTurnPIDslow_maxTurn, kTurnPIDslow_kp, kTurnPIDslow_ki, kTurnPIDslow_kd, kTurnPIDslow_tolerance;
    public double testAngle = 90;

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

        kTurnPIDslow_maxTurn = settings.getDouble("kTurnPIDslow_maxTurn");
        kTurnPIDslow_kp = settings.getDouble("kTurnPIDslow_kp");
        kTurnPIDslow_ki = settings.getDouble("kTurnPIDslow_ki");
        kTurnPIDslow_kd = settings.getDouble("kTurnPIDslow_kd");
        kTurnPIDslow_tolerance = settings.getDouble("kTurnPIDslow_tolerance");

        ValuesAdjuster adjuster = new ValuesAdjuster(this, telemetry);
        adjuster.addValue("kTurnPIDslow_maxTurn", "Max Turn Speed", 0, 10000);
        adjuster.addValue("kTurnPIDslow_kp", "KP", 0, 10000);
        adjuster.addValue("kTurnPIDslow_ki", "KI", 0, 10000);
        adjuster.addValue("kTurnPIDslow_kd", "KD", 0, 10000);
        adjuster.addValue("kTurnPIDslow_tolerance", "Tolerance", 0, 10000);
        adjuster.addValue("testAngle", "Testing Angle", 0, 360);

        while (opModeIsActive()) {
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addData("2", "Press A to test");
                telemetry.addData("3", "Press Start and back to change the increment");
                telemetry.addData("4", "Press dpad left and right to changed adjusted value");
                telemetry.addData("5", "Press the bumpers to adjust the value");
                adjuster.update(gamepad1);
                telemetry.update();
            }
            drive.kTurnPIDslow_maxTurn = kTurnPIDslow_maxTurn;
            drive.kTurnPIDslow_kp = kTurnPIDslow_kp;
            drive.kTurnPIDslow_ki = kTurnPIDslow_ki;
            drive.kTurnPIDslow_kd = kTurnPIDslow_kd;
            drive.kTurnPIDslow_tolerance = kTurnPIDslow_tolerance;

            Thread.sleep(1000);

            drive.turnPIDslow(testAngle);
            drive.stop();

            while (opModeIsActive()) {
                telemetry.addData("6", "Press Y to save and exit");
                telemetry.addData("7", "Press B to try again");
                telemetry.update();
                if(gamepad1.y) {
                    settings.setDouble("kTurnPIDslow_maxTurn", kTurnPIDslow_maxTurn);
                    settings.setDouble("kTurnPIDslow_kp", kTurnPIDslow_kp);
                    settings.setDouble("kTurnPIDslow_ki", kTurnPIDslow_ki);
                    settings.setDouble("kTurnPIDslow_kd", kTurnPIDslow_kd);
                    settings.setDouble("kTurnPIDslow_tolerance", kTurnPIDslow_tolerance);
                    settings.save();
                    return;
                }
                if(gamepad1.b) {
                    break;
                }
            }
        }
    }
}
