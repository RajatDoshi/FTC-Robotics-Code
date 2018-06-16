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
@TeleOp(group = "calibration", name = "Calibrate Drive With Corrections X")
public class CalibrateDriveWithCorrectionsX extends LinearOpMode {
    public double kDriveWithCorrectionsSlowX_maxSpeed,
            kDriveWithCorrectionsSlowX_p,
            kDriveWithCorrectionsSlowX_i,
            kDriveWithCorrectionsSlowX_d,
            kDriveWithCorrectionsSlowX_maxTurn,
            kDriveWithCorrectionsSlowX_kp,
            kDriveWithCorrectionsSlowX_ki,
            kDriveWithCorrectionsSlowX_kd,
            kDriveWithCorrectionsSlowX_tolerance;
    public double testDistance = 24;

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

        kDriveWithCorrectionsSlowX_maxSpeed = settings.getDouble("kDriveWithCorrectionsSlowX_maxSpeed");
        kDriveWithCorrectionsSlowX_p = settings.getDouble("kDriveWithCorrectionsSlowX_p");
        kDriveWithCorrectionsSlowX_i = settings.getDouble("kDriveWithCorrectionsSlowX_i");
        kDriveWithCorrectionsSlowX_d = settings.getDouble("kDriveWithCorrectionsSlowX_d");
        kDriveWithCorrectionsSlowX_maxTurn = settings.getDouble("kDriveWithCorrectionsSlowX_maxTurn");
        kDriveWithCorrectionsSlowX_kp = settings.getDouble("kDriveWithCorrectionsSlowX_kp");
        kDriveWithCorrectionsSlowX_ki = settings.getDouble("kDriveWithCorrectionsSlowX_ki");
        kDriveWithCorrectionsSlowX_kd = settings.getDouble("kDriveWithCorrectionsSlowX_kd");
        kDriveWithCorrectionsSlowX_tolerance = settings.getDouble("kDriveWithCorrectionsSlowX_tolerance");

        ValuesAdjuster adjuster = new ValuesAdjuster(this, telemetry);
        adjuster.addValue("kDriveWithCorrectionsSlowX_maxSpeed", "Max Speed", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_p", "Drive P", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_i", "Drive I", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_d", "Drive D", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_maxTurn", "Max Turn", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_kp", "turn P", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_ki", "turn I", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_kd", "turn D", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlowX_tolerance", "turn tolerance", 0, 10000);
        adjuster.addValue("testDistance", "Testing Distance (inches)", -10000, 10000);

        while (opModeIsActive()) {
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addData("2", "Press A to test");
                telemetry.addData("3", "Press Start and back to change the increment");
                telemetry.addData("4", "Press dpad left and right to changed adjusted value");
                telemetry.addData("5", "Press the bumpers to adjust the value");
                adjuster.update(gamepad1);
                telemetry.update();
            }
            drive.kDriveWithCorrectionsSlowX_maxSpeed = kDriveWithCorrectionsSlowX_maxSpeed;
            drive.kDriveWithCorrectionsSlowX_p = kDriveWithCorrectionsSlowX_p;
            drive.kDriveWithCorrectionsSlowX_i = kDriveWithCorrectionsSlowX_i;
            drive.kDriveWithCorrectionsSlowX_d = kDriveWithCorrectionsSlowX_d;
            drive.kDriveWithCorrectionsSlowX_maxTurn = kDriveWithCorrectionsSlowX_maxTurn;
            drive.kDriveWithCorrectionsSlowX_kp = kDriveWithCorrectionsSlowX_kp;
            drive.kDriveWithCorrectionsSlowX_ki = kDriveWithCorrectionsSlowX_ki;
            drive.kDriveWithCorrectionsSlowX_kd = kDriveWithCorrectionsSlowX_kd;
            drive.kDriveWithCorrectionsSlowX_tolerance = kDriveWithCorrectionsSlowX_tolerance;

            Thread.sleep(1000);

            drive.driveWithCorrectionSlowX(testDistance, 0);
            drive.stop();

            while (opModeIsActive()) {
                telemetry.addData("6", "Press Y to save and exit");
                telemetry.addData("7", "Press B to try again");
                telemetry.update();
                if(gamepad1.y) {
                    settings.setDouble("kDriveWithCorrectionsSlowX_maxSpeed", kDriveWithCorrectionsSlowX_maxSpeed);
                    settings.setDouble("kDriveWithCorrectionsSlowX_p", kDriveWithCorrectionsSlowX_p);
                    settings.setDouble("kDriveWithCorrectionsSlowX_i", kDriveWithCorrectionsSlowX_i);
                    settings.setDouble("kDriveWithCorrectionsSlowX_d", kDriveWithCorrectionsSlowX_d);
                    settings.setDouble("kDriveWithCorrectionsSlowX_maxTurn", kDriveWithCorrectionsSlowX_maxTurn);
                    settings.setDouble("kDriveWithCorrectionsSlowX_kp", kDriveWithCorrectionsSlowX_kp);
                    settings.setDouble("kDriveWithCorrectionsSlowX_ki", kDriveWithCorrectionsSlowX_ki);
                    settings.setDouble("kDriveWithCorrectionsSlowX_kd", kDriveWithCorrectionsSlowX_kd);
                    settings.setDouble("kDriveWithCorrectionsSlowX_tolerance", kDriveWithCorrectionsSlowX_tolerance);
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
