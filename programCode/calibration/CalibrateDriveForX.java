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
@TeleOp(group = "calibration", name = "Calibrate DriveForX")
public class CalibrateDriveForX extends LinearOpMode {
    public double kDriveForXSlow_maxSpeed, kDriveForXSlow_kp, kDriveForXSlow_ki, kDriveForXSlow_kd, kDriveForXSlow_tolerance;
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

        kDriveForXSlow_maxSpeed = settings.getDouble("kDriveForXSlow_maxSpeed");
        kDriveForXSlow_kp = settings.getDouble("kDriveForXSlow_kp");
        kDriveForXSlow_ki = settings.getDouble("kDriveForXSlow_ki");
        kDriveForXSlow_kd = settings.getDouble("kDriveForXSlow_kd");
        kDriveForXSlow_tolerance = settings.getDouble("kDriveForXSlow_tolerance");

        ValuesAdjuster adjuster = new ValuesAdjuster(this, telemetry);
        adjuster.addValue("kDriveForXSlow_maxSpeed", "Max Speed", 0, 10000);
        adjuster.addValue("kDriveForXSlow_kp", "P", 0, 10000);
        adjuster.addValue("kDriveForXSlow_ki", "I", 0, 10000);
        adjuster.addValue("kDriveForXSlow_kd", "D", 0, 10000);
        adjuster.addValue("kDriveForXSlow_tolerance", "Tolerance", 0, 10000);
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
            drive.kDriveForXSlow_maxSpeed = kDriveForXSlow_maxSpeed;
            drive.kDriveForXSlow_kp = kDriveForXSlow_kp;
            drive.kDriveForXSlow_ki = kDriveForXSlow_ki;
            drive.kDriveForXSlow_kd = kDriveForXSlow_kd;
            drive.kDriveForXSlow_tolerance = kDriveForXSlow_tolerance;

            Thread.sleep(1000);

            drive.driveForX(testDistance);
            drive.stop();

            while (opModeIsActive()) {
                telemetry.addData("6", "Press Y to save and exit");
                telemetry.addData("7", "Press B to try again");
                telemetry.update();
                if(gamepad1.y) {
                    settings.setDouble("kDriveForXSlow_maxSpeed", kDriveForXSlow_maxSpeed);
                    settings.setDouble("kDriveForXSlow_kp", kDriveForXSlow_kp);
                    settings.setDouble("kDriveForXSlow_ki", kDriveForXSlow_ki);
                    settings.setDouble("kDriveForXSlow_kd", kDriveForXSlow_kd);
                    settings.setDouble("kDriveForXSlow_tolerance", kDriveForXSlow_tolerance);
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
