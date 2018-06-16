package net.kno3.season.relicrecovery.nerva.program.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.kno3.robot.Robot;
import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.robot.NervaDriveSystem;
import net.kno3.util.SynchronousPID;
import net.kno3.util.Threading;
import net.kno3.util.ValuesAdjuster;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

/**
 * Created by robotics on 10/29/2017.
 */
@TeleOp(group = "calibration", name = "Calibrate DriveForY")
public class CalibrateDriveForY extends LinearOpMode {
    public double kDriveForYSlow_maxSpeed, kDriveForYSlow_kp, kDriveForYSlow_ki, kDriveForYSlow_kd, kDriveForYSlow_tolerance;
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

        kDriveForYSlow_maxSpeed = settings.getDouble("kDriveForYSlow_maxSpeed");
        kDriveForYSlow_kp = settings.getDouble("kDriveForYSlow_kp");
        kDriveForYSlow_ki = settings.getDouble("kDriveForYSlow_ki");
        kDriveForYSlow_kd = settings.getDouble("kDriveForYSlow_kd");
        kDriveForYSlow_tolerance = settings.getDouble("kDriveForYSlow_tolerance");

        ValuesAdjuster adjuster = new ValuesAdjuster(this, telemetry);
        adjuster.addValue("kDriveForYSlow_maxSpeed", "Max Speed", 0, 10000);
        adjuster.addValue("kDriveForYSlow_kp", "P", 0, 10000);
        adjuster.addValue("kDriveForYSlow_ki", "I", 0, 10000);
        adjuster.addValue("kDriveForYSlow_kd", "D", 0, 10000);
        adjuster.addValue("kDriveForYSlow_tolerance", "Tolerance", 0, 10000);
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
            drive.kDriveForYSlow_maxSpeed = kDriveForYSlow_maxSpeed;
            drive.kDriveForYSlow_kp = kDriveForYSlow_kp;
            drive.kDriveForYSlow_ki = kDriveForYSlow_ki;
            drive.kDriveForYSlow_kd = kDriveForYSlow_kd;
            drive.kDriveForYSlow_tolerance = kDriveForYSlow_tolerance;

            Thread.sleep(1000);

            drive.driveForY(testDistance);
            drive.stop();

            while (opModeIsActive()) {
                telemetry.addData("6", "Press Y to save and exit");
                telemetry.addData("7", "Press B to try again");
                telemetry.update();
                if(gamepad1.y) {
                    settings.setDouble("kDriveForYSlow_maxSpeed", kDriveForYSlow_maxSpeed);
                    settings.setDouble("kDriveForYSlow_kp", kDriveForYSlow_kp);
                    settings.setDouble("kDriveForYSlow_ki", kDriveForYSlow_ki);
                    settings.setDouble("kDriveForYSlow_kd", kDriveForYSlow_kd);
                    settings.setDouble("kDriveForYSlow_tolerance", kDriveForYSlow_tolerance);
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
