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
@TeleOp(group = "calibration", name = "Calibrate Drive With Corrections Y")
public class CalibrateDriveWithCorrectionsY extends LinearOpMode {
    public double kDriveWithCorrectionsSlow_maxSpeed,
            kDriveWithCorrectionsSlow_p,
            kDriveWithCorrectionsSlow_i,
            kDriveWithCorrectionsSlow_d,
            kDriveWithCorrectionsSlow_maxTurn,
            kDriveWithCorrectionsSlow_kp,
            kDriveWithCorrectionsSlow_ki,
            kDriveWithCorrectionsSlow_kd,
            kDriveWithCorrectionsSlow_tolerance;
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

        kDriveWithCorrectionsSlow_maxSpeed = settings.getDouble("kDriveWithCorrectionsSlow_maxSpeed");
        kDriveWithCorrectionsSlow_p = settings.getDouble("kDriveWithCorrectionsSlow_p");
        kDriveWithCorrectionsSlow_i = settings.getDouble("kDriveWithCorrectionsSlow_i");
        kDriveWithCorrectionsSlow_d = settings.getDouble("kDriveWithCorrectionsSlow_d");
        kDriveWithCorrectionsSlow_maxTurn = settings.getDouble("kDriveWithCorrectionsSlow_maxTurn");
        kDriveWithCorrectionsSlow_kp = settings.getDouble("kDriveWithCorrectionsSlow_kp");
        kDriveWithCorrectionsSlow_ki = settings.getDouble("kDriveWithCorrectionsSlow_ki");
        kDriveWithCorrectionsSlow_kd = settings.getDouble("kDriveWithCorrectionsSlow_kd");
        kDriveWithCorrectionsSlow_tolerance = settings.getDouble("kDriveWithCorrectionsSlow_tolerance");

        ValuesAdjuster adjuster = new ValuesAdjuster(this, telemetry);
        adjuster.addValue("kDriveWithCorrectionsSlow_maxSpeed", "Max Speed", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_p", "Drive P", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_i", "Drive I", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_d", "Drive D", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_maxTurn", "Max Turn", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_kp", "turn P", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_ki", "turn I", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_kd", "turn D", 0, 10000);
        adjuster.addValue("kDriveWithCorrectionsSlow_tolerance", "turn tolerance", 0, 10000);
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
            drive.kDriveWithCorrectionsSlow_maxSpeed = kDriveWithCorrectionsSlow_maxSpeed;
            drive.kDriveWithCorrectionsSlow_p = kDriveWithCorrectionsSlow_p;
            drive.kDriveWithCorrectionsSlow_i = kDriveWithCorrectionsSlow_i;
            drive.kDriveWithCorrectionsSlow_d = kDriveWithCorrectionsSlow_d;
            drive.kDriveWithCorrectionsSlow_maxTurn = kDriveWithCorrectionsSlow_maxTurn;
            drive.kDriveWithCorrectionsSlow_kp = kDriveWithCorrectionsSlow_kp;
            drive.kDriveWithCorrectionsSlow_ki = kDriveWithCorrectionsSlow_ki;
            drive.kDriveWithCorrectionsSlow_kd = kDriveWithCorrectionsSlow_kd;
            drive.kDriveWithCorrectionsSlow_tolerance = kDriveWithCorrectionsSlow_tolerance;

            Thread.sleep(1000);

            drive.driveWithCorrectionSlow(testDistance, 0);
            drive.stop();

            while (opModeIsActive()) {
                telemetry.addData("6", "Press Y to save and exit");
                telemetry.addData("7", "Press B to try again");
                telemetry.update();
                if(gamepad1.y) {
                    settings.setDouble("kDriveWithCorrectionsSlow_maxSpeed", kDriveWithCorrectionsSlow_maxSpeed);
                    settings.setDouble("kDriveWithCorrectionsSlow_p", kDriveWithCorrectionsSlow_p);
                    settings.setDouble("kDriveWithCorrectionsSlow_i", kDriveWithCorrectionsSlow_i);
                    settings.setDouble("kDriveWithCorrectionsSlow_d", kDriveWithCorrectionsSlow_d);
                    settings.setDouble("kDriveWithCorrectionsSlow_maxTurn", kDriveWithCorrectionsSlow_maxTurn);
                    settings.setDouble("kDriveWithCorrectionsSlow_kp", kDriveWithCorrectionsSlow_kp);
                    settings.setDouble("kDriveWithCorrectionsSlow_ki", kDriveWithCorrectionsSlow_ki);
                    settings.setDouble("kDriveWithCorrectionsSlow_kd", kDriveWithCorrectionsSlow_kd);
                    settings.setDouble("kDriveWithCorrectionsSlow_tolerance", kDriveWithCorrectionsSlow_tolerance);
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
