package net.kno3.season.relicrecovery.nerva.program.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import net.kno3.season.relicrecovery.nerva.program.auto.NervaAutoBlue;
import net.kno3.season.relicrecovery.nerva.program.auto.NervaAutoRed;
import net.kno3.season.relicrecovery.nerva.program.auto.VuforiaScanner;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;
import net.kno3.season.relicrecovery.nerva.robot.NervaGlyphSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaIntakeSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaJewelSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaPersist;
import net.kno3.util.AngleUtil;
import net.kno3.util.AutoTransitioner;
import net.kno3.util.SimpleColor;
import net.kno3.util.Threading;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by robotics on 10/29/2017.
 */
@Disabled

@Autonomous(name = "Red Auto Test")
public class BlueAutoTest extends NervaAutoBlue {
    private VuforiaScanner vuforiaScanner;

    private ColorSensor columnSensor;
    @Override
    public void postInit() {
        super.postInit();
        vuforiaScanner = new VuforiaScanner(hardwareMap, telemetry);
        Threading.async(() -> {
            while(!vuforiaScanner.isInitialized()) {
                if(isStopRequested()) {
                    return;
                }
                Thread.yield();
            }
            while(!isStarted() && !isStopRequested()) {
                vuforiaScanner.scan();
                RelicRecoveryVuMark vuMark = vuforiaScanner.getLastValid();
                telemetry.addData("Vuforia Current", vuforiaScanner.getLastValid().name());
                telemetry.update();
                Thread.yield();
            }
        });
        AutoTransitioner.transitionOnStop(this, "Nerva Teleop");

        columnSensor = hardwareMap.colorSensor.get("column color");
        glyphSys.setTopClamp(NervaGlyphSystem.TopClampState.CLOSED);
        glyphSys.setBottomClamp(NervaGlyphSystem.BottomClampState.OPEN);
        intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.INIT);
        intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.INIT);
    }

    @Override
    public void main() {
        //Bring Lift up
        glyphSys.liftEncZero -= 600;
        glyphSys.setLiftPower(-0.3);
        waitFor(1.25);
        glyphSys.setLiftPower(0);
        waitFor(.15);


        //This is the initial code for opening the claw
        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.CENTER);
        waitFor(.15);
        jewelSys.setBlueKnocker(NervaJewelSystem.KnockerPosition.DOWN);
        waitFor(.15);
        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.DOWN);
        waitFor(.15);

        //gets jewel color
        SimpleColor jewelColor = jewelSys.getLeftColor();
        waitFor(.15);

        //knocks jewel based on read jewel color
        if(jewelColor == SimpleColor.BLUE) {
            jewelSys.setBlueKnocker(NervaJewelSystem.KnockerPosition.RIGHT);
        } else if (jewelColor == SimpleColor.RED){
            jewelSys.setBlueKnocker(NervaJewelSystem.KnockerPosition.LEFT);
        }

        //puts arm back up
        waitFor(.15);
        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.CENTER);
        waitFor(.15);
        jewelSys.setBlueKnocker(NervaJewelSystem.KnockerPosition.CENTER);
        waitFor(.15);
        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.UP);
        waitFor(.15);

        //drives forwards off ramp
        drive.modeVoltage();
        drive.driveRobotOriented(0, 0.25, 0);
        waitFor(1.2);
        drive.stop();
        waitFor(0.15);

        //Open Intake
        intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.BACK);
        waitFor(.25);
        intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.BACK);
        waitFor(.25);

        //Bring Lift Down
        glyphSys.setLiftPower(0.3);
        waitFor(2.25);
        glyphSys.setLiftPower(0);
        waitFor(.15);

        //drive a bit to the left
        drive.modeVoltage();
        drive.driveForX(-4);
        drive.stop();
        waitFor(.15);

        //drives into cryptobox wall
        drive.modeVoltage();
        drive.driveRobotOriented(0, .2, 0);
        waitFor(1.6);
        drive.stop();
        waitFor(0.15);

        drive.modeSpeed();
        drive.turnPIDslow(0);
        drive.stop();
        waitFor(.15);

        //drive against wall
        drive.modeVoltage();
        drive.driveForXTime(10, 4);
        drive.stop();

        drive.modeSpeed();
        drive.turnPIDslow(0);
        drive.stop();
        waitFor(.15);

        drive.modeVoltage();
        drive.driveForY(-6);
        drive.stop();
        waitFor(0.15);
        //drives to column based on read pictograph
        drive.modeVoltage();
        RelicRecoveryVuMark vuMark = vuforiaScanner.getLastValid();

        //drives to column based on read pictograph
        switch(vuMark) {
            case LEFT:
                drive.driveForX(7.5);
                drive.stop();
                break;
            case CENTER:
                drive.driveForX(15);
                drive.stop();
                break;
            case RIGHT:
                drive.driveForX(22.5);
                drive.stop();
                break;
            default:
                drive.driveForX(15);
                drive.stop();
                break;
        }

        //sets heading back to initial heading
        drive.modeSpeed();
        drive.turnPIDslow(0);
        drive.stop();
        waitFor(0.15);

        //drives forward into cryptobox
        drive.modeVoltage();
        drive.driveRobotOriented(0, 0.25, 0);
        waitFor(.8);
        drive.stop();
        waitFor(0.15);

        //open glyph clamp
        glyphSys.setTopClamp(NervaGlyphSystem.TopClampState.OPEN);
        waitFor(.25);

        //back up a little bit
        drive.driveForY(-4);
        drive.stop();
        waitFor(0.25);

        //Close Intake
        intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.OUT);
        waitFor(.15);
        intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.OUT);
        waitFor(.15);


        //transition to teleop
        NervaPersist.lastWasAuto = true;
        NervaPersist.lastAngle = AngleUtil.normalize(drive.getHeading() - 90);

    }

}