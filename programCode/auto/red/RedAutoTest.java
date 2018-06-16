package net.kno3.season.relicrecovery.nerva.program.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

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
import net.kno3.util.TimeStopParameter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Created by robotics on 10/29/2017.
 */


//testing 2 glyph auto
@Autonomous(name = "Red Audience 2 glyph Test")
public class RedAutoTest extends NervaAutoRed {
    private VuforiaScanner vuforiaScanner;

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

        glyphSys.setTopClamp(NervaGlyphSystem.TopClampState.CLOSED);
        glyphSys.setBottomClamp(NervaGlyphSystem.BottomClampState.OPEN);
        intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.INIT);
        intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.INIT);

    }

    @Override
    public void main() {
        drive.modeSpeed();

        //Bring Lift up
        glyphSys.liftEncZero -= 600;
        glyphSys.setLiftPower(-0.5);

        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.CENTER);
        waitFor(.1);
        jewelSys.setRedKnocker(NervaJewelSystem.KnockerPosition.DOWN);
        waitFor(.1);
        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.DOWN);
        waitFor(.15);

        //gets jewel color
        SimpleColor jewelColor = jewelSys.getLeftColor();
        waitFor(.15);

        //knocks jewel based on read jewel color
        if(jewelColor == SimpleColor.BLUE) {
            jewelSys.setRedKnocker(NervaJewelSystem.KnockerPosition.RIGHT);
        } else if (jewelColor == SimpleColor.RED){
            jewelSys.setRedKnocker(NervaJewelSystem.KnockerPosition.LEFT);
        }

        //puts arm back up
        waitFor(.1);
        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.CENTER);
        waitFor(.1);
        jewelSys.setRedKnocker(NervaJewelSystem.KnockerPosition.CENTER);
        waitFor(.1);
        jewelSys.setLeftArm(NervaJewelSystem.ArmPosition.UP);
        waitFor(.1);


        glyphSys.setLiftPower(0);

        //drives to column based on read pictograph
        RelicRecoveryVuMark vuMark = vuforiaScanner.getLastValid();


        glyphSys.setLiftPower(0.6);
        intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.BACK);
        intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.BACK);

        switch(vuMark) {
            case LEFT:
                drive.driveWithCorrectionSlow(-42, 0, 0.05, new TimeStopParameter(1.8));
                drive.stop();
                break;
            case CENTER:
                drive.driveWithCorrectionSlow(-34.5, 0, 0.05, new TimeStopParameter(1.8));
                drive.stop();
                break;
            case RIGHT:
                drive.driveWithCorrectionSlow(-27.5, 0, 0.05, new TimeStopParameter(1.8));
                drive.stop();
                break;
            default:
                drive.driveWithCorrectionSlow(-34.5, 0, 0.05, new TimeStopParameter(1.8));
                drive.stop();
                break;
        }

        //Bring Lift Down
        glyphSys.setLiftPower(0);
        drive.stop();

        //Turn 270
        drive.turnPIDslow(270);
        drive.stop();

        //drives forward into cryptobox
        //drive.driveRobotOriented(0, 0.6, 0);
        //waitFor(1.5);
        drive.driveWithCorrectionSlow(17, 270, 0.05, new TimeStopParameter(1.2));
        drive.stop();

        //open glyph clamp
        glyphSys.setTopClamp(NervaGlyphSystem.TopClampState.OPEN);
        //back up a little bit
        drive.driveForY(-3.5);
        drive.stop();

        //Close Intake
        intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.OUT);
        intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.OUT);
        intakeSys.spitGlyph();

        waitFor(1);

        drive.driveForY(-8);
        drive.stop();
        intakeSys.stop();

        drive.turnPIDslow(90);
        drive.stop();

        intakeSys.intakeGlyph();
        drive.driveWithCorrectionSlow(30, 90, 0.05, () -> glyphSys.hasGlyph());
        if(glyphSys.hasGlyph()) {

            waitFor(.2);
            drive.driveWithCorrectionSlow(-20, 90, 0.05, new TimeStopParameter(2));
            drive.stop();
            intakeSys.stop();
            glyphSys.setTopClamp(NervaGlyphSystem.TopClampState.CLOSED);
            glyphSys.setHolderState(NervaGlyphSystem.GlyphLiftHolderState.OPEN);
            glyphSys.setBottomClamp(NervaGlyphSystem.BottomClampState.CLOSED);
            glyphSys.setLiftPower(-0.6);
            switch(vuMark) {
                case LEFT:
                case RIGHT:
                    waitFor(2.25);
                    break;
                case CENTER:
                default:
                    waitFor(.5);
                    break;
            }
            glyphSys.setLiftPower(0);


            drive.turnPIDslow(270);
            drive.stop();

            intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.BACK);
            intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.BACK);
            switch(vuMark) {
                case LEFT:
                case RIGHT:
                    drive.driveWithCorrectionSlow(25, 270, .05, new TimeStopParameter(1.75));
                    drive.stop();
                    glyphSys.setTopClamp(NervaGlyphSystem.TopClampState.OPEN);
                    glyphSys.setBottomClamp(NervaGlyphSystem.BottomClampState.OPEN);
                    waitFor(.5);
                    drive.driveForY(-4);
                    glyphSys.setLiftPower(.45);
                    waitFor(1);
                    glyphSys.setLiftPower(0);
                    drive.stop();
                    break;
                case CENTER:
                default:
                    drive.driveWithCorrectionSlowX(-7, 270, 0.05, new TimeStopParameter(1.5));
                    drive.stop();
                    waitFor(.15);
                    drive.driveWithCorrectionSlow(25, 270, .05, new TimeStopParameter(1.75));
                    glyphSys.setTopClamp(NervaGlyphSystem.TopClampState.OPEN);
                    glyphSys.setBottomClamp(NervaGlyphSystem.BottomClampState.OPEN);
                    //back up a little bit
                    drive.driveForY(-2);
                    drive.stop();

                    //Close Intake
                    intakeSys.setLeftArm(NervaIntakeSystem.ArmPositions.OUT);
                    intakeSys.setRightArm(NervaIntakeSystem.ArmPositions.OUT);
                    intakeSys.spitGlyph();
                    waitFor(1);
                    glyphSys.setLiftPower(.45);
                    drive.driveForY(-4);
                    glyphSys.setLiftPower(0);
                    intakeSys.stop();
                    break;
            }


        }
        else {
            drive.driveWithCorrectionSlow(-35, 90);
            drive.stop();
            intakeSys.stop();
        }
        glyphSys.zeroLiftEnc();
        //transition to teleop
        NervaPersist.lastWasAuto = true;
        NervaPersist.lastAngle = AngleUtil.normalize(drive.getHeading() - 90);
    }

}
