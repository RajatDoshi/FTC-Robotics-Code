package net.kno3.season.relicrecovery.nerva.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.Robot;
import net.kno3.robot.SubSystem;
import net.kno3.util.SimpleColor;

/**
 * Created by robotics on 10/28/2017.
 */

public class NervaJewelSystem extends SubSystem {
    public Servo leftArm, rightArm, leftKnocker, rightKnocker;
    public ColorSensor leftSensor, rightSensor;

    private NervaGlyphSystem glyphSystem;

    public NervaJewelSystem(Robot robot) {
        super(robot);

        ArmPosition.UP.updateSettings(robot);
        ArmPosition.DOWN.updateSettings(robot);
        ArmPosition.CENTER.updateSettings(robot);
        KnockerPosition.LEFT.updateSettings(robot);
        KnockerPosition.RIGHT.updateSettings(robot);
        KnockerPosition.CENTER.updateSettings(robot);
        KnockerPosition.DOWN.updateSettings(robot);
    }

    @Override
    public void init() {
        this.leftArm = hardwareMap().servo.get(Nerva.JEWEL_ARM_LEFT_KEY);
        //this.rightArm = hardwareMap().servo.get(Nerva.JEWEL_ARM_RIGHT_KEY);
        this.leftKnocker = hardwareMap().servo.get(Nerva.JEWEL_KNOCKER_LEFT_KEY);
        //this.rightKnocker = hardwareMap().servo.get(Nerva.JEWEL_KNOCKER_RIGHT_KEY);
        this.leftSensor = hardwareMap().colorSensor.get(Nerva.JEWEL_SENSOR_LEFT_KEY);
        //this.rightSensor = hardwareMap().colorSensor.get(Nerva.JEWEL_SENSOR_RIGHT_KEY);

        setLeftArm(ArmPosition.UP);
        //setRightArm(ArmPosition.UP);
        setRedKnocker(KnockerPosition.CENTER);
        //setRightKnocker(KnockerPosition.CENTER);

        leftSensor.enableLed(true);
        //rightSensor.enableLed(true);

        glyphSystem = robot.getSubSystem(NervaGlyphSystem.class);
    }

    @Override
    public void handle() {
        setLeftArm(ArmPosition.UP);
        //setRightArm(ArmPosition.UP);

        if(glyphSystem.hasGlyph()) setBlueKnocker(KnockerPosition.CENTER);
        else setRedKnocker(KnockerPosition.CENTER);
    }


    public void setLeftArm(ArmPosition position) {
        leftArm.setPosition(position.getLeft());
    }

    //public void setRightArm(ArmPosition position) ( rightArm.setPosition(position.getRight()); }

    public void setRedKnocker(KnockerPosition position) {
        leftKnocker.setPosition(position.getRed());
    }

    public void setBlueKnocker(KnockerPosition position) {
        leftKnocker.setPosition(position.getBlue());
    }

    /** \brief Determines the most likely color forward of the color sensor arm.
     *
     * @return
     */
    public SimpleColor getLeftColor() {

        int blueDecisions = 0;
        int redDecisions = 0;
        for (int i = 0; i < 5; i++) {
            SimpleColor detectedColor = SimpleColor.diffGetSimpleColor(leftSensor, 1);
            if (detectedColor == SimpleColor.BLUE) blueDecisions++;
            else if (detectedColor == SimpleColor.RED) redDecisions++;
            telemetry().addData("red", leftSensor.red());
            telemetry().addData("blue", leftSensor.blue());
            telemetry().update();
            try { Thread.sleep(25); } catch (InterruptedException ex) { }
        }
        telemetry().addData("blueDecisions", blueDecisions);
        telemetry().addData("redDecisions", redDecisions);
        telemetry().update();

        if ( (blueDecisions > redDecisions)
            && (blueDecisions > 0)) return SimpleColor.BLUE;
        else if ( (redDecisions > blueDecisions)
                && (redDecisions > 0)) return SimpleColor.RED;
        else return null;

        /*
        //Simple version of the code is
        SimpleColor.simpleGetSimpleColor(leftSensor);

        if (leftSensor.blue() > leftSensor.red())
            return SimpleColor.BLUE;
        else if (leftSensor.red() > leftSensor.blue())
            return SimpleColor.RED;
        else
            return null;
         */
    }

    public SimpleColor getRightColor() {

        int blueDecisions = 0;
        int redDecisions = 0;
        for (int i = 0; i < 10; i++) {
            SimpleColor detectedColor = SimpleColor.diffGetSimpleColor(rightSensor, 1);
            if (detectedColor == SimpleColor.BLUE) blueDecisions++;
            else if (detectedColor == SimpleColor.RED) redDecisions++;
            telemetry().addData("red", rightSensor.red());
            telemetry().addData("blue", rightSensor.blue());
            telemetry().update();
            try { Thread.sleep(50); } catch (InterruptedException ex) { }
        }
        telemetry().addData("blueDecisions", blueDecisions);
        telemetry().addData("redDecisions", redDecisions);
        telemetry().update();

        if ( (blueDecisions > redDecisions)
                && (blueDecisions > 1)) return SimpleColor.BLUE;
        else if ( (redDecisions > blueDecisions)
                && (redDecisions > 1)) return SimpleColor.RED;
        else return null;

        /*
        //Simple version of the code is
        SimpleColor.simpleGetSimpleColor(rightSensor);

        if (rightSensor.blue() > rightSensor.red())
            return SimpleColor.BLUE;
        else if (rightSensor.red() > rightSensor.blue())
            return SimpleColor.RED;
        else
            return null;
         */
    }

    public int getRightColorValue() {
        return leftSensor.red();
    }



    @Override
    public void stop() {

    }

    public enum ArmPosition {
        UP("jewel_arm_left_up", "jewel_arm_right_up"),
        DOWN("jewel_arm_left_down", "jewel_arm_right_down"),
        CENTER("jewel_arm_left_center", "jewel_arm_right_center");

        private String left, right;
        private double leftP, rightP;

        ArmPosition(String left, String right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return leftP;
        }

        public double getRight() {
            return rightP;
        }

        public void updateSettings(Robot robot) {
            leftP = robot.settings.getDouble(left);
            rightP = robot.settings.getDouble(right);
        }
    }

    public enum KnockerPosition {
        LEFT("jewel_knocker_red_left", "jewel_knocker_blue_left"),
        RIGHT("jewel_knocker_red_right", "jewel_knocker_blue_right"),
        CENTER("jewel_knocker_red_center", "jewel_knocker_blue_center"),
        DOWN("jewel_knocker_red_down", "jewel_knocker_blue_down");

        private String red, blue;
        private double redP, blueP;

        KnockerPosition(String red, String blue) {
            this.red = red;
            this.blue = blue;
        }

        public double getRed() {
            return redP;
        }

        public double getBlue() {
            return blueP;
        }

        public void updateSettings(Robot robot) {
            redP = robot.settings.getDouble(red);
            blueP = robot.settings.getDouble(blue);
        }
    }
}
