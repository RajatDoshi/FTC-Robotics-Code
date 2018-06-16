package net.kno3.season.relicrecovery.nerva.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.Robot;
import net.kno3.robot.SubSystem;

/**
 * Created by robotics on 11/12/2017.
 */

public class NervaIntakeSystem extends SubSystem {
    private DcMotor leftIntake, rightIntake;
    private Servo leftArm, rightArm;
    private boolean rbLock, lbLock, rbLock1;
    private int intake = 0;
    private boolean leftArmOut = true, rightArmOut = true;

    public NervaIntakeSystem(Robot robot) {
        super(robot);
        ArmPositions.INIT.updateSettings(robot);
        ArmPositions.OUT.updateSettings(robot);
        ArmPositions.BACK.updateSettings(robot);
        ArmPositions.SENSE.updateSettings(robot);
    }

    @Override
    public void init() {
        this.leftIntake = hardwareMap().dcMotor.get(Nerva.ENCODER_X_KEY);
        this.rightIntake = hardwareMap().dcMotor.get(Nerva.ENCODER_Y_KEY);

        //this.rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftArm = hardwareMap().servo.get(Nerva.INTAKE_LEFT_ARM_KEY);
        this.rightArm = hardwareMap().servo.get(Nerva.INTAKE_RIGHT_ARM_KEY);

        setLeftArm(ArmPositions.INIT);
        setRightArm(ArmPositions.INIT);
    }

    @Override
    public void handle() {
        if(gamepad2().right_bumper && !rbLock) {
            leftArmOut = !leftArmOut;
            rightArmOut = leftArmOut;
            rbLock = true;
        }
        if(!gamepad2().right_bumper && rbLock) {
            rbLock = false;
        }

        //Added same code to gamepad 1
        if(gamepad1().right_bumper && !rbLock1) {
            leftArmOut = !leftArmOut;
            rightArmOut = leftArmOut;
            rbLock1 = true;
        }
        if(!gamepad1().right_bumper && rbLock1) {
            rbLock1 = false;
        }

        if(leftArmOut) {
            setLeftArm(ArmPositions.OUT);
        } else {
            setLeftArm(ArmPositions.BACK);
        }

        if(rightArmOut) {
            setRightArm(ArmPositions.OUT);
        } else {
            setRightArm(ArmPositions.BACK);
        }


        //leftIntake.setPower(0.75*(gamepad2().right_trigger - gamepad2().left_trigger));
        //rightIntake.setPower(gamepad2().right_trigger - gamepad2().left_trigger);

        //Added same code to gamepad 1
        leftIntake.setPower(0.75*(gamepad1().right_trigger - gamepad1().left_trigger));
        rightIntake.setPower(-(gamepad1().right_trigger - gamepad1().left_trigger));


        /*if(gamepad2().right_bumper && !rbLock) {
            if(intake == 1) {
                intake = 0;
            } else {
                intake = 1;
            }
            rbLock = true;
        }
        if(!gamepad2().right_bumper && rbLock) {
            rbLock = false;
        }

        if(gamepad2().left_bumper && !lbLock) {
            if(intake == -1) {
                intake = 0;
            } else {
                intake = -1;
            }
            lbLock = true;
        }
        if(!gamepad2().left_bumper && lbLock) {
            lbLock = false;
        }


        if(intake == 1) {
            leftIntake.setPower(1 - gamepad2().right_trigger * 2);
            rightIntake.setPower(1 - gamepad2().left_trigger * 2);
        } else if(intake == -1) {
            leftIntake.setPower(-1 + gamepad2().right_trigger * 2);
            rightIntake.setPower(-1 + gamepad2().left_trigger * 2);
        } else {
            leftIntake.setPower(gamepad2().left_trigger);
            rightIntake.setPower(gamepad2().right_trigger);
        }*/
    }

    public void setLeftArm(ArmPositions position) {
        leftArm.setPosition(position.getLeft());
    }

    public void setRightArm(ArmPositions position) {
        rightArm.setPosition(position.getRight());
    }

    @Override
    public void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void spitGlyph() {
        leftIntake.setPower(-.85);
        rightIntake.setPower(.85);
    }

    public void intakeGlyph() {
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
    }


    public enum ArmPositions {
        INIT("intake_arm_left_init", "intake_arm_right_init"),
        OUT("intake_arm_left_out", "intake_arm_right_out"),
        BACK("intake_arm_left_back", "intake_arm_right_back"),
        SENSE("intake_arm_left_sense","intake_arm_right_sense");

        private String left, right;
        private double leftP, rightP;

        ArmPositions(String left, String right) {
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
}
