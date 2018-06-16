package net.kno3.season.relicrecovery.nerva.program.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.kno3.opMode.DriverControlledProgram;
import net.kno3.robot.Robot;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;
import net.kno3.season.relicrecovery.nerva.robot.NervaDriveSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaPersist;
import net.kno3.season.relicrecovery.nerva.robot.NervaRelicSystem;

/**
 * Created by robotics on 10/28/2017.
 */
@TeleOp(name = "Downers")
public class NervaDowners extends DriverControlledProgram {
    public NervaDowners() {
        disableTimer();
    }

    @Override
    protected Robot buildRobot() {
        return new Nerva(this, null);
    }

    @Override
    protected void onStart() {
        NervaPersist.lastWasAuto = false;
        getRobot().getSubSystem(NervaDriveSystem.class).modeSpeed();
        getRobot().getSubSystem(NervaRelicSystem.class).limits = false;
    }
}
