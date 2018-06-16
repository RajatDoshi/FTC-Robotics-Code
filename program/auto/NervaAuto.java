package net.kno3.season.relicrecovery.nerva.program.auto;

import net.kno3.opMode.AutonomousProgram;
import net.kno3.robot.Robot;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;
import net.kno3.season.relicrecovery.nerva.robot.NervaDriveSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaGlyphSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaIntakeSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaJewelSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaPersist;
import net.kno3.season.relicrecovery.nerva.robot.NervaRelicSystem;
import net.kno3.util.SimpleColor;

/**
 * Created by robotics on 10/28/2017.
 */
public abstract class NervaAuto extends AutonomousProgram {
    public NervaDriveSystem drive;
    public NervaGlyphSystem glyphSys;
    public NervaJewelSystem jewelSys;
    public NervaRelicSystem relicSys;
    public NervaIntakeSystem intakeSys;


    private SimpleColor alliance;

    public NervaAuto(SimpleColor alliance) {
        this.alliance = alliance;
        NervaPersist.lastWasAuto = false;
    }

    @Override
    protected Robot buildRobot() {
        Nerva nerva = new Nerva(this, alliance);
        drive = nerva.getSubSystem(NervaDriveSystem.class);
        glyphSys = nerva.getSubSystem(NervaGlyphSystem.class);
        jewelSys = nerva.getSubSystem(NervaJewelSystem.class);
        relicSys = nerva.getSubSystem(NervaRelicSystem.class);
        intakeSys = nerva.getSubSystem(NervaIntakeSystem.class);
        return nerva;
    }

    @Override
    public void postInit() {
        //NervaPersist.lastWasAuto = true;
        drive.modeVoltage();
    }
}
