package frc.robot.commands.autoscore;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ScoreCommand extends CommandBase{
    /*
     * 
     */
    public ScoreCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    double targetExtend;
    boolean scored;

    @Override
    public void initialize() {
        double targetWrist = GrabberSubsystem.getInstance().getOrientPos() + 6;
        GrabberSubsystem.getInstance().orientPos(targetWrist);
        GrabberSubsystem.getInstance().set(-0.1);

        double currExtend = ArmSubsystem.getInstance().getExtendNU();
        targetExtend = currExtend - 10;

        ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
        ArmSubsystem.getInstance().extendNU(targetExtend);
        scored = false;
    }

    @Override
    public void execute() {
        if(!scored){
            if(Math.abs(ArmSubsystem.getInstance().getExtendNU()-targetExtend) < 1) scored = true;
        }
        else{
            ArmSubsystem.getInstance().setExtendCruiseVelocity(88.7955);    //Tune these values
            ArmSubsystem.getInstance().setPivotCruiseVelocity(0.39407);

            ArmSubsystem.getInstance().pivot(0);
            ArmSubsystem.getInstance().extendNU(3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().set(0);
    }
}
 