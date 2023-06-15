package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.ArmKinematics.LinearArmPath;
import frc.robot.lib.ArmKinematics.Point;
import frc.robot.subsystems.ArmSubsystem;

public class LinearArmCommand extends CommandBase{
    private LinearArmPath path;
    private double startTime;
    public LinearArmCommand(LinearArmPath path){
        this.path = path;
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double t = System.currentTimeMillis() - startTime;
        double x = path.getX(t);
        double y = path.getY(t);

        ArmSubsystem.getInstance().setPoint(x, y);
    }

    
}
