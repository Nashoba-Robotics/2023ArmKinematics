package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Point;
import frc.robot.lib.ArmKinematics.LinearArmPath;
import frc.robot.subsystems.ArmSubsystem;

public class LinearArmCommand extends CommandBase{
    private LinearArmPath path;
    private double startTime;
    private Point lastPoint;

    public LinearArmCommand(LinearArmPath path){
        this.path = path;
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis()/1000;
        lastPoint = new Point(0, 0);

        ArmSubsystem.getInstance().setPivotAcceleration(1);
        // ArmSubsystem.getInstance().setPivotCruiseVelocity(0.49);
        ArmSubsystem.getInstance().setExtendAcceleration(150);
        // ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
    }

    @Override
    public void execute() {
        double t = System.currentTimeMillis()/1000 - startTime;
        Point p = path.getPoint(t);

        if(p != lastPoint){
            ArmSubsystem.getInstance().setPoint(p.x, p.y);
            lastPoint = p;
        } 
        SmartDashboard.putString("Point", p.toString());
        SmartDashboard.putString("LastPoint", lastPoint.toString());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
