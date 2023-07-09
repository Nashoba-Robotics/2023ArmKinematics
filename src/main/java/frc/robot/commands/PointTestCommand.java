package frc.robot.commands;

import java.util.ArrayList;

import javax.net.ssl.TrustManagerFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.ArmKinematics;
import frc.robot.lib.Point;
import frc.robot.subsystems.ArmSubsystem;

public class PointTestCommand extends CommandBase{
    ArrayList<Point> points = new ArrayList<>();
    int i;
    Point targetPoint;
    Point lastPoint;
    boolean run;

    double targetTime = 0.5;

    public PointTestCommand(){
        points.add(new Point(0, 1.3));
        points.add(new Point(0.25, 1.3));
        points.add(new Point(0.5, 1.3));
        points.add(new Point(0.75, 1.3));
        points.add(new Point(1.0, 1.3));

        run = false;
        lastPoint = new Point(0, 0);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Index", 0);

        ArmSubsystem.getInstance().setPivotAcceleration(0.49);
        ArmSubsystem.getInstance().setPivotCruiseVelocity(0.49);
        ArmSubsystem.getInstance().setExtendAcceleration(100);
        ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
    }

    @Override
    public void execute() {
        i = (int)SmartDashboard.getNumber("Index", 0);
        if(!run){
            switch(i){
                case 0:
                    targetPoint = points.get(0);
                    break;
                case 1:
                    targetPoint = points.get(1);
                    break;
                case 2:
                    targetPoint = points.get(2);
                    break;
                case 3:
                    targetPoint = points.get(3);
                    break;
                case 4:
                    targetPoint = points.get(4);
                    break;
                default:
                    break;
            }
            if(targetPoint != lastPoint){
                ArmSubsystem.getInstance().setPoint(targetPoint);
                lastPoint = targetPoint;
                run = true;
            }
        }

        SmartDashboard.putBoolean("In Domain", ArmSubsystem.getInstance().inDomain(targetPoint));
        SmartDashboard.putNumber("Target X", targetPoint.x);
        SmartDashboard.putNumber("Target Y", targetPoint.y);
        
        double currRad = ArmSubsystem.getInstance().getEncoderRad();
        double currRot = ArmSubsystem.getInstance().getExtendNU();

        Point currPoint = ArmKinematics.getPoint(currRad, currRot);
        
        if(ArmSubsystem.getInstance().closeEnough(currPoint, targetPoint)){
            run = false;
        }
    }
}
