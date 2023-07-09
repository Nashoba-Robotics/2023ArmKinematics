package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class PIDTuneCommand extends CommandBase{
    //Remember to give pivot input in radians
    //Take input in degrees
    private boolean finish = false;
    private double lastKF = 0;
    private double lastKP = 0;
    private double lastKD = 0;

    private double lastTarget = 0;

    private DataLog log;
    private DoubleLogEntry pivotPos;
    private DoubleLogEntry targetPos;

    @Override
    public void initialize() {
        // if(!ArmSubsystem.getInstance().invertedCorrectly()) finish = true;
        ArmSubsystem.getInstance().setPivotCruiseVelocity(0.25);
        ArmSubsystem.getInstance().setPivotAcceleration(0.5);

        SmartDashboard.putNumber("kF", 0);
        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kD", 0);
        SmartDashboard.putNumber("Input Pos", 0);

        DataLogManager.start();
        log = DataLogManager.getLog();
        pivotPos = new DoubleLogEntry(log, "Pivot Pos");
        targetPos = new DoubleLogEntry(log, "Target");
    }

    @Override
    public void execute() {
        double kF = SmartDashboard.getNumber("kF", 0);  //6
        if(kF != lastKF){
            lastKF = kF;
            ArmSubsystem.getInstance().setKF(kF);
        }

        double kP = SmartDashboard.getNumber("kP", 0);
        if(kP != lastKP){
            lastKP = kP;
            ArmSubsystem.getInstance().setKP(kP);
        }

        double kD = SmartDashboard.getNumber("kD", 0);
        if(kD != lastKD){
            lastKD = kD;
            ArmSubsystem.getInstance().setKD(kD);
        }

        double pos = SmartDashboard.getNumber("Input Pos", 0);
        
        if(pos != lastTarget){
            lastTarget = pos;
            targetPos.append(lastTarget);
        }

        pos *= Constants.TAU/360;
        ArmSubsystem.getInstance().pivot(pos);

        pivotPos.append(ArmSubsystem.getInstance().getPivotDeg());
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setPivotSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return finish;
    }
}
