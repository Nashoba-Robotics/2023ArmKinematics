package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class VelocityTuneCommand extends CommandBase{
    final double MAX_PIVOT_VELOCITY = 0.49;

    double lastFeez = 0;
    double lastPeez = 0;
    double lastDeez = 0;

    DataLog log;
    DoubleLogEntry pivotVelocity;
    DoubleLogEntry targetVelocity;
    double lastTarget = 0;

    double pos;
    double lastPos;

    double time;
    double lastTime;

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Velocity", 0);
        SmartDashboard.putNumber("Vel kF", 0);
        SmartDashboard.putNumber("Vel kP", 0);

        DataLogManager.start();
        log = DataLogManager.getLog();
        pivotVelocity = new DoubleLogEntry(log, "Pivot Velocity");
        targetVelocity = new DoubleLogEntry(log, "Target");

        lastPos = ArmSubsystem.getInstance().getEncoderDeg()/360;
        lastTime = System.currentTimeMillis()/1000;
    }
    @Override
    public void execute() {
        double kF = SmartDashboard.getNumber("Vel kF", 0);
        if(kF != lastFeez){
            ArmSubsystem.getInstance().setVelocityF(kF);
            lastFeez = kF;
        }
    
        double kP = SmartDashboard.getNumber("Vel kP", 0);
        if(lastPeez != kP){
            ArmSubsystem.getInstance().setVelocityKP(kP);
            lastPeez = kP;
        }

        // In percentage
        double velocity = SmartDashboard.getNumber("Velocity", 0);
        velocity *= MAX_PIVOT_VELOCITY;
        if(velocity != lastTarget){
            targetVelocity.append(velocity/MAX_PIVOT_VELOCITY);
            lastTarget = velocity;
        }

        pos = ArmSubsystem.getInstance().getEncoderDeg()/360;
        time = System.currentTimeMillis()/1000;
        double testVel = Math.abs(pos-lastPos)/(time-lastTime);
        testVel = ArmSubsystem.getInstance().getPivotVelocity();
        SmartDashboard.putNumber("Test Velocity", testVel);
        //TODO: -0.05 on setPivotSpeed seems to keep the pivot from falling
        SmartDashboard.putNumber("pos", pos);
        SmartDashboard.putNumber("lastPos", lastPos);
        lastPos = pos;
        lastTime = time;

        // ArmSubsystem.getInstance().setPivotSpeed(velocity);
        ArmSubsystem.getInstance().pivotVelocity(velocity);

        // pivotVelocity.append(ArmSubsystem.getInstance().getPivotVelocity()/MAX_PIVOT_VELOCITY);
        pivotVelocity.append(testVel/MAX_PIVOT_VELOCITY);

    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setPivotSpeed(0);
    }
}
