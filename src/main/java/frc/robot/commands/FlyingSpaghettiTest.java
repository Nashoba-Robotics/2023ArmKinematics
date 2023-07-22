package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.ArmKinematics;
import frc.robot.lib.Point;
import frc.robot.subsystems.ArmSubsystem;

public class FlyingSpaghettiTest extends CommandBase{
    public enum ArmState{
        PREP,
        HIGH,
        MID,
        LOW
    }

    ArmState currState;
    ArmState targetState;
    boolean atState = false;

    final double targetTime = 0.3;  //seconds

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(!atState){
            switch(targetState){
                case PREP:
                    atState = atPrep();
                    if(atState) currState = ArmState.PREP;
                    break;
                case LOW:
                    if(atLow()){
                        atState = true;
                        currState = ArmState.LOW;
                    }
                    break;
                case MID:
                    if(atMid()){
                        atState = true;
                        currState = ArmState.MID;
                    }
                    break;
                case HIGH:
                    if(atHigh()){
                        atState = true;
                        currState = ArmState.HIGH;
                    }
                    break;
            }
        }

        if(currState != targetState && atState){
            double currPivot = ArmSubsystem.getInstance().getEncoderRad();
            double currExtend = ArmSubsystem.getInstance().getExtendNU();

            double targetPivot;
            double targetExtend;
            switch(targetState){
                case PREP:
                    targetPivot = Constants.Arm.PREP_ANGLE;
                    targetExtend = 3;
                    // switch(currState){

                    // }
                    //For now, we assume that the cruise velocity of the previous set was the same that we're using
                    ArmSubsystem.getInstance().pivot(targetPivot);
                    ArmSubsystem.getInstance().extendNU(targetExtend);
                    break;
                case LOW:
                    break;
                case MID:
                    break;
                case HIGH:
                    targetPivot = Constants.Arm.HIGH_FRONT_ANGLE;
                    targetExtend = Constants.Arm.HIGH_EXTEND_NU;
                    switch(currState){
                        case LOW:
                            targetState = ArmState.PREP;
                            break;
                        default:
                            setPivotVelocity(currPivot, targetPivot, targetTime);
                            setExtendVelocity(currExtend, targetExtend, targetTime);

                            ArmSubsystem.getInstance().pivot(targetPivot);
                            ArmSubsystem.getInstance().extendNU(targetExtend);
                            atState = false;
                            break;
                    }
                    break;
                    
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    public boolean atPrep(){
        Point prepPoint = ArmKinematics.getPoint(Constants.Arm.PREP_ANGLE, 3);
        Point currPoint = ArmKinematics.getPoint(ArmSubsystem.getInstance().getEncoderRad(), ArmSubsystem.getInstance().getExtendNU());

        return (prepPoint.x-currPoint.x)*(prepPoint.x-currPoint.x) + (prepPoint.y-currPoint.y)*(prepPoint.y-currPoint.y) <= 0.05;
    }
    public boolean atLow(){
        Point prepPoint = ArmKinematics.getPoint(Constants.Arm.LOW_ANGLE, Constants.Arm.LOW_EXTEND_NU);
        Point currPoint = ArmKinematics.getPoint(ArmSubsystem.getInstance().getEncoderRad(), ArmSubsystem.getInstance().getExtendNU());

        return (prepPoint.x-currPoint.x)*(prepPoint.x-currPoint.x) + (prepPoint.y-currPoint.y)*(prepPoint.y-currPoint.y) <= 0.05;
    }
    public boolean atMid(){
        Point prepPoint = ArmKinematics.getPoint(Constants.Arm.MID_ANGLE, Constants.Arm.MID_EXTEND_NU);
        Point currPoint = ArmKinematics.getPoint(ArmSubsystem.getInstance().getEncoderRad(), ArmSubsystem.getInstance().getExtendNU());

        return (prepPoint.x-currPoint.x)*(prepPoint.x-currPoint.x) + (prepPoint.y-currPoint.y)*(prepPoint.y-currPoint.y) <= 0.05;
    
    }
    public boolean atHigh(){
        //Should add an if statement for front and back
        Point prepPoint = ArmKinematics.getPoint(Constants.Arm.HIGH_FRONT_ANGLE, Constants.Arm.HIGH_EXTEND_NU);
        Point currPoint = ArmKinematics.getPoint(ArmSubsystem.getInstance().getEncoderRad(), ArmSubsystem.getInstance().getExtendNU());

        return (prepPoint.x-currPoint.x)*(prepPoint.x-currPoint.x) + (prepPoint.y-currPoint.y)*(prepPoint.y-currPoint.y) <= 0.05;
    }
    public void setPivotVelocity(double currPivot, double targetPivot, double time){
        double pivotVel = Math.abs(targetPivot-currPivot);
        pivotVel /= Constants.TAU;
        pivotVel /= time;

        ArmSubsystem.getInstance().setPivotCruiseVelocity(pivotVel);
    }
    public void setExtendVelocity(double currExtend, double targetExtend, double time){
        double extendVel = Math.abs(targetExtend-currExtend);
        extendVel /= time;
        ArmSubsystem.getInstance().setExtendCruiseVelocity(extendVel);
    }

    public static void main(String[] args){
        Point prepPoint = ArmKinematics.getPoint(Constants.Arm.HIGH_FRONT_ANGLE, Constants.Arm.HIGH_EXTEND_NU);
        Point currPoint = ArmKinematics.getPoint(1.2, 39);
        boolean b = (prepPoint.x-currPoint.x)*(prepPoint.x-currPoint.x) + (prepPoint.y-currPoint.y)*(prepPoint.y-currPoint.y) <= 0.05;

        System.out.println(b);
    }
}
