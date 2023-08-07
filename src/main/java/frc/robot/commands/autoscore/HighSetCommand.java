package frc.robot.commands.autoscore;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class HighSetCommand extends CommandBase{
    double pivotAngle;
    double extendNU;
    double targetTime;  //seconds
    boolean atPrep;
    public HighSetCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setPivotAcceleration(0.9);
        ArmSubsystem.getInstance().setExtendAcceleration(120);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(0.1);
        ArmSubsystem.getInstance().pivot(Constants.Arm.PREP_ANGLE);
        atPrep = false;

        pivotAngle = Constants.Arm.HIGH_FRONT_ANGLE;
        extendNU = Constants.Arm.HIGH_EXTEND_NU+1.75;

        // SmartDashboard.putNumber("Prep Time", 2);
        targetTime = 0.1;

        GrabberSubsystem.getInstance().orientPos(-7);
    }

    @Override
    public void execute() {
        if(true ||Math.abs(ArmSubsystem.getInstance().getEncoderRad()-Constants.Arm.PREP_ANGLE) < 2*Constants.TAU/360){
            atPrep = true;

            double pivotVelocity = Math.abs(pivotAngle-ArmSubsystem.getInstance().getEncoderRad());
            pivotVelocity /= Constants.TAU;
            pivotVelocity /= targetTime;
            ArmSubsystem.getInstance().setPivotCruiseVelocity(pivotVelocity);

            double extendVelocity = Math.abs(extendNU-ArmSubsystem.getInstance().getExtendNU());
            extendVelocity /= targetTime;
            ArmSubsystem.getInstance().setExtendCruiseVelocity(extendVelocity);
        } 

        if(atPrep){
            ArmSubsystem.getInstance().pivot(pivotAngle);
            ArmSubsystem.getInstance().extendNU(extendNU);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        // ArmSubsystem.getInstance().pivot(Constants.Arm.PREP_ANGLE);
        // ArmSubsystem.getInstance().extendNU(3);

    }
}
