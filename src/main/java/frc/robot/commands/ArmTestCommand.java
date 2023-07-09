package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

/*
Cube:
 * -108 degrees
 * -10 NU wrist
 */

public class ArmTestCommand extends CommandBase{
    public ArmTestCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Pivot Deg Input", 0);
        SmartDashboard.putNumber("Extend Rot", 0);
        SmartDashboard.putNumber("Wrist NU", 0);
        SmartDashboard.putNumber("Grab Speed", 0);

        ArmSubsystem.getInstance().setExtendCruiseVelocity(100);
        ArmSubsystem.getInstance().setExtendAcceleration(116);


        ArmSubsystem.getInstance().setPivotAcceleration(1);
        ArmSubsystem.getInstance().setPivotCruiseVelocity(0.49);    //Max Velocity looks like 0.49
    }

    @Override
    public void execute() {
        double deg = SmartDashboard.getNumber("Pivot Deg Input", 0);
        double rad = deg * Constants.TAU/360;

        ArmSubsystem.getInstance().pivot(rad);
        // ArmSubsystem.getInstance().setPivotSpeed(deg);

        double extendRot = SmartDashboard.getNumber("Extend Rot", 0);
        ArmSubsystem.getInstance().extendNU(extendRot);

        double wristNU = SmartDashboard.getNumber("Wrist NU", 0);
        GrabberSubsystem.getInstance().orientPos(wristNU);

        double grabSpeed = SmartDashboard.getNumber("Grab Speed", 0);
        GrabberSubsystem.getInstance().set(grabSpeed);
    }
}
