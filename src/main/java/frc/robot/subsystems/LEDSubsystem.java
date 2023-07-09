package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem.ArmStatus;

public class LEDSubsystem extends SubsystemBase{
    private CANdle candle;

    public LEDSubsystem(){
        candle = new CANdle(0, "drivet");
    }

    @Override
    public void periodic() {
        if(ArmSubsystem.status != ArmStatus.OK){
            candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.3, 116, 0), 0);
        }
        else if(DriverStation.isDisabled()){
            candle.animate(new LarsonAnimation(0xFF, 0x10, 0x0, 0, 0.3, 116, BounceMode.Back, 7), 0);
        }
        else{
            candle.clearAnimation(0);
            candle.setLEDs(148, 0, 211, 0, 0, 116);
        }
    }
}
