package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.MagnetHealthValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.ArmKinematics.Point;
import frc.robot.lib.math.NRUnits;


public class ArmSubsystem extends SubsystemBase {
    private TalonFX tromboneSlide;  //Controls the extension/retraction of the arm
    private TalonFXConfigurator tuningSlide;
    private TalonFXConfiguration tuneConfig;

    private CANcoder encoder;
    private CANcoderConfigurator encoderConfigurator;
    private CANcoderConfiguration encoderConfig;

    private MotionMagicDutyCycle extendSetter;
    private MotionMagicDutyCycle pivotSetter;

    private TalonFX kick1, kick2; //Control the pivoting of the entire arm
    private TalonFXConfigurator foot;
    private TalonFXConfiguration footConfig;

    public static ArmStatus status = ArmStatus.OK;

    //Not an FMS (Flying Monster Spaghetti) :(
    public static enum ArmStatus{
        OK,
        ENCODER_BAD,
        EXTEND_BAD,
        PIVOT_BAD
    }
    
    public ArmSubsystem(){
        tromboneSlide = new TalonFX(Constants.Arm.ARM_PORT, "drivet");
        tuningSlide = tromboneSlide.getConfigurator();
        extendSetter = new MotionMagicDutyCycle(0, true, 0, 0, false); 
        pivotSetter = new MotionMagicDutyCycle(0, true, 0, 0, false);


        kick1 = new TalonFX(Constants.Arm.PIVOT_PORT_1, "drivet");
        kick2 = new TalonFX(Constants.Arm.PIVOT_PORT_2, "drivet");
        Follower kickFollow = new Follower(Constants.Arm.PIVOT_PORT_1, true);
        kick2.setControl(kickFollow);

        foot = kick1.getConfigurator();

        encoder = new CANcoder(Constants.Arm.ENCODER_PORT, "drivet");
        encoderConfigurator = encoder.getConfigurator();

        config();
    }

    private static ArmSubsystem singleton;
    public static ArmSubsystem getInstance(){
        if(singleton == null) singleton = new ArmSubsystem();
        return singleton;
    }


    // Kinematics:
    /*
     *  Define (0, 0) As the arm's resting position 
     * Units: meters, radians
     * x axis = floor
     * y axis = arm
     */

    private final double PIVOT_HEIGHT = 0;

    public void setPoint(double x, double y){
        if(!inDomain(x, y)) return;

        //Pivot
        double angle = Math.atan2(x, y-PIVOT_HEIGHT);   //Rotate 90 degrees, then flip over the x axis
        pivot(angle);

        //Extension
        double  length = Math.sqrt(x*x + (y-PIVOT_HEIGHT)*(y-PIVOT_HEIGHT));
        double nu = NRUnits.Extension.mToNU(length);
        extendNU(nu);
    }

    public boolean inDomain(double x, double y){
        return true;
    }















































    public void config(){
        kick2.setControl(new Follower(Constants.Arm.PIVOT_PORT_1, true));

        //Configure the extension motor
        tuneConfig = new TalonFXConfiguration();
        tuneConfig.Slot0.kS = 0;
        tuneConfig.Slot0.kV = Constants.Arm.ARM_KF;
        tuneConfig.Slot0.kP = Constants.Arm.ARM_KP;
        tuneConfig.Slot0.kI = Constants.Arm.ARM_KI;
        tuneConfig.Slot0.kD = Constants.Arm.ARM_KD;

        tuneConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        tuneConfig.CurrentLimits.StatorCurrentLimit = 0;
        tuneConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        tuneConfig.CurrentLimits.SupplyCurrentLimit = 0;

        tuneConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        tuneConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        tuneConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        tuneConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.EXTEND_FORWARD_SOFT_LIMIT;
        tuneConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        tuneConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.EXTEND_REVERSE_SOFT_LIMIT;

        tuneConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.ARM_CRUISE_VELOCITY;
        tuneConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.ARM_ACCELERATION;
        tuneConfig.MotionMagic.MotionMagicJerk = 0;

        tuningSlide.apply(tuneConfig);

        //Configure the pivot motors
        footConfig = new TalonFXConfiguration();
        footConfig.Slot0.kS = 0;
        footConfig.Slot0.kV = Constants.Arm.PIVOT_KF;
        footConfig.Slot0.kP = Constants.Arm.PIVOT_KP;
        footConfig.Slot0.kI = Constants.Arm.PIVOT_KI;
        footConfig.Slot0.kD = Constants.Arm.PIVOT_KD;

        footConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        footConfig.CurrentLimits.StatorCurrentLimit = 0;
        footConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        footConfig.CurrentLimits.SupplyCurrentLimit = 0;

        footConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        footConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        footConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT;
        footConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        footConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT;

        footConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.PIVOT_CRUISE_VELOCITY;
        footConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.PIVOT_ACCELERATION;
        footConfig.MotionMagic.MotionMagicJerk = 0;

        //Remote CANcoder
        footConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        foot.apply(footConfig);


        encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.FutureProofConfigs = true;

        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.MagnetOffset = Constants.Arm.ENCODER_OFFSET;

        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        encoderConfigurator.apply(encoderConfig);
    }

    public void addToAbsoluteOffset(double offset) {
        encoderConfig.MagnetSensor.MagnetOffset += offset;
        encoderConfigurator.apply(encoderConfig);
    }

    public void zeroExtend(){
        tromboneSlide.setRotorPosition(0);
    }

    public void extendNU(double nu){
        // if(Constants.Logging.ARM) LogManager.appendToLog(nu, "Arm:/Extender/SetNU");
        extendSetter.Position = nu;
        tromboneSlide.setControl(extendSetter);
    }

    public double getEncoderDeg(){
        return encoder.getAbsolutePosition().getValue() * 360;
    }

    public boolean encoderOK(){
        if(encoder.getFault_BadMagnet().getValue()) return false;
        if(encoder.getFault_BootDuringEnable().getValue()) return false;
        if(encoder.getFault_Hardware().getValue()) return false;
        if(encoder.getFault_Undervoltage().getValue()) return false;

        if(encoder.getFaultField().getError() != StatusCode.OK) return false;

        if(encoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Green) return false;

        return true;
    }

    public boolean motorOK(TalonFX motor){
        if(!motor.isAlive()) return false;

        if(motor.getFault_DeviceTemp().getValue()) return false;
        if(motor.getFault_BootDuringEnable().getValue()) return false;
        if(motor.getFault_FusedSensorOutOfSync().getValue()) return false;
        if(motor.getFault_MissingRemoteSensor().getValue()) return false;
        if(motor.getFault_OverSupplyV().getValue()) return false;
        if(motor.getFault_ProcTemp().getValue()) return false;
        if(motor.getFault_Undervoltage().getValue()) return false;
        if(motor.getFault_UnstableSupplyV().getValue()) return false;

        if(motor.getFaultField().getError() != StatusCode.OK) return false;

        return true;
    }

    public boolean extendOK(){
        return motorOK(tromboneSlide);
    }

    public boolean pivotOK(){
        return motorOK(kick1) && motorOK(kick2);
    }

    public void resetPivotNU(){
        kick1.setRotorPosition(NRUnits.Pivot.degToRot(getEncoderDeg()));
    }

    //Pivots arm to specified angle (radians) (0 = upright)
    public void pivot(double angle){
        double NU = NRUnits.Pivot.radToRot(angle);
        // if(Constants.Logging.ARM) LogManager.appendToLog(NU, "Arm:/Pivot2/SetPosition");

        // double ff = 0;
        // if(NU >= 8552.632){
        //     ff = 0.00000076 * tromboneSlide.getPosition().getValue()-0.00653;
        // }

        // ff *= -Math.sin(angle);

  
        pivotSetter.Slot = 0;
        pivotSetter.Position = NU;
        // posSetter.FeedForward = ff;
        kick1.setControl(pivotSetter);
    }

    public double getExtendNU(){
        return tromboneSlide.getPosition().getValue();
    }

    public double getPivotPos(){
        return kick1.getPosition().getValue();
    }

    public double getPivotDeg(){
        return NRUnits.Pivot.rotToRad(getPivotPos()) * 360/Constants.TAU;
    }
    //Returns the angle of the arm
    public double getPivotRad(){
        return getPivotDeg() * Constants.TAU/360;
    }

    public void setPivotCruiseVelocity(double cruiseVelocity) {
        footConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        foot.apply(footConfig);
    }

    public void setPivotAcceleration(double acceleration) {
        footConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        foot.apply(footConfig);
    }   

    public void setPivotJerk(double jerk){
        footConfig.MotionMagic.MotionMagicJerk = jerk;
        foot.apply(footConfig);
    }

    public void setExtendCruiseVelocity(double cruiseVelocity) {
        tuneConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        tuningSlide.apply(tuneConfig);
    }
    public void setExtendAcceleration(double acceleration) {
        tuneConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        tuningSlide.apply(tuneConfig);
    }
    public void setExtendJerk(double jerk){
        tuneConfig.MotionMagic.MotionMagicJerk = jerk;
        tuningSlide.apply(tuneConfig);
    }

    public void setMotionMagicConfigs(double extendCVel, double extendAcc, double pivotCVel, double pivotAcc){
        setExtendCruiseVelocity(extendCVel);
        setExtendAcceleration(extendAcc);

        setPivotCruiseVelocity(pivotCVel);
        setPivotAcceleration(pivotAcc);
    }

    public void setDefaultCruiseVelocity() {
        setExtendCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        setPivotCruiseVelocity(Constants.Arm.PIVOT_CRUISE_VELOCITY);
    }

    public void setDefaultAcceleration() {
        setExtendAcceleration(Constants.Arm.ARM_ACCELERATION);
        setPivotAcceleration(Constants.Arm.PIVOT_ACCELERATION);
    }
}