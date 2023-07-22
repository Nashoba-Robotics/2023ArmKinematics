package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.MagnetHealthValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.ArmKinematics;
import frc.robot.lib.Point;
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
    private PositionDutyCycle s;

    private TalonFX kick1, kick2; //Control the pivoting of the entire arm
    private TalonFXConfigurator foot;
    private TalonFXConfiguration footConfig;

    public static ArmStatus status = ArmStatus.OK;

    private double lastTime;
    private Point lastPoint;

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

        lastTime = System.currentTimeMillis()/1000;
        lastPoint = new Point(0, REST_LENGTH);

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

    private final double PIVOT_HEIGHT = 0.518;
    public void setPoint(Point p){
        setPoint(p.x, p.y);
    }
    public void setPoint(double x, double y){
        if(!inDomain(x, y)) return;
        double lastX = lastPoint.x;
        double lastY = lastPoint.y;
        double targetTime = 0.5;  //seconds
        //Pivot
        double angle = Math.atan2(x, y-PIVOT_HEIGHT);   //Rotate 90 degrees, then flip over the x axis
        double lastAngle = Math.atan2(lastX, lastY-PIVOT_HEIGHT);
        double velocity = (Math.abs(angle-lastAngle))/(targetTime);
        velocity = NRUnits.Pivot.radToRot(velocity);
        setPivotCruiseVelocity(velocity);
        pivot(angle);

        //Extension
        double  length = Math.sqrt(x*x + (y-PIVOT_HEIGHT)*(y-PIVOT_HEIGHT));
        double nu = NRUnits.Extension.mToRot(length);

        double lastLength = Math.sqrt(lastX*lastX + (lastY-PIVOT_HEIGHT)*(lastY-PIVOT_HEIGHT));
        double lastNU = NRUnits.Extension.mToRot(lastLength);
        velocity = (Math.abs(nu-lastNU))/targetTime;
        setExtendCruiseVelocity(velocity);
        extendNU(nu);


        lastPoint = new Point(x, y);
    }

    

    private final double REST_LENGTH = 0.733;
    private final double MAX_EXTEND = 1.63;
    public boolean inDomain(Point p){
        return inDomain(p.x, p.y);
    }
    public boolean inDomain(double x, double y){
        //Areas less than where the arm can reach
        if(x*x + (y-PIVOT_HEIGHT)*(y-PIVOT_HEIGHT) < REST_LENGTH*REST_LENGTH) return false;
        //Area greater than where the arm is allowed to reach
        if(x*x + (y-PIVOT_HEIGHT)*(y-PIVOT_HEIGHT) > MAX_EXTEND*MAX_EXTEND) return false;
        //Keep arm above ground
        if(y <= 0) return false;

        return true;
    }










































    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Deg", getPivotDeg());
        SmartDashboard.putNumber("Encoder Deg", getEncoderDeg());
        SmartDashboard.putNumber("Extend NU", getExtendNU());
        SmartDashboard.putNumber("Pivot Current", getPivotStator());
        SmartDashboard.putNumber("Extend Current", getExtendStator());
        SmartDashboard.putNumber("Pivot Voltage", getPivotPower());
        SmartDashboard.putBoolean("Encoder Ok", encoderOK());
        double pivotVelocity = getPivotVelocity();
        SmartDashboard.putNumber("Pivot Velocity", pivotVelocity);
        double time = System.currentTimeMillis()/1000;
        SmartDashboard.putNumber("Pivot Acceleration", pivotVelocity/(time-lastTime));
        lastTime = time;

        Point pt = ArmKinematics.getPoint(getPivotRad(), getExtendNU());
        SmartDashboard.putNumber("Arm X", pt.x);
        SmartDashboard.putNumber("Arm Y", pt.y);
    }

    public boolean invertedCorrectly(){
        return kick1.getInverted() != kick2.getInverted();
    }

    public void config(){
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

        kick2.setControl(new Follower(Constants.Arm.PIVOT_PORT_1, true));
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
        footConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        footConfig.Feedback.FeedbackRemoteSensorID = 4;
        footConfig.Feedback.FeedbackRotorOffset = Constants.Arm.ENCODER_OFFSET;
        footConfig.Feedback.RotorToSensorRatio = Constants.Arm.PIVOT_GEARRATIO;
        footConfig.Feedback.SensorToMechanismRatio = 1;
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
    public double getEncoderRad(){
        return getEncoderDeg() * Constants.TAU/360;
    }
    public boolean closeEnough(Point p1, Point p2){
        double x1 = p1.x;
        double y1 = p1.y;

        double x2 = p2.x;
        double y2 = p2.y;

        return Math.abs(x1 - x2) < 0.1 && Math.abs(y1-y2) < 0.1;
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

        double ff = -Math.sin(getPivotRad()) * 0.01128405;
        double extendRot = getExtendNU();
        if(extendRot > 15){
            ff -= 0.00199741*extendRot;
        }
  
        pivotSetter.Slot = 0;
        pivotSetter.Position = NU;
        // pivotSetter.FeedForward = ff;
        kick1.setControl(pivotSetter);
    }

    public void pivotPos(double pos){
        pivotSetter.Position = pos;
        kick1.setControl(pivotSetter);
    }
    public void setPivotSpeed(double speed){
        kick1.set(speed);
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
    public double getPivotStator(){
        return kick1.getStatorCurrent().getValue();
    }
    public double getPivotVelocity(){
        return kick1.getVelocity().getValue();
    }
    public double getPivotPower(){
        // return kick1.getSupplyVoltage().getValue();
        return kick1.getClosedLoopOutput().getValue();
    }
    public double getExtendStator(){
        return tromboneSlide.getStatorCurrent().getValue();
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

    public void setKF(double kF){
        footConfig.Slot0.kV = kF;
        foot.apply(footConfig);
    }
    public void setKP(double kP){
        footConfig.Slot0.kP = kP;
        foot.apply(footConfig);
    }
    public void setKD(double kD){
        footConfig.Slot0.kD = kD;
        foot.apply(footConfig);
    }
}