package frc.robot.lib.math;

import frc.robot.Constants;

public class NRUnits {

    // Constrains the given angle to 180 DEGREES 
    public static double constrainDeg(double angle){
        angle %= 360;

        if(Math.abs(angle) <= 180){
            return angle;
        }

        if(angle > 0){
            return angle - 360;
        }
        else{
            return angle + 360;
        }
    }

    public static double constrainRad(double angle) {
        return constrainDeg(angle * 360 / Constants.TAU) * Constants.TAU / 360;
    }

    public static class Extension {
        public static double mToNU(double m){
            m *= 1000;  //Convert to mm
            m /= Constants.Arm.MM_PER_NU;   //Convert to NU
    
            return m;
        }
    
        public static double NUToM(double pos){
            pos *= Constants.Arm.MM_PER_NU;    //Convert to mm
            pos /= 1000;    //conert to m
    
            return pos;
        }

        public static double NUtoMPS(double NU){
            //Converts NU/100ms to meters/s
            NU = NUToM(NU);
            NU *= 10;
    
            return NU;
        }

        public static double mpsToNU(double mps){
            mps = mToNU(mps);
            mps /= 10;
    
            return mps;
        }
    }
    
    public static class Pivot {
        public static double radToRot(double angle){
            //Convert angle into rotations
            angle /= Constants.TAU;
    
            //Convert from arm rotations into motor rotations
            angle *= Constants.Arm.PIVOT_GEARRATIO;
    
            return angle;
        }

        public static double degToRot(double angle){
            //Covnert deg to rad
            angle *= Constants.TAU/360;

            //Yes... I'm lazy
            return radToRot(angle);
        }

        public static double rotToRad(double pos){
            //Convert to rotation of the arm
            pos /= Constants.Arm.PIVOT_GEARRATIO;
    
            //Convert to radians
            pos *= Constants.TAU;
    
            return pos;
        }

        public static double NUToRPS(double NU){
            NU *= 10;
            NU = rotToRad(NU);
            return NU;
        }

    }    
}