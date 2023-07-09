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
        public static double mToRot(double m){
            return 52.0051*m - 38.581;
        }

        public static double rotToM(double rot){
            return 0.0191262*rot + 0.743667;    //Error of ~2%
        }

        public static double NUtoMPS(double NU){
            //Converts NU/100ms to meters/s
            NU = rotToM(NU);
    
            return NU;
        }

        public static double mpsToNU(double mps){
            mps = mToRot(mps);
    
            return mps;
        }
    }
    
    public static class Pivot {
        public static double radToRot(double angle){
            //Convert angle into rotations
            angle /= Constants.TAU;
    
            //Convert from arm rotations into motor rotations
            // angle *= Constants.Arm.PIVOT_GEARRATIO;
    
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
            // pos /= Constants.Arm.PIVOT_GEARRATIO;
    
            //Convert to radians
            pos *= Constants.TAU;
    
            return pos;
        }

        // public static double NUToRPS(double NU){
        //     NU *= 10;
        //     NU = rotToRad(NU);
        //     return NU;
        // }

    }    
    public static class Grabber {
        // Converts from degrees to rotations
        public static double radToNU(double angle){
            return angle / Constants.TAU * Constants.Grabber.GEAR_RATIO;
        }

        public static double NUtoRad(double rotations) {
            return rotations * Constants.TAU / Constants.Grabber.GEAR_RATIO;
        }
    }
}