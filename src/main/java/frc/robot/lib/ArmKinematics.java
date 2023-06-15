package frc.robot.lib;

import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class ArmKinematics {
    public class Point{
        public double x, y;

        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }
    }

    public class ArmPath{
        //Processing for 2 parametric equations
    }

    public class LinearArmPath{

    }

    public class QuardaticArmPath{

    }

    public class CustomArmPath{

    }

    public static double findPivot(Point point){
        double x = point.x;
        double y = point.y;
        return Constants.TAU/4 - Math.atan((y+1.5)/x);
    }
    
    public static double findExtend(Point point){
        double x = point.x;
        double y = point.y;

        double  length = Math.sqrt(x*x + (y+1.5)*(y+1.5));
        double nu = NRUnits.Extension.mToNU(length);
        return nu;
    }
}
