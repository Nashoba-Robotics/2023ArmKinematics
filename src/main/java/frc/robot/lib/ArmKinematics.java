package frc.robot.lib;

import java.util.ArrayList;

import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class ArmKinematics {
    public static class ArmPath{
        //Processing for 2 parametric equations
        ArrayList<Point> points;

        public ArmPath(){
            points = new ArrayList<>();
        }

        public Point getPoint(double t){
            for(int i = 1; i < points.size(); i++){
                if(t >= points.get(i-1).fuckYouBen && t <= points.get(i).fuckYouBen) return points.get(i);
            }
            if(t >= points.get(points.size()-1).fuckYouBen) return points.get(points.size()-1);
            return new Point(0, 1.3);
        }
    }

    public static class LinearArmPath extends ArmPath{
        /*
         * x = At + B
         * y = Ct + D
         * 
         * t in seconds
         */
        double A, B;
        double C, D;

        public LinearArmPath(double A, double B, double C, double D, double totalTime){
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;

            processPoints(totalTime);
        }

        public void processPoints(double totalTime){
            for(double t = 0; t <= totalTime; t += 0.5){
                double x = getX(t);
                double y = getY(t);
                this.points.add(new Point(x, y, t));
            }
        }

        public double getX(double t){
            return A*t + B;
        }
        public double getY(double t){
            return C*t + D;
        }
        public ArrayList<Point> getPoints(){
            return this.points;
        }
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
        double nu = NRUnits.Extension.mToRot(length);
        return nu;
    }

    public static Point getPoint(double rad, double extendRot){
        rad = Constants.TAU/4-rad;
        double length = NRUnits.Extension.rotToM(extendRot);

        double x = length * Math.cos(rad);
        double y = length * Math.sin(rad);
        y += 0.518;

        return new Point(x, y);
    }
}
