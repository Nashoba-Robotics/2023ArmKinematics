package frc.robot;

import frc.robot.lib.math.NRUnits;

public class Test {
    private static final double PIVOT_HEIGHT = 0.518;
    private static final double REST_HEIGHT = 0.733;

    public static void main(String[] args){
        double x = 1;   //0.653 
        double y = 1.3;   //1.171

        /*
         *  (0, 1.3): 2.0869, 0.0
         * (0.25, 1.3): 4.114, 17.7287
         * (0.5, 1.3): 9.689, 32.594
         */

        double length = getPointLength(x, y);
        double pivot = getPointPivot(x, y);

        System.out.println("Length: " + length);
        System.out.println("Pivot: " + pivot*180/Math.PI);

        System.out.println("In Domain? " + inDomain(x, y));
        System.out.println(Constants.ArmKinematics.line.getPoints());
    }

    public static double getPointLength(double x, double y){
        double  length = Math.sqrt(x*x + (y-PIVOT_HEIGHT)*(y-PIVOT_HEIGHT));
        double nu = NRUnits.Extension.mToRot(length);
        return nu;
    }
    public static double getPointPivot(double x, double y){
        double angle = Math.atan2(x, y-PIVOT_HEIGHT);   //Rotate 90 degrees, then flip over the x axis
        return angle;
    }
    public static boolean inDomain(double x, double y){
        //Areas less than where the arm can reach
        if(x*x + (y-PIVOT_HEIGHT)*(y-PIVOT_HEIGHT) < 0.733*0.733) return false;
        //Area greater than where the arm is allowed to reach
        if(x*x + (y-PIVOT_HEIGHT)*(y-PIVOT_HEIGHT) > 1.63*1.63) return false;
        //Keep arm above ground
        if(y <= 0) return false;

        return true;
    }
}
