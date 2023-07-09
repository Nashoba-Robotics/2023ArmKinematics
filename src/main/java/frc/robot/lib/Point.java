package frc.robot.lib;

public class Point {
    public double x;
    public double y;
    public double fuckYouBen;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
    public Point(double x, double y, double t){
        this(x, y);
        fuckYouBen = t;
    }

    @Override
    public String toString(){
        return "(" + x + ", " + y + ", " + fuckYouBen + ")";
    }
}
