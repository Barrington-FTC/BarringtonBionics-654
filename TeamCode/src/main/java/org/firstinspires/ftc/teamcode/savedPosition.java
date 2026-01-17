package org.firstinspires.ftc.teamcode;

public class savedPosition {
    public static double x;
    public static double y;
    public static double heading;

    public void SavePosition(double x,double y,double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public static double getX() {
        return x;
    }
    public static double getY() {
        return y;
    }
    public static double getHeading() {
        return heading;
    }
}
