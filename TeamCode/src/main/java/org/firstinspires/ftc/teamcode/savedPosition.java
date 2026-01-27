package org.firstinspires.ftc.teamcode;

public class savedPosition {
    public static double x;//inches
    public static double y;//inches
    public static double heading;//radians

    public void SavePosition(double x,double y,double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public static void setX(double xval){x = xval;}
    public static void sety(double yval){y = yval;}
    public static void seth(double hval){heading = hval;}
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
