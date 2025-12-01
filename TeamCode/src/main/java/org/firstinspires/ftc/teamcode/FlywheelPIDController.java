package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelPIDController {
    private double kP;
    private double kI;
    private double kD;
    private double kF; // Feedforward term

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public FlywheelPIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double update(double targetRPM, double currentRPM) {
        double error = targetRPM - currentRPM;

        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();

        if (Math.abs(error) < 20) { // Reset integral sum when the error is small to prevent windup
            integralSum = 0;
        }

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * targetRPM);

        lastError = error;
        timer.reset();

        // Clamp the output to be between -1 and 1 for motor power
        return Math.max(-1.0, Math.min(1.0, output));
    }

    // Getters and setters for tuning
    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }
}
