package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A PID controller for regulating the velocity of a flywheel motor.
 * This class calculates the necessary motor power to maintain a target velocity.
 */
public class FlywheelPIDController {

    private double Kp, Ki, Kd, Kf;
    private DcMotorEx flywheelMotor;

    private double integralSum = 0;
    private double lastError = 0;
    private double targetVelocity = 0;

    private ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor for the Flywheel PID Controller.
     * @param Kp Proportional gain.
     * @param Ki Integral gain.
     * @param Kd Derivative gain.
     * @param Kf Feedforward gain.
     * @param motor The flywheel motor (must be a DcMotorEx).
     */
    public FlywheelPIDController(double Kp, double Ki, double Kd, double Kf, DcMotorEx motor) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.flywheelMotor = motor;
    }

    /**
     * Sets the target velocity for the flywheel.
     * @param targetVelocity The desired velocity in ticks per second.
     */
    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    /**
     * Updates the PID controller and applies the calculated power to the motor.
     * This method should be called in a loop.
     */
    public void update() {
        double currentVelocity = flywheelMotor.getVelocity();
        double error = targetVelocity - currentVelocity;

        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();

        // PIDF calculation
        double motorPower = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * targetVelocity);

        // Apply power to the motor
        flywheelMotor.setPower(motorPower);

        // Update state for next iteration
        lastError = error;
        timer.reset();
    }

    /**
     * Sets new PIDF gains for tuning purposes.
     * @param Kp Proportional gain.
     * @param Ki Integral gain.
     * @param Kd Derivative gain.
     * @param Kf Feedforward gain.
     */
    public void setGains(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return flywheelMotor.getVelocity();
    }

    public double getLastError() {
        return lastError;
    }
}