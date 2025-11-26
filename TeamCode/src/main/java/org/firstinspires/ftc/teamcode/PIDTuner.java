package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDTuner {

    private PIDControllerRyan pidController;
    private Gamepad gamepad;
    private Telemetry telemetry;

    private double pIncrement = 0.001;
    private double iIncrement = 0.0001;
    private double dIncrement = 0.0001;

    private boolean gamepad2_dpad_up_pressed = false;
    private boolean gamepad2_dpad_down_pressed = false;
    private boolean gamepad2_dpad_left_pressed = false;
    private boolean gamepad2_dpad_right_pressed = false;
    private boolean gamepad2_a_pressed = false;
    private boolean gamepad2_b_pressed = false;

    public PIDTuner(PIDControllerRyan pidController, Gamepad gamepad, Telemetry telemetry) {
        this.pidController = pidController;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void update() {
        // Update P
        if (gamepad.dpad_up && !gamepad2_dpad_up_pressed) {
            pidController.setKp(pidController.getKp() + pIncrement);
        } else if (gamepad.dpad_down && !gamepad2_dpad_down_pressed) {
            pidController.setKp(pidController.getKp() - pIncrement);
        }

        // Update I
        if (gamepad.a && !gamepad2_a_pressed) {
            pidController.setKi(pidController.getKi() + iIncrement);
        } else if (gamepad.b && !gamepad2_b_pressed) {
            pidController.setKi(pidController.getKi() - iIncrement);
        }

        // Update D
        if (gamepad.dpad_right && !gamepad2_dpad_right_pressed) {
            pidController.setKd(pidController.getKd() + dIncrement);
        } else if (gamepad.dpad_left && !gamepad2_dpad_left_pressed) {
            pidController.setKd(pidController.getKd() - dIncrement);
        }

        gamepad2_dpad_up_pressed = gamepad.dpad_up;
        gamepad2_dpad_down_pressed = gamepad.dpad_down;
        gamepad2_dpad_left_pressed = gamepad.dpad_left;
        gamepad2_dpad_right_pressed = gamepad.dpad_right;
        gamepad2_a_pressed = gamepad.a;
        gamepad2_b_pressed = gamepad.b;

        telemetry.addData("PID Tuner", "--- (Gamepad 2) ---");
        telemetry.addData("P (dpad up/down)", "%.4f", pidController.getKp());
        telemetry.addData("I (a/b)", "%.4f", pidController.getKi());
        telemetry.addData("D (dpad left/right)", "%.4f", pidController.getKd());
    }
}
