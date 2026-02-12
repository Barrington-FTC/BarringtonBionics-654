package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class flyWheelLogic {
    private DcMotorEx Flywheel = null;

    private DcMotorEx Indexer = null;

    private DcMotorEx Turret = null;
    private Servo leftKicker = null;
    private Servo pitch = null;

    private ElapsedTime stateTimer = new ElapsedTime();

    public enum FlywheelState {
        IDLE,
        SPIN_UP,
        SHOOT,
        RESET_GATE
    }

    private FlywheelState FlywheelState;
    // constants
    private double GATE_CLOSE_ANGLE = 0;
    private double GATE_OPEN_ANGLE = 1;

    private double GATE_OPEN_TIME = 1000;
    private double GATE_CLOSE_TIME = 1000;

    // flywheel constants
    private int shotsRemaning = 0;

    private double TARGET_FLYWHEEL_VELOCITY = 1300;

    private int shootOne = 90;
    private int shootTwo = 270;
    private int shootThree = 450;
    private int targetposition = 0;

    public void init(HardwareMap hwMap, DcMotorEx index) {
        Turret = hwMap.get(DcMotorEx.class, "turret");
        Indexer = index;
        leftKicker = hwMap.get(Servo.class, "leftKicker");
        Flywheel = hwMap.get(DcMotorEx.class, "Flywheel");
        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients flyhweelconts = new PIDFCoefficients(5.75, 0, 0, 2);
        Flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flyhweelconts);

        leftKicker.setDirection(Servo.Direction.FORWARD);

        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setTargetPosition(610);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPower(1);

        FlywheelState = FlywheelState.IDLE;
        leftKicker.setPosition(.01);
    }

    public void update() {
        switch (FlywheelState) {
            case IDLE:
                if (shotsRemaning > 0) {
                    Flywheel.setVelocity(1430);
                    if (shotsRemaning == 1) {
                        targetposition = shootOne;
                    }
                    if (shotsRemaning == 2) {
                        targetposition = shootTwo;
                    }
                    if (shotsRemaning == 3) {
                        targetposition = shootThree;
                    }
                    stateTimer.reset();
                    FlywheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (shotsRemaning == 1) {
                    targetposition = shootOne;
                }
                if (shotsRemaning == 2) {
                    targetposition = shootTwo;
                }
                if (shotsRemaning == 3) {
                    targetposition = shootThree;
                }
                Indexer.setTargetPosition(targetposition);
                if (!Indexer.isBusy() && Flywheel.getVelocity()<-1300) {
                    stateTimer.reset();
                    FlywheelState = FlywheelState.SHOOT;
                }
                break;
            case SHOOT:
                leftKicker.setPosition(.3);
                if (stateTimer.seconds()>.3) {
                    shotsRemaning--;
                    stateTimer.reset();
                    FlywheelState = FlywheelState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if (shotsRemaning > 0) {
                    leftKicker.setPosition(.01);
                    if (stateTimer.seconds() > .5) {
                        stateTimer.reset();
                        FlywheelState = FlywheelState.SPIN_UP;
                    }
                } else {
                    leftKicker.setPosition(.01);
                    stateTimer.reset();
                    FlywheelState = FlywheelState.IDLE;
                }
                break;

        }
    }

    public void pids() {
        // removed due to being very bad
    }

    public void fireShots(int numShots) {
        if (FlywheelState == FlywheelState.IDLE) {
            shotsRemaning = numShots;
        }
    }

    public boolean isBusy() {
        return FlywheelState != FlywheelState.IDLE;
    }

    public int getShotsRemaning() {
        return shotsRemaning;
    }

    public double getFlyhwheelVelocity(){
        return Flywheel.getVelocity();
    }

    public FlywheelState getFlywheelState() {
        return FlywheelState;
    }

    public int getIndexerPos() {
        return Indexer.getCurrentPosition();
    }

    public int getIndexerTargetPos() {
        return Indexer.getTargetPosition();
    }

    public double getIndexerpower() {
        return Indexer.getPower();
    }

    public double getkickerpos() {
        return leftKicker.getPosition();
    }

}
