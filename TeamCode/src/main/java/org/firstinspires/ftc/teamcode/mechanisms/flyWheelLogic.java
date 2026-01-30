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

    public enum FlywheelState{
        IDLE,
        SPIN_UP,
        SHOOT,
        RESET_GATE
    }
    private FlywheelState FlywheelState;
    //constants
    private double GATE_CLOSE_ANGLE = 0;
    private double GATE_OPEN_ANGLE = 1;

    private double GATE_OPEN_TIME = 1000;
    private double GATE_CLOSE_TIME = 1000;

    //flywheel constants
    private int shotsRemaning = 0;

    private double TARGET_FLYWHEEL_VELOCITY = 1300;

    private int shootOne = 90;
    private int shootTwo = 270;
    private int shootThree = 450;
    private int targetposition = 0;

    public void init(HardwareMap hwMap){
        pitch = hwMap.get(Servo.class, "Pitch");
        Turret = hwMap.get(DcMotorEx.class, "turret");
        Indexer = hwMap.get(DcMotorEx.class, "Indexer");
        leftKicker = hwMap.get(Servo.class, "leftKicker");
        Flywheel = hwMap.get(DcMotorEx.class, "Flywheel");
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients flyhweelconts = new PIDFCoefficients(700,0,0,17);
        Flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,flyhweelconts);

        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setTargetPosition(550);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPower(1);

        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Indexer.setTargetPosition(0);
        Indexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Indexer.setPower(1);


        FlywheelState = FlywheelState.IDLE;
        Flywheel.setVelocity(1300);
        pitch.setPosition(GATE_CLOSE_ANGLE);
        leftKicker.setPosition(1);
    }

    public void update(){
        switch(FlywheelState) {
            case IDLE:
                if(shotsRemaning > 0){
                    pitch.setPosition(GATE_OPEN_ANGLE);
                    Flywheel.setVelocity(TARGET_FLYWHEEL_VELOCITY);
                    if(shotsRemaning == 3){
                        targetposition = shootThree;
                    } else if (shotsRemaning == 2) {
                        targetposition = shootTwo;
                    }
                    else{
                        targetposition = shootOne;
                    }
                    stateTimer.reset();
                    FlywheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                Indexer.setTargetPosition(targetposition);
                if(!Indexer.isBusy()){
                    pitch.setPosition(GATE_OPEN_ANGLE);
                    stateTimer.reset();
                    FlywheelState = FlywheelState.SHOOT;
                }
                break;
            case SHOOT:
                if(stateTimer.milliseconds() > GATE_OPEN_TIME){
                    leftKicker.setPosition(.7);
                    shotsRemaning--;
                    stateTimer.reset();
                    FlywheelState = FlywheelState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if(shotsRemaning>0){
                    if(leftKicker.getPosition() == .7){
                        leftKicker.setPosition(1);
                    }
                    else if(leftKicker.getPosition() == 1){
                        stateTimer.reset();
                        FlywheelState = FlywheelState.SPIN_UP;
                    }
                }
                else{
                    stateTimer.reset();
                    FlywheelState = FlywheelState.IDLE;
                }
                break;

        }
}
    public void pids(){
        Indexer.setTargetPosition(targetposition);
        Flywheel.setVelocity(1300);
    }
    public void fireShots(int numShots){
        if(FlywheelState == FlywheelState.IDLE){
            shotsRemaning = numShots;
        }
    }

    public boolean isBusy(){
        return FlywheelState != FlywheelState.IDLE;
    }

    public int getShotsRemaning(){
        return shotsRemaning;
    }

    public FlywheelState getFlywheelState(){
        return FlywheelState;
    }
    public int getIndexerPos(){
        return Indexer.getCurrentPosition();
    }
    public int getIndexerTargetPos(){
        return  Indexer.getTargetPosition();
    }
    public double getIndexerpower(){
        return  Indexer.getPower();
    }


}
