package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private enum FlywheelState{
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
    private double flywheelVelocity = 0;
    private double MIN_FLYWHEEL_RPM = 1000;

    private double TARGET_FLYWHEEL_VELOCITY = 1250;
    private double MAX_FLYWHEEL_RPM = 1450;
    private double FLYWHEEL_MAX_SPINUP_TIME = 3;

    private int shootOne = 90;
    private int shootTwo = 270;
    private int shootThree = 450;

    public void init(HardwareMap hwMap){
        pitch = hwMap.get(Servo.class, "pitch");
        Turret = hwMap.get(DcMotorEx.class, "turret");
        Indexer = hwMap.get(DcMotorEx.class, "Indexer");
        leftKicker = hwMap.get(Servo.class, "leftKicker");
        Flywheel = hwMap.get(DcMotorEx.class, "flyWheel");
        PIDFCoefficients flyhweelconts = new PIDFCoefficients(0,0,0,0);
        PIDFCoefficients turretconts = new PIDFCoefficients(0.008,0,0,0);
        PIDFCoefficients indexerconts = new PIDFCoefficients(0.005,0,0,0);
        Flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,flyhweelconts);
        Turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,turretconts);
        Indexer.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,indexerconts);
        Indexer.setTargetPositionTolerance(1);

        FlywheelState = FlywheelState.IDLE;
        flywheelVelocity = MIN_FLYWHEEL_RPM;
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
                        Turret.setTargetPosition(shootThree);
                    } else if (shotsRemaning == 2) {
                        Turret.setTargetPosition(shootTwo);
                    }
                    else{
                        Indexer.setTargetPosition(shootOne);
                    }
                    stateTimer.reset();
                    FlywheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if((flywheelVelocity > MIN_FLYWHEEL_RPM || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) && Indexer.isBusy()){
                    pitch.setPosition(GATE_OPEN_ANGLE);
                    stateTimer.reset();
                    FlywheelState = FlywheelState.SHOOT;
                }
                break;
            case SHOOT:
                if(stateTimer.seconds() > GATE_OPEN_TIME){
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
                    Flywheel.setVelocity(MIN_FLYWHEEL_RPM);
                    stateTimer.reset();
                    FlywheelState = FlywheelState.IDLE;
                }
                break;

        }
}
    public void fireShots(int numShots){
        if(FlywheelState == FlywheelState.IDLE){
            shotsRemaning = numShots;
        }
    }

    public boolean isBusy(){
        return FlywheelState != FlywheelState.IDLE;
    }


}
