package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLogic {
    private DcMotorEx Intake = null;

    private DcMotorEx Indexer = null;

    private DigitalChannel laserInput;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum IntakeState {
        IDLE,
        INTAKE,
        INDEX,
    }

    private IntakeState IntakeState;
    // constants
    private int INTAKE_TARGET = 0;
    private int intakeCount = 0;

    public void init(HardwareMap hwMap, DcMotorEx index) {
        //
        Indexer = index;
        Intake = hwMap.get(DcMotorEx.class, "Intake");
        laserInput = hwMap.get(DigitalChannel.class, "laserInput");

        // pid
        // removed conflicting pid setup
        // tolerances for speed
        Indexer.setTargetPositionTolerance(1);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        // set case
        IntakeState = IntakeState.IDLE;
    }

    public void update() {
        switch (IntakeState) {
            case IDLE:
                if (intakeCount > 0) {
                    if (INTAKE_TARGET > 360) {
                        INTAKE_TARGET = 0;
                    }
                    Indexer.setTargetPosition(INTAKE_TARGET);
                    stateTimer.reset();
                    IntakeState = IntakeState.INTAKE;
                }
                break;
            case INTAKE:
                Intake.setPower(1);
                    if (stateTimer.seconds() >= .8) {
                        INTAKE_TARGET += 180;
                        stateTimer.reset();
                        IntakeState = IntakeState.INDEX;
                }
                break;
            case INDEX:
                Indexer.setTargetPosition(INTAKE_TARGET);
                if(stateTimer.seconds() >= .9) {
                    intakeCount--;
                    stateTimer.reset();
                    IntakeState = IntakeState.IDLE;
                }
                break;
        }
    }

    public void intakeBALLZ(int ballz) {
        intakeCount += ballz;
    }
    public int getnumballz(){
        return intakeCount;
    }
    public void setNumBallz(int ballz){
        intakeCount = ballz;
    }

    public boolean isBusy() {
        return IntakeState != IntakeState.IDLE;
    }
}
