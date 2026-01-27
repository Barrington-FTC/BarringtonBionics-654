package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    //constants
    private int INTAKE_TARGET = 0;
    private int intakeCount = 0;
    public void init(HardwareMap hwMap){
        //
        Indexer = hwMap.get(DcMotorEx.class, "Indexer");
        Intake = hwMap.get(DcMotorEx.class, "Intake");
        laserInput = hwMap.get(DigitalChannel.class, "laserInput");

        //pid
        PIDFCoefficients indexerconts = new PIDFCoefficients(0.005,0,0,0);
        Indexer.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,indexerconts);
        //tolerances for speed
        Indexer.setTargetPositionTolerance(1);
        //set case
        IntakeState = IntakeState.IDLE;
    }

    public void update(){
        switch(IntakeState) {
            case IDLE:
                if(intakeCount>0){
                    if(INTAKE_TARGET>360){
                        INTAKE_TARGET = 0;
                    }
                    Indexer.setTargetPosition(INTAKE_TARGET);
                    stateTimer.reset();
                    IntakeState = IntakeState.INTAKE;
                }
                break;
            case INTAKE:
                if(!Indexer.isBusy()){
                    Intake.setPower(1);
                    if(laserInput.getState()){
                        INTAKE_TARGET+=180;
                        stateTimer.reset();
                        IntakeState = IntakeState.INDEX;
                    }
                }
                break;
            case INDEX:
                if(stateTimer.seconds()>=1){
                    Indexer.setTargetPosition(INTAKE_TARGET);
                    intakeCount--;
                    stateTimer.reset();
                    IntakeState = IntakeState.IDLE;
                }
                break;
        }
    }
    public void intakeBALLZ(int ballz){
        intakeCount = ballz;
    }

    public boolean isBusy(){
        return IntakeState != IntakeState.IDLE;
    }
}
