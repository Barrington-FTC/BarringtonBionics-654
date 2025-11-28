package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@Config
@TeleOp(name="TestTeleOp")
public class TestTeleOp extends LinearOpMode {
    //Dc Motor dec
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Intake = null;
    private DcMotor Indexer = null;
    private DcMotor Flywheel = null;

    private Servo leftKicker =null;
    private Servo rightKicker =null;
   // Limelight3A limelight;
    //Constants
    //indexer
    int rotationCounter = 360;
    int ballOne = 0;
    int ballTwo = ballOne+180;
    int ballThree= ballTwo +180;
    int[] ordera = {2,1,1};
    int[] orderb = {1,2,1};
    int[] orderc = {1,1,2};
    int[] targetOrder = {1};
    int ballOneColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballTwoColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballThreeColor = 0;// 0 acts as null 1 is purple 2 is green

    boolean ballOneCounted = false;
    boolean ballTwoCounted = false;
    boolean ballThreeCounted = false;
    int[] ballPosArray = {ballOne,ballTwo,ballThree};
    int[] ballColorArray = {ballOneColor,ballTwoColor,ballThreeColor};
    int Shootingpos = ballTwo+90;
    boolean lastintake = true;
    int segments = 0; //if moved too intake + 2 rotations moved to shooting + 1
    private static final int one = 90; //intake
    private static final int two = 109; // shooting
    private static final int three = 180;//intake
    private static final int four = 293;//shooting
    private static final int five = 360;//intake
    private static final int six = 470;//shooting


    //cr servo
    private static final int rDistance = 0; //the change in encoder value between each slot
    //For normal servo mode
    private static final int Left = 0;
    private static final int Center = 0;
    private static final int Right = 0;
    //constants for indexer
    private ColorSensorV3 colorSensor;
    int TargetPosition = 0;
    private PIDControllerRyan indexerPID = null;
    private PIDTuner pidTuner = null;

    //constants for Limlight

    private static final double llHeight = 0;
    private static final double flyWheelR = 0;
    int targetID = 0;
    double tx = 0;
    double ty = 0;

    boolean canSeeTarget = false;

    @Override
    public void runOpMode() {

        //Lime Light
       // limelight = hardwareMap.get(Limelight3A.class, "limelight");
       // limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
       // limelight.start();
       // limelight.pipelineSwitch(0);
        //base
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        Indexer = hardwareMap.get(DcMotor.class, "Indexer");
        Flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        rightKicker = hardwareMap.get(Servo.class, "rightKicker");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        leftKicker.setDirection(Servo.Direction.FORWARD);
        rightKicker.setDirection(Servo.Direction.FORWARD);
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftKicker.setPosition(1);
        rightKicker.setPosition(0);
        //shooting


        //telemetry dec
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        indexerPID = new PIDControllerRyan(0.005, 0.0001, 0.0002, 0, Indexer);
        pidTuner = new PIDTuner(indexerPID, gamepad2, telemetry);
        Thread indexerPIDThread = new Thread(this::indexerPIDLoop);
        colorSensor = new ColorSensorV3(hardwareMap, telemetry, "colorSensor");
        waitForStart();
        indexerPIDThread.start();

        while (opModeIsActive()) { // Loop
            colorSensor.addTelemetry();
            pidTuner.update();
            //color sensor;
            if(TargetPosition == ballOne && !ballOneCounted){
                if(colorSensor.isPurple()){
                    ballOneColor=1;
                    ballOneCounted = true;
                }
                else if(colorSensor.isGreen()){
                    ballOneColor=2;
                    ballOneCounted = true;
                }
                else{
                    ballOneColor=0;
                }
            }
            else if(TargetPosition == ballTwo && !ballTwoCounted){
                if(colorSensor.isPurple()){
                    ballTwoColor=1;
                    ballTwoCounted = true;
                }
                else if(colorSensor.isGreen()){
                    ballTwoColor=2;
                    ballTwoCounted = true;
                }
                else{
                    ballTwoColor=0;
                }
            }
            else if(TargetPosition == ballThree && !ballThreeCounted){
                if(colorSensor.isPurple()){
                    ballThreeColor=1;
                    ballThreeCounted = true;
                }
                else if(colorSensor.isGreen()){
                    ballThreeColor=2;
                    ballThreeCounted = true;
                }
                else{
                    ballThreeColor=0;
                }
            }


            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower =  axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower =  axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Reduced Power Mode
            if (gamepad1.a) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //Intake
            if(gamepad1.right_trigger > 0.01){
                Intake.setPower(1);
            }
            else{
                Intake.setPower(0);
            }
            if(gamepad1.left_trigger >0.01){
                Flywheel.setPower(gamepad1.left_trigger);
            }
            else{
                Flywheel.setPower(0);
            }

            if(gamepad1.a){
                leftKicker.setPosition(0);
                rightKicker.setPosition(1);
                sleep(750);
                leftKicker.setPosition(1);
                rightKicker.setPosition(0);
            }
            if(gamepad1.dpadUpWasPressed()){
                int temp = TargetPosition;
                if(lastintake){
                    lastintake = false;
                    temp += 90;
                }
                else{
                    temp+=180;
                }
                if(temp > rotationCounter){
                    ballOne+=540;
                    ballTwo= ballOne + 180;
                    ballThree = ballTwo + 180;
                    rotationCounter += 360;
                    Shootingpos = ballTwo + 90;
                }
                TargetPosition = temp;
            }
            if(gamepad1.dpadRightWasPressed()){
                int temp = TargetPosition;
                if(!lastintake){
                    lastintake = true;
                    temp += 90;
                }
                else{
                    temp += 180;
                }
                if(temp > rotationCounter){
                    ballOne+=540;
                    ballTwo= ballOne + 180;
                    ballThree = ballTwo + 180;
                    rotationCounter += 360;
                    Shootingpos = ballTwo + 90;
                }
                TargetPosition = temp;

            }
            if(gamepad1.xWasPressed()){
                int[] ballPosArray = {ballOne,ballTwo,ballThree};
                int[] ballColorArray = {ballOneColor,ballTwoColor,ballThreeColor};
                boolean[] ballCountedArray = {ballOneCounted,ballTwoCounted,ballThreeCounted};
                for(int target : targetOrder){
                    for(int i = 0; i<3; i++){
                        if(ballColorArray[i] == target){
                            lastintake = false;
                            if(ballCountedArray[i] == ballOneCounted){
                                ballOneCounted = false;
                            }
                            else if(ballCountedArray[i] == ballTwoCounted){
                                ballTwoCounted = false;
                            }
                            else{
                                ballThreeCounted = false;
                            }
                            // error here
                            if(ballPosArray[i] < Shootingpos){
                                int diff = ballPosArray[i] - Shootingpos;
                                int temp = ballPosArray[i] + diff;
                                if(temp > rotationCounter){
                                    rotationCounter += 360;
                                    ballOne+=540;
                                    ballTwo= ballOne + 180;
                                    ballThree = ballTwo + 180;
                                    Shootingpos = ballTwo + 90;
                                    TargetPosition = temp;
                                }
                            }
                            else{
                                int diff = ballPosArray[i] + 540 - Shootingpos;
                                int temp = ballPosArray[i] + diff;
                                if(temp > rotationCounter){
                                    rotationCounter += 360;
                                    ballOne+=540;
                                    ballTwo= ballOne + 180;
                                    ballThree = ballTwo + 180;
                                    Shootingpos = ballTwo + 90;
                                    TargetPosition = temp;
                                }

                            }
                        }
                    }

                }
            }


            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.

            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Indexer Position", "%d",
                    Indexer.getCurrentPosition());
            telemetry.addData("Indexer Target Position", "%d",
                    TargetPosition);
            telemetry.addData("Target order", targetOrder);
            telemetry.addData("Ball one pos", ballOne);
            telemetry.addData("Ball two pos ", ballTwo);
            telemetry.addData("Ball three pos", ballThree);
            telemetry.addData("Ball one color", ballOneColor);
            telemetry.addData("Ball two color", ballTwoColor);
            telemetry.addData("Ball three color", ballThreeColor);
            telemetry.addData("Ball one counted", ballOneCounted);
            telemetry.addData("Ball two counted", ballTwoCounted);
            telemetry.addData("Ball three counted", ballThreeCounted);
            telemetry.addData("Shootingpos", Shootingpos);
            telemetry.update();
        }
    }

    // Dedicated method for the PID loop
    private void indexerPIDLoop() {
        while (opModeIsActive() && !isStopRequested()) { // Loop until OpMode stops
            // The PID controller calculates the necessary power to reach the TargetPosition
            double power = indexerPID.update(TargetPosition); // Use the D-pad updated TargetPosition
            Indexer.setPower(power);
            // Sleep to prevent the loop from hogging the CPU
            sleep(20); // Sleep for 20ms (a 50Hz loop is common for PID)
        }
    }
    // Dedicated method for the Limlight;
/*
    private void limeLightLoop() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        boolean isfound = false;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id == targetID) {
                tx = fiducial.getTargetXDegrees(); // Where it is (left-right)
                ty = fiducial.getTargetYDegrees(); // Where it is (up-down)
                isfound = true;

            }
        }
        if (isfound == false){
            canSeeTarget = false;
        }
        else{
            canSeeTarget = true;
        }
    }

 */
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
