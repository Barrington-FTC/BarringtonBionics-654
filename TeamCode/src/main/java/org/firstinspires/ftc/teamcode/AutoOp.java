package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="AutoOp", group="Auto") // Annotation for FTC Manager
public class AutoOp extends LinearOpMode{
    // Dc Motor dec
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Intake = null;
    private DcMotor Indexer = null;
    private DcMotorEx Flywheel = null;

    private Servo leftKicker = null;
    private Servo rightKicker = null;

    private Servo Pitch = null;
    Limelight3A limelight;
    // Constants
    // indexer
    int rotationCounter = 360;
    int ballOne = 0;
    int ballTwo = ballOne + 180;
    int ballThree = ballTwo + 180;
    int[] ordera = { 2, 1, 1 };
    int[] orderb = { 1, 2, 1 };
    int[] orderc = { 1, 1, 2 };
    int[] targetOrder = orderc;
    int ballOneColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballTwoColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballThreeColor = 0;// 0 acts as null 1 is purple 2 is green

    boolean ballOneCounted = false;
    boolean ballTwoCounted = false;
    boolean ballThreeCounted = false;

    boolean purple = false;
    boolean green = false;

    int[] ballPosArray = { ballOne, ballTwo, ballThree };
    int[] ballColorArray = { ballOneColor, ballTwoColor, ballThreeColor };
    boolean lastintake = true;
    int segments = 0; // if moved too intake + 2 rotations moved to shooting + 1

    // cr servo
    private static final int rDistance = 0; // the change in encoder value between each slot
    // For normal servo mode
    private static final int Left = 0;
    private static final int Center = 0;
    private static final int Right = 0;
    // constants for indexer
    private ColorSensorV3 colorSensor;

    // declared like this so if motors are swapped I only have to find one value
    int TargetPosition = ballOne;
    int Intakepos = ballOne;
    int Shootingpos = ballOne + 270;
    private PIDControllerRyan indexerPID = null;
    private PIDTuner pidTuner = null;

    // constants for Limlight
    // constants for Limlight

    // Ball properties
    private static final double BALL_RADIUS = .0635; // 5" diameter = 0.127m -> radius = 0.0635m
    private static final double BALL_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
    public static final double HEIGHT_DIFF_METERS = 0.60; // Example: 60cm difference
    // Angle your limelight is mounted at (Degrees). 0 = pointing straight forward,
    // 20 = angled up.
    // light

    static double TargetX = 0;// find this

    double TargetAngle = 0;

    public static double VF = 0;
    int targetID = 0;
    double tx = 0;
    double ty = 0;

    double ta = 0;
    boolean autoIntake = false;
    public void runOpMode() throws InterruptedException{

        // Lime Light
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        // base
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        Indexer = hardwareMap.get(DcMotor.class, "Indexer");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        rightKicker = hardwareMap.get(Servo.class, "rightKicker");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftKicker.setDirection(Servo.Direction.FORWARD);
        rightKicker.setDirection(Servo.Direction.FORWARD);
        Pitch = hardwareMap.get(Servo.class, "Pitch");
        leftKicker.setDirection(Servo.Direction.FORWARD);// 0 is min angle 1 is max angle
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // intake
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftKicker.setPosition(1);
        rightKicker.setPosition(0);
        // Color
        colorSensor = new ColorSensorV3(hardwareMap, telemetry, "colorSensor");
        // telemetry dec
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        indexerPID = new PIDControllerRyan(0.005, 0.0001, 0.0001, 0, Indexer);
        pidTuner = new PIDTuner(indexerPID, gamepad2, telemetry);
        ElapsedTime timer = new ElapsedTime();
        // threads
        Thread sortingThread = new Thread(this::SortingLoop);
        Thread LimeLightThread = new Thread(this::limeLightLoop);
        Thread colorThread = new Thread(this::color);
        Thread pidThread = new Thread(this::PIDLoop);

        waitForStart();
        pidThread.start();
        sortingThread.start();
        LimeLightThread.start();
        colorThread.start();
        if(opModeIsActive()){
            telemetry.addData("ll status",limelight.getConnectionInfo());
            telemetry.update();
            Flywheel.setVelocity(1300);
            leftFrontDrive.setPower(-1);
            leftBackDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(-1);
            sleep(750);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            shoot();
            sleep(1500);
            Kick();
            shoot();
            sleep(1500);
            Kick();
            shoot();
            sleep(1500);
            Kick();
            sleep(1500);
            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(-0.5);
            sleep(750);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(500);
            leftFrontDrive.setPower(0.2);
            leftBackDrive.setPower(0.2);
            rightFrontDrive.setPower(0.2);
            rightBackDrive.setPower(0.2);
            sleep(500);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

    }
    // Dedicated method for the PID loop
    private void color(){
        while(opModeIsActive()){
            if(colorSensor.isPurple()){
                purple = true;
                sleep(500);
            }
            else{
                purple = false;
            }
            if(colorSensor.isGreen()){
                green = true;
                sleep(500);
            }
            else {
                green = false;

            }
            sleep(100);
        }}
    private void intake(){
        int temp = TargetPosition;
        if (!lastintake) {
            lastintake = true;
            temp += 90;
            ballOne += 90;
            ballTwo = ballOne + 180;
            ballThree = ballTwo + 180;

        } else {
            temp += 180;
            ballOne += 180;
            ballTwo = ballOne + 180;
            ballThree = ballTwo + 180;
        }
        if (ballOne > Intakepos) {
            Intakepos += 540;
        }
        if (temp >= rotationCounter) {
            rotationCounter += 540;
            Shootingpos += 540;
        }
        TargetPosition = temp;
    }
    private void shoot(){
        int temp = TargetPosition;
        if (lastintake) {
            lastintake = false;
            temp += 90;
            ballOne += 90;
            ballTwo = ballOne + 180;
            ballThree = ballTwo + 180;
        } else {
            temp += 180;
            ballOne += 180;
            ballTwo = ballOne + 180;
            ballThree = ballTwo + 180;
        }
        if (temp >= rotationCounter) {
            rotationCounter += 540;
            Intakepos += 540;
            Shootingpos += 540;
        }
        if (ballOne > Intakepos) {
            Intakepos += 540;
        }
        TargetPosition = temp;
    }

    private void PIDLoop() {
        // The PID controller calculates the necessary power to reach the TargetPosition
        double inpower = indexerPID.update(TargetPosition); // Use the D-pad updated TargetPosition
        Indexer.setPower(inpower);
    }

    private void SortingLoop() {
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.xWasPressed()) {
                int[] ballPosArray = { ballOne, ballTwo, ballThree };
                int[] ballColorArray = { ballOneColor, ballTwoColor, ballThreeColor };
                boolean arrived;
                for (int target : targetOrder) {
                    int i = 0;
                    arrived = false;
                    while (i < 3) {
                        if (ballColorArray[i] != target) {
                            i++;
                            continue;
                        }
                        if (!arrived) {
                            arrived = true;
                            int diff;
                            if (lastintake) {
                                if (ballPosArray[i] + 90 == Shootingpos) {
                                    diff = 90;
                                } else if (ballPosArray[i] + 270 == Shootingpos) {
                                    diff = 270;
                                } else {
                                    diff = 450;
                                }
                                lastintake = false;
                            } else {
                                if (ballPosArray[i] == Shootingpos) {
                                    diff = 0;
                                } else if (ballPosArray[i] + 180 == Shootingpos) {
                                    diff = 180;
                                } else {
                                    diff = 360;
                                }
                            }
                            int temp = TargetPosition + diff;
                            ballOne += diff;
                            ballTwo += diff;
                            ballThree += diff;
                            if (temp >= rotationCounter) {
                                rotationCounter += 540;
                                Shootingpos += 540;
                            }
                            if (ballOne > Intakepos) {
                                Intakepos += 540;
                            }
                            sleep(500);
                            TargetPosition = temp;
                        } else {
                            if (gamepad1.a) {
                                Kick();
                                if (ballColorArray[i] == ballOneColor) {
                                    ballOneColor = 0;
                                    ballOneCounted = false;
                                } else if (ballColorArray[i] == ballTwoColor) {
                                    ballTwoColor = 0;
                                    ballTwoCounted = false;
                                } else {
                                    ballThreeColor = 0;
                                    ballThreeCounted = false;
                                }

                                break;
                            } else if (gamepad1.y) {
                                break;

                            }
                        }
                    }
                }
            }
            sleep(90);
        }

    }
    // Dedicated method for the Limlight;

    private void limeLightLoop() {

            LLResult result = limelight.getLatestResult();
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            if(ta>=0.7 && ta<6){
                VF=-53.81907*Math.pow(ta,4) + 373.73888*Math.pow(ta,3) -783.69634 * Math.pow(ta,2) + 361.9612 * ta + 1500.46531;
            }
            else if(ta<0.7 && ta>0){
                VF = -1428.57143*ta +2021.42857;
            }
        Flywheel.setVelocity(VF);
        sleep(100);

    }



    private void Kick() {
        leftKicker.setPosition(0);
        rightKicker.setPosition(1);
        sleep(500);
        leftKicker.setPosition(1);
        rightKicker.setPosition(0);
    }
    private void sort(){
        int[] ballPosArray = { ballOne, ballTwo, ballThree };
        int[] ballColorArray = { ballOneColor, ballTwoColor, ballThreeColor };
        boolean arrived;
        for (int target : targetOrder) {
            int i = 0;
            arrived = false;
            while (i < 3) {
                if (ballColorArray[i] != target) {
                    i++;
                    continue;
                }
                if (!arrived) {
                    arrived = true;
                    int diff;
                    if (lastintake) {
                        if (ballPosArray[i] + 90 == Shootingpos) {
                            diff = 90;
                        } else if (ballPosArray[i] + 270 == Shootingpos) {
                            diff = 270;
                        } else {
                            diff = 450;
                        }
                        lastintake = false;
                    } else {
                        if (ballPosArray[i] == Shootingpos) {
                            diff = 0;
                        } else if (ballPosArray[i] + 180 == Shootingpos) {
                            diff = 180;
                        } else {
                            diff = 360;
                        }
                    }
                    int temp = TargetPosition + diff;
                    ballOne += diff;
                    ballTwo += diff;
                    ballThree += diff;
                    if (temp >= rotationCounter) {
                        rotationCounter += 540;
                        Shootingpos += 540;
                    }
                    if (ballOne > Intakepos) {
                        Intakepos += 540;
                    }
                    sleep(500);
                    TargetPosition = temp;
                } else {
                        Kick();
                        if (ballColorArray[i] == ballOneColor) {
                            ballOneColor = 0;
                            ballOneCounted = false;
                        } else if (ballColorArray[i] == ballTwoColor) {
                            ballTwoColor = 0;
                            ballTwoCounted = false;
                        } else {
                            ballThreeColor = 0;
                            ballThreeCounted = false;
                        }
                        break;

                    }
                sleep(500);
                }
            }
        }




    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }


}