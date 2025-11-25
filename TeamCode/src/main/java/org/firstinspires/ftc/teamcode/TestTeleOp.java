package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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




    //Constants
    //indexer
    private static final int one = 2; //intake
    private static final int two = 109; // shooting
    private static final int three = 182;//intake
    private static final int four = 293;//shooting
    private static final int five = 369;//intake
    private static final int six = 470;//shooting


    //cr servo
    private static final int rDistance = 0; //the change in encoder value between each slot
    //For normal servo mode
    private static final int Left = 0;
    private static final int Center = 0;
    private static final int Right = 0;
    //Kicker
    private static final int Extended = 1;
    private static final int Retracted = 0;
    int[] intakepos = {one,three,five};
    int[] shootingpos = {two,four,six};
    int currentIntake = 0;
    int currentShooting = 0;
    int TargetPosition = 0;
    private PIDControllerRyan indexerPID = null;

    @Override
    public void runOpMode() {
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

        indexerPID = new PIDControllerRyan(0.1, 0, 0, 3, Indexer);
        Thread indexerPIDThread = new Thread(this::indexerPIDLoop);
        waitForStart();
        indexerPIDThread.start();
        Indexer.setTargetPosition(intakepos[0]);

        while (opModeIsActive()) { // Loop



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
            if(gamepad1.dpad_down){
                currentShooting--;
                if(currentShooting<0){
                    currentShooting=2;
                }
                Indexer.setTargetPosition(shootingpos[currentShooting]);
            }
            if(gamepad1.dpad_up){
                currentShooting++;
                if(currentShooting>2){
                    currentShooting=0;
                }
                Indexer.setTargetPosition(shootingpos[currentShooting]);

            }
            if(gamepad1.dpad_left){
                    currentIntake++;
                    if(currentIntake>2){
                        currentIntake=0;
                    }
                    Indexer.setTargetPosition(intakepos[currentIntake]);
            }
            if(gamepad1.dpad_right){
                currentIntake--;
                if(currentIntake<0){
                    currentIntake=2;
                }
                Indexer.setTargetPosition(intakepos[currentIntake]);
            }






            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.

            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Indexer Position", "%d",
                    Indexer.getCurrentPosition());
            telemetry.update();
        }
    }

    // Dedicated method for the PID loop
    private void indexerPIDLoop() {
                double power = indexerPID.update(TargetPosition);
                Indexer.setPower(power);
            sleep(1);
    }
    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
