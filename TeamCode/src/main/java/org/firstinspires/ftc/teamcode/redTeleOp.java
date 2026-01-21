package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
@TeleOp(name = "Red turret teleop")
public class redTeleOp extends LinearOpMode {
    // Dc Motor dec
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Intake = null;
    private DcMotor Indexer = null;
    private DcMotorEx Flywheel = null;
    private DcMotorEx turret = null;
    private Servo leftKicker = null;
    private Servo rightKicker = null;

    private Servo Pitch = null;
    Limelight3A limelight;

    private GoBildaPinpointDriver pinpoint = null;
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
    // constants for indexer
    private ColorSensorV3 colorSensor;

    // declared like this so if motors are swapped I only have to find one value
    int TargetPosition = ballOne;
    int Intakepos = ballOne;
    int Shootingpos = ballOne + 270;
    private PIDControllerRyan indexerPID = null;
    private PIDControllerRyan turretPID = null;
    private PIDTuner pidTuner = null;


    static double TargetX = 0;// find this

    double TargetAngle = 0;

    public static double VF = 0;
    int targetID = 0;
    double tx = 0;
    double ty = 0;

    double ta = 0;
    private final int turretmaxl = 1087;
    private final int turretmaxr = -538;

    private final int offset = 202;

    private double tpr = 537.7;

    private double x = 0;
    private double y = 0;
    private double heading = 0;

    Pose2D currentPose = new Pose2D(DistanceUnit.INCH,96, 8, AngleUnit.DEGREES,90);//used to save position after autonomous
    boolean autoIntake = false;
    private double distanceToTarget;
    private double targetx = 144;
    private double targety = 144;
    private double xV;
    private double yV;
    private double netV;
    private double hV;
    private double targetangle;
    private double relTargetangle;
    private double TURRET_TICKS_PER_RADIAN = 537.7/(Math.PI*2);

    private int turretTargetPosition = 0;

    @Override
    public void runOpMode() {

        // Lime Light
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);

        //imu
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(3.425,6.424,DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(currentPose);
        
        // base
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        Indexer = hardwareMap.get(DcMotor.class, "Indexer");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        rightKicker = hardwareMap.get(Servo.class, "rightKicker");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        Indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftKicker.setDirection(Servo.Direction.FORWARD);
        rightKicker.setDirection(Servo.Direction.FORWARD);
        Pitch = hardwareMap.get(Servo.class, "Pitch");
        leftKicker.setDirection(Servo.Direction.FORWARD);// 0 is min angle 1 is max angle
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // intake
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftKicker.setPosition(1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        indexerPID = new PIDControllerRyan(0.005, 0.000, 0.000, 0, Indexer);
        turretPID = new PIDControllerRyan(0.008, 0.000, 0.000, 0, turret);
        pidTuner = new PIDTuner(indexerPID, gamepad2, telemetry);
        Thread KickerThread = new Thread(this::Kick);
        Thread PintpointThread = new Thread(this:: PinpointLoop);
        waitForStart();
        KickerThread.start();
        //PintpointThread.start();

        while (opModeIsActive()) { // Loop
            pinpoint.update();
            x = pinpoint.getPosition().getX(DistanceUnit.INCH);
            y = pinpoint.getPosition().getY(DistanceUnit.INCH);
            heading = pinpoint.getHeading(AngleUnit.RADIANS);
            distanceToTarget = Math.sqrt(Math.pow(x - targetx, 2) + Math.pow(y - targety, 2));
            Calculate(distanceToTarget);
            xV = pinpoint.getVelX(DistanceUnit.INCH);
            yV = pinpoint.getVelY(DistanceUnit.INCH);
            netV = Math.sqrt(Math.pow(xV, 2) + Math.pow(yV, 2));
            hV = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            targetangle = Math.atan2(targety - y, targetx - x);
            relTargetangle = targetangle - heading;
            relTargetangle = Math.atan2(Math.sin(relTargetangle), Math.cos(relTargetangle));

// Shift so forward = pi/2
            double turretAngle = relTargetangle + (Math.PI / 2.0);

// Clamp to turret range
            turretAngle = Math.max(0.0, Math.min(Math.PI, turretAngle));

// Convert to ticks
            turretTargetPosition = (int)(turretAngle * TURRET_TICKS_PER_RADIAN*4)+40;
            // Clamp the target position to within the physical limits of the turret
            turretTargetPosition = Math.max(turretmaxr,
                    Math.min(turretmaxl, turretTargetPosition));


            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to
            // rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3); // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
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
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }
            if(gamepad1.yWasPressed()){
                Pitch.setPosition(0);
            }
            if(gamepad1.bWasPressed()){
                Pitch.setPosition(1);
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Intake
            if (gamepad1.right_trigger > 0.01) {
                Intake.setPower(1);
            } else {
                Intake.setPower(0);
            }
            if (gamepad1.dpadUpWasPressed()) {
                shoot();
            }
            if (gamepad1.dpadRightWasPressed()) {
                intake();
            }
            if(gamepad2.rightBumperWasPressed()){
                ballOne+=10;
                ballTwo+=10;
                ballThree+=10;
                TargetPosition+=10;
            }
            if(gamepad2.leftBumperWasPressed()){
                ballOne-=10;
                ballTwo-=10;
                ballThree-=10;
                TargetPosition-=10;
            }
            if(distanceToTarget>125){
                Pitch.setPosition(0);
            }
            else{
                Pitch.setPosition(1);
            }


            PIDLoop();
            turretPID();
            Flywheel.setVelocity(VF);


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
            // colorSensor.addTelemetry();
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("H", heading);
            telemetry.addData("distance to target",distanceToTarget);
            telemetry.addData("Target Turret", turretTargetPosition);
            telemetry.addData("Turret pos", turret.getCurrentPosition());
            telemetry.addData("Ball one pos", ballOne);
            telemetry.addData("Ball two pos ", ballTwo);
            telemetry.addData("Ball three pos", ballThree);
            telemetry.addData("Shooting Position", Shootingpos);
            telemetry.addData("Intake Position", Intakepos);
            telemetry.addData("Current Order", targetOrder[0]);
            telemetry.addData("Current Order", targetOrder[1]);
            telemetry.addData("Current Order", targetOrder[2]);
            telemetry.addData("Flywheel Target RPM", VF);
            telemetry.addData("distance", TargetX);
            telemetry.addData("ty", ty);
            telemetry.addData("tx", tx);
            telemetry.addData("ta", ta);
            telemetry.addData("ll status", limelight.isConnected());
            telemetry.update();
        }
    }

    public void PinpointLoop(){
        while (opModeIsActive()) {
            pinpoint.update();
            sleep(50);
        }

    }
    private void turretPID(){
        double inpower = turretPID.update(turretTargetPosition); // Use the D-pad updated TargetPosition
        turret.setPower(inpower);
    }
    public void Calculate(double dih){
        VF = 5.11095 * dih + 631.27328;
    }
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
    // Dedicated method for the Limlight;



    private void Kick() {
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                leftKicker.setPosition(1);
                sleep(800);
                leftKicker.setPosition(0);
            }
            sleep(50);
        }
    }



    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
