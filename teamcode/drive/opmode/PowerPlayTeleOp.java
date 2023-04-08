package org.firstinspires.ftc.teamcode.drive.opmode;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="power-play-tele_op")
public class PowerPlayTeleOp extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront, yAxis, xAxis;
    private DcMotor lSlides, rSlides; //joey was here
    private List<DcMotorEx> motors;
    private boolean isOpen = false;
    boolean tCancel = false;



    private Servo fBL, rightClaw, leftClaw, pusher;
    private DistanceSensor frontDist;

    private int upperBound = 2400;
    private int lowerBound = 0;


    private boolean isAlive = false;
    private int fBLpos = 0;
    private double[] dropPositions = {1,0.9 ,0.4, 0.2, 0, 0.45};
    private int polePos = 0;
    private double actfieldCentricMultiplier = .7;
    private double fieldCentricMultiplier = .7;
    private double rotationalMult = .6;
    double lastResetpos = 0.0;
    //private int normTurnTable = turnTable.getCurrentPosition();
    private CRServo one;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //Brake behavier
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        rSlides = hardwareMap.get(DcMotor.class, "rSlides");
        lSlides = hardwareMap.get(DcMotor.class, "lSlides");

        rSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        lSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        fBL = hardwareMap.get(Servo.class, "fourbar");
        pusher = hardwareMap.get(Servo.class, "pusher");
        //fBL.setPosition(0);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");



        int count = 0;
        //double botHeading = -imu.getAngularOrientation().firstAngle;
        waitForStart();



        //button list:
        //  Gamepad 1 -
        //      dpad left/right - cone alignment
        //      left joystick x/y - field centric lateral movement
        //      right joystick x - rotation
        //      circle - invert field centric
        //      cross - set the robot's 0 heading(straight forward) to where the robot is pointing right now
        //      triangle - set the robot's 0 heading back to it's original place
        //  Gamepad 2 -
        //      dpad left/right - pole alignment;

        while (opModeIsActive()) {


            fieldCentricPlus();
            gp1();
            gp2();
            telemetry.addData("pos: ", lSlides.getCurrentPosition());
            telemetry.addData("fbl pos: ", fBL.getPosition());
            telemetry.update();

            //Normal(from -1120 to 1120) turn table ticks
            //normTurnTable();


        }

    }

    public void gp1() {
        int readDist = 7;

        fieldCentricMultiplier = (gamepad1.right_trigger/actfieldCentricMultiplier)*Math.abs(actfieldCentricMultiplier);


        //invert field centric driving;
        if (gamepad1.circle) {
            actfieldCentricMultiplier = (actfieldCentricMultiplier == -0.7) ? 0.7 : -0.7;

        }

        //reset IMU position to zero
        if (gamepad1.cross) {
            double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            heading = (heading < 0) ? Math.toRadians(360) + heading : heading;
            lastResetpos = heading;

        }

        //reset IMU position to original start
        if (gamepad1.triangle) {
            lastResetpos = 0;
        }

        if (frontDist.getDistance(DistanceUnit.INCH) < 6){
            gamepad1.rumble(1000);
            gamepad1.setLedColor(1,1.5,9,1000);
        }


        if(gamepad1.square){
            rotationalMult = (rotationalMult == 0.9) ? 0.2 : 0.9;

        }


        if(gamepad1.dpad_left){
            new Thread(()->{
                while((frontDist.getDistance(DistanceUnit.INCH) > readDist) && !tCancel) {
                    turnLeft(0.6);
                    gamepad1.setLedColor(1,0,0,250);

                    if(gamepad1.dpad_down){
                        tCancel = true;
                    }
                }


            }).start();
        }
        gamepad1.stopRumble();

        tCancel = false;

        if(gamepad1.dpad_right){
            new Thread(()->{
                while((frontDist.getDistance(DistanceUnit.INCH) > readDist) && !tCancel){
                    turnRight(0.6);
                    if(gamepad1.dpad_down){
                        tCancel = true;
                    }
                }

            }).start();
        }



    }
    public void turnLeft(double power){
        leftRear.setPower(power);
        leftFront.setPower(power);
        rightRear.setPower(-power);
        rightFront.setPower(-power);
    }
    public void turnRight(double power){
        leftRear.setPower(-power);
        leftFront.setPower(-power);
        rightRear.setPower(power);
        rightFront.setPower(power);
    }

    public void gp2() {

        //auto bsck drop
        if(gamepad2.dpad_up){
            new Thread(()->{
                close();
                fBL.setPosition(0.2);
                lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                lSlides.setPower(1);
                rSlides.setPower(1);

                while(lSlides.getCurrentPosition() < 2100){
                    continue;
                }
                lSlides.setPower(0);
                rSlides.setPower(0);
            }).start();
        }
        if(gamepad2.dpad_right){
            new Thread(()->{
                close();
                fBL.setPosition(0.2);
                lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                lSlides.setPower(1);
                rSlides.setPower(1);

                while(lSlides.getCurrentPosition() < 1100){
                    continue;
                }
                lSlides.setPower(0);
                rSlides.setPower(0);
            }).start();
        }


        //Claw
        if(gamepad2.left_bumper){
            if(!isOpen) {
                new Thread(()->{
                    open();
                }).start();
            } else{
                new Thread(()->{
                    close();
                }).start();
            }
        }

        //slides
        //lSlides.setPower(gamepad2.left_stick_y);
        //rSlides.setPower(gamepad2.left_stick_y);

        //Intake Position
        if(gamepad2.cross){
            new Thread(()->{
                close();
                pusher.setPosition(1);
                fBL.setPosition(1);
            }).start();
        }
        //front drop
        if(gamepad2.square){
            new Thread(()->{
                fBLpos = 2;


                close();
                fBL.setPosition(dropPositions[fBLpos]);
                sleep(350);
                pusher.setPosition(0.8); //was 0.8
            }).start();
        }
        //back setup
        if(gamepad2.circle){
            new Thread(()->{
                fBLpos = 3;
                close();
                fBL.setPosition(dropPositions[fBLpos]);
                sleep(350);
                pusher.setPosition(0);
            }).start();
        }
        //back drop
        if(gamepad2.triangle){
            new Thread(()->{
                fBLpos = 4;
                close();
                fBL.setPosition(dropPositions[fBLpos]);
                sleep(350);
                pusher.setPosition(0);
            }).start();
        }
        //vertical
        if(gamepad2.right_bumper){
            new Thread(()->{
                fBLpos = 5;
                close();
                fBL.setPosition(dropPositions[fBLpos]);
                sleep(350);
                pusher.setPosition(0);
            }).start();
        }


        slides();




    }




    public void fieldCentricPlus() {
        double x, y, mag, rads, rangle;

        fieldCentricMultiplier = (fieldCentricMultiplier == 0) ? actfieldCentricMultiplier : fieldCentricMultiplier;
        x = fieldCentricMultiplier * (gamepad1.left_stick_x);
        y = fieldCentricMultiplier * (gamepad1.left_stick_y);

        double rotational = gamepad1.right_stick_x * rotationalMult;

        //find the resultant vector of the joystick
        mag = Math.pow(x, 2) + Math.pow(y, 2);
        mag = Math.sqrt(mag);
        //angle of the vector
        rads = Math.atan2(y, x);

        rads = (rads >= 0 && rads < Math.toRadians(270)) ? (-1 * rads) + Math.toRadians(90) : (-1 * rads) + Math.toRadians(450);
        //makes joystick angles go from 0(which is pointing the joystick straight up) to 180 back down to 0

        //find the robot angle
        rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        //find angle robot needs to turn in order to reach joystick angle


        //normalizes two angles
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;
        //subtracts reset position
        rads = rads - lastResetpos;
        rangle = rangle - lastResetpos;
        //turns those back positive
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;
        //calculates the distance between joystick and heading as a positive int
        //double turn = (rads < rangle) ? (Math.toRadians(360) - rangle) + (Math.abs(0 - rads)) : rads - rangle;
        double turn = rads;
        double equationone = (Math.sin(turn + (Math.PI / 4)) * mag);
        double equationtwo = (Math.sin(turn - (Math.PI / 4)) * mag);

        /*
        telemetry.addData("eq w o mag:", (Math.sin(turn - (14.318))) );
        telemetry.addData("mag:", mag);
        telemetry.addData("eq 1:", equationone);
        telemetry.update();
         */

        rightFront.setPower((equationone + rotational));
        leftRear.setPower((equationone - rotational));
        rightRear.setPower((-equationtwo + rotational));
        leftFront.setPower((-equationtwo - rotational));


    }
    public void open(){
        rightClaw.setPosition(.5);
        leftClaw.setPosition((.3));
        sleep(500);
        isOpen = true;

    }
    public void close(){
        rightClaw.setPosition(.9);
        leftClaw.setPosition(0);
        sleep(500);
        isOpen=false;
    }

    public void slides(){
        lSlides.setPower( (lSlides.getCurrentPosition() >= lowerBound) ? -gamepad2.left_stick_y : 0);
        rSlides.setPower( (lSlides.getCurrentPosition() >= lowerBound) ? -gamepad2.left_stick_y : 0);

        if(lSlides.getCurrentPosition() < 0  && lSlides.getPower() == 0 ){
            lowerBound = lSlides.getCurrentPosition();
        }

    }
}