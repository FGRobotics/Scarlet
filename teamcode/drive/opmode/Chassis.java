package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
@TeleOp(name="chassis")
public class Chassis extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private int fieldCentricMultiplier = 1;
    private BNO055IMU imu;
    private double restrictor = 1;

    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.initialize(parameters);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.circle){
                imu.initialize(parameters);
                telemetry.addData("imu angle: ", imu.getAngularOrientation().firstAngle);
                telemetry.update();

            }
            //fieldCentricPlus();
            gm0FCP();


        }
    }
    public void gm0FCP(){
        double y = (-gamepad1.left_stick_y); // Remember, this is reversed!
        double x = (gamepad1.left_stick_x * 1.1); // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * -0.8;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = (-imu.getAngularOrientation().firstAngle);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    public void fieldCentricPlus() {
        double x, y, mag, rads, rangle;


        x = fieldCentricMultiplier * (-gamepad1.left_stick_x);
        y = fieldCentricMultiplier * (-gamepad1.left_stick_y);

        double rotational = gamepad1.right_stick_x;

        //find the resultant vector of the joystick
        mag = Math.pow(x, 2) + Math.pow(y, 2);
        mag = Math.sqrt(mag);
        //angle of the vector
        rads = Math.atan2(y, x);

        rads = (rads >= 0 && rads < Math.toRadians(270) ) ? (-1*rads) + Math.toRadians(90) : (-1*rads) + Math.toRadians(450);
        //makes joystick angles go from 0(which is pointing the joystick straight up) to 180 back down to 0

        //find the robot angle
        rangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        //find angle robot needs to turn in order to reach joystick angle


        //normalizes two angles
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;
        //subtracts reset position

        //rads = rads -  lastResetpos;
        //rangle = rangle - lastResetpos;

        //turns those back positive

        //rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        //rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;

        //calculates the distance between joystick and heading as a positive int
        double turn =  (rads < rangle) ? (Math.toRadians(360)-rangle) + (Math.abs(0-rads)) : rads - rangle;

        double equationone = (Math.sin(turn + Math.PI/4) * mag);
        double equationtwo = (Math.sin(turn - Math.PI/4) * mag);

        /*
        telemetry.addData("eq w o mag:", (Math.sin(turn - (14.318))) );
        telemetry.addData("mag:", mag);
        telemetry.addData("eq 1:", equationone);
        telemetry.update();
         */

        rightFront.setPower(equationone + rotational);
        leftRear.setPower(equationone - rotational);
        rightRear.setPower(-equationtwo + rotational);
        leftFront.setPower(-equationtwo - rotational);


    }
}
