package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;
import java.util.List;

@Autonomous(name="Red Left",group="Linear OpMode")
public class BlueLeft extends LinearOpMode {

    private int parkingSpot = 1;
    private SampleMecanumDrive drive;


    private DcMotor lSlides, rSlides;
    private Servo fBL, rightClaw, leftClaw;


    private AnalogInput turnTableEncoder;

    //private List<DcMotorEx> motors;
    enum State {
        CYCLING,
        PARKING,
        IDLE
    }
    private State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        drive = new SampleMecanumDrive(hardwareMap);
        //turnTable = hardwareMap.get(DcMotor.class, "turnTable");
        rSlides = hardwareMap.get(DcMotor.class, "rSlides");
        lSlides = hardwareMap.get(DcMotor.class, "lSlides");

        rSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        lSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");


        fBL = hardwareMap.get(Servo.class, "fourbar");
        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(0));

        //claw.setPosition(1);


        //Traj Seq
        TrajectorySequence start = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, -24, Math.toRadians(0)))//first pole-small
                .lineToSplineHeading(new Pose2d(-42, -24, Math.toRadians(0)))



                .waitSeconds(0.5)

                .addDisplacementMarker(()->{
                    open();
                    sleep(200);
                    fBL.setPosition(0.3);
                }) // drops off

                .lineToSplineHeading(new Pose2d(-36, -24, Math.toRadians(0)))


                .waitSeconds(0.5)
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(start.end())
                .lineToSplineHeading(new Pose2d(36, -13, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(36, -25), ()->
                        fBL.setPosition(.9)
                ) // flips over for pickup
                .addSpatialMarker(new Vector2d(38, -13), ()->open())
                .lineTo(
                        new Vector2d(40,-13),
                        SampleMecanumDrive.getVelocityConstraint(12 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//pickup
                .waitSeconds(1.5)
                .addDisplacementMarker(()->{
                    Subs highPole = new Subs(lSlides, rSlides, 650);
                    highPole.start();
                    fBL.setPosition(0.51);

                })




                .addTemporalMarker(8, ()->open())
                .lineToSplineHeading(new Pose2d(26, -13, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//pickup))//second pole-medium
                .waitSeconds(1.5)
                // .waitSeconds(2)
                /*
                .lineToSplineHeading(new Pose2d(58, -12, Math.toRadians(0)))//pickup
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(24, -12, Math.toRadians(-90)))//third pole-medium
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(58, -12, Math.toRadians(0)))//pickup
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(0, -12, Math.toRadians(-90)))//fourth pole-high
                .waitSeconds(2)
                */

                //back 25 for parking spot 1
                //foward 25 for parking spot 3
                //stay put for parking spot 2
                //.forward(25)
                .build();

        //parking spot one

        Trajectory parktwo = drive.trajectoryBuilder(start.end())
                .lineTo(new Vector2d(-36,-32))

                .build();
        //parking spot two
        //JUST STAY PUT AT 36, -10
        //parking spot three
        Trajectory parkthree = drive.trajectoryBuilder(parktwo.end())
                .lineTo(new Vector2d(-13,-32))


                .build();
        Trajectory parkone = drive.trajectoryBuilder(parktwo.end())
                .lineTo(new Vector2d(-57,-32))


                .build();
        //THIS CODE RUNS CONTOUR PIPELINE
        //JUST INITIALIZES EVERYTHING SHOULD BE EASY


        OpenCvWebcam webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        //OpenCV Pipeline

        ContourPipeline myPipeline = new ContourPipeline();
        webcam.setPipeline(myPipeline);

        OpenCvWebcam finalWebcam = webcam;
        finalWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Initialization passed ", 1);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Init Failed ", errorCode);
                telemetry.update();

            }

        });


        while (!opModeIsActive()) {
            close();
            parkingSpot = myPipeline.parkingSpot;
            telemetry.addData("parking spot: ", parkingSpot);
            telemetry.update();

        }



        waitForStart();
        if (isStopRequested()) return;
        close();
        drive.setPoseEstimate(startPose);
        fBL.setPosition(.23);
        sleep(500);
        drive.followTrajectorySequence(start);

        if(parkingSpot == 1){
            drive.followTrajectory(parktwo);
            drive.followTrajectory(parkone);
        }
        if(parkingSpot == 2){
            drive.followTrajectory(parktwo);
        }
        if(parkingSpot ==3){
            drive.followTrajectory(parktwo);
            drive.followTrajectory(parkthree);
        }
        sleep(5000);






    }
    public void open(){
        rightClaw.setPosition(.5);
        leftClaw.setPosition((.3));
        sleep(1000);
        close();
    }
    public void close(){
        rightClaw.setPosition(1);
        leftClaw.setPosition(-0.1);
    }
}