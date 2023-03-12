package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name="Red Right",group="Linear OpMode")
public class RedRight extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(0));

        //claw.setPosition(1);


        //Traj Seq
        TrajectorySequence start = drive.trajectorySequenceBuilder(new Pose2d(36, -64, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(36, -48, Math.toRadians(0)))//first pole-small
                .lineToSplineHeading(new Pose2d(31, -48, Math.toRadians(0)))



                .waitSeconds(0.5)

                .addDisplacementMarker(()->{
                    open();
                    sleep(200);
                    fBL.setPosition(0.5);
                }) // drops off

                .lineToSplineHeading(new Pose2d(36, -48, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(36, -32, Math.toRadians(0)))




                .waitSeconds(0.5)
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(start.end())
                .lineToSplineHeading(new Pose2d(36, -12.5, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(36, -25), ()->
                        fBL.setPosition(.88)
                ) // flips over for pickup
                .addSpatialMarker(new Vector2d(38, -12.5), ()->open())
                .lineTo(
                        new Vector2d(39.8,-12.5),
                        SampleMecanumDrive.getVelocityConstraint(12 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//pickup
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-1.0,()->{
                    Subs highPole = new Subs(lSlides, rSlides, 650);
                    highPole.start();
                    fBL.setPosition(0.51);

                })




                .addTemporalMarker(8, ()->open())
                .lineToSplineHeading(new Pose2d(26.5, -13, Math.toRadians(-90)),
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
        Trajectory parkone = drive.trajectoryBuilder(start.end())
                .lineTo(new Vector2d(15,-32))
                .build();

        //parking spot two
        //JUST STAY PUT AT 36, -10
        //parking spot three
        Trajectory parkthree = drive.trajectoryBuilder(start.end())
                .lineTo(new Vector2d(60,-32))
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
        fBL.setPosition(.23);

        sleep(500);
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(start);
        //drive.followTrajectorySequence(cycle);
        close();
        sleep(200);
        fBL.setPosition(0.3);

        //Subs down = new Subs(lSlides, rSlides, 0);
        //down.start();

        if(parkingSpot == 1){
            drive.followTrajectory(parkone);
        }

        if(parkingSpot ==3){
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
        leftClaw.setPosition(-0.2);
    }
}