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

@Autonomous(name="Blue Right",group="Linear OpMode")
public class BlueRight extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(36, -63, Math.toRadians(0));//x: -64

        //claw.setPosition(1);


        //Traj Seq
        TrajectorySequence start = drive.trajectorySequenceBuilder(new Pose2d(36, -64, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(36, -22, Math.toRadians(0)))//first pole-small
                //.lineToSplineHeading(new Pose2d(35, -24, Math.toRadians(0)))



                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->open())
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->fBL.setPosition(0.5))





                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(start.end())
                .lineToSplineHeading(new Pose2d(37, -11.6, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->fBL.setPosition(0.88))

                .lineToSplineHeading(new Pose2d(55, -11.6, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->close())
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(1);
                        rSlides.setPower(1);


                        while(lSlides.getCurrentPosition() < 800){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                })
                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->fBL.setPosition(0.53))
                .lineToSplineHeading(new Pose2d(36, -25, Math.toRadians(0)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
                    open();
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(-1);
                        rSlides.setPower(-1);


                        while(lSlides.getCurrentPosition() > 0){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->fBL.setPosition(0.5))





                .build();
        //second pickup
        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(cycle.end())
                .lineToSplineHeading(new Pose2d(37, -11.6, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> fBL.setPosition(0.9))

                .lineToSplineHeading(new Pose2d(55, -11.6, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->close())
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{

                        new Thread(()->{


                            lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                            lSlides.setPower(1);
                            rSlides.setPower(1);


                            while(lSlides.getCurrentPosition() < 1300){
                                continue;
                            }
                            lSlides.setPower(0);
                            rSlides.setPower(0);
                        }).start();
                })
                .lineToSplineHeading(new Pose2d(14, -12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->fBL.setPosition(0.5))
                .lineToSplineHeading(new Pose2d(14, -24, Math.toRadians(0)))

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
                    open();


                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->close())
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->fBL.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(-1);
                        rSlides.setPower(-1);


                        while(lSlides.getCurrentPosition() > 0){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                })





                .build();




        //parking spot one
        Trajectory parkone = drive.trajectoryBuilder(cycle2.end())
                .lineTo(new Vector2d(14,-39))
                .build();
        Trajectory parktwo = drive.trajectoryBuilder(cycle2.end())
                .lineTo(new Vector2d(38,-39))
                .build();
        //parking spot two
        //JUST STAY PUT AT 36, -10
        //parking spot three
        Trajectory parkthree = drive.trajectoryBuilder(cycle2.end())
                .lineTo(new Vector2d(64,-39))
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
        fBL.setPosition(.5);

        sleep(500);
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(start);
        drive.followTrajectorySequence(cycle);
        drive.followTrajectorySequence(cycle2);


        close();
        sleep(200);


        if(parkingSpot == 1){
            drive.followTrajectory(parkone);
        }
        if(parkingSpot == 2){
            drive.followTrajectory(parkone);
            drive.followTrajectory(parktwo);
        }
        if(parkingSpot ==3){
            drive.followTrajectory(parkone);
            drive.followTrajectory(parkthree);
        }
        open();
        fBL.setPosition(0.3);
        sleep(2000);






    }
    public void open(){
        rightClaw.setPosition(.5);
        leftClaw.setPosition((.3));


    }
    public void close(){
        rightClaw.setPosition(1);
        leftClaw.setPosition(-0.1);
    }
}