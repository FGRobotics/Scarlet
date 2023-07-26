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

@Autonomous(name="Left PARK",group="Linear OpMode")
public class bluePark extends LinearOpMode {

    private int parkingSpot = 1;
    private SampleMecanumDrive drive;


    private DcMotor lSlides, rSlides;
    private DcMotor LED;
    private Servo fBL, rightClaw, leftClaw, pusher;


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

        LED = hardwareMap.get(DcMotor.class, "yAxis");

        rSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        lSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");


        fBL = hardwareMap.get(Servo.class, "fourbar");
        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setPosition(1);

        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(0));

        //claw.setPosition(1);


        //Traj Seq
        TrajectorySequence start = drive.trajectorySequenceBuilder(startPose)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    pusher.setPosition(0);
                    fBL.setPosition(0.4);
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(1);
                        rSlides.setPower(1);


                        while(lSlides.getCurrentPosition() < 1625){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                }) // drops off

                .lineToConstantHeading(new Vector2d(-34,0))


                .waitSeconds(0.3)

                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    pusher.setPosition(0.5);
                    sleep(1000);
                    open();
                    sleep(200);
                }) // drops off
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    pusher.setPosition(0);

                })

                 */
                .lineToSplineHeading(new Pose2d(-33,-15,Math.toRadians(0)))
                /*
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
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
                */





                .build();



        //parking spot one

        Trajectory parktwo = drive.trajectoryBuilder(start.end())
                .lineToSplineHeading(new Pose2d(-36, -16, Math.toRadians(-90)))

//-36,-16


                .build();
        //parking spot two
        //JUST STAY PUT AT 36, -10
        //parking spot three
        Trajectory parkthree = drive.trajectoryBuilder(parktwo.end())
                .lineTo(new Vector2d(-13,-16))


                .build();
        Trajectory parkone = drive.trajectoryBuilder(parktwo.end())
                .lineTo(new Vector2d(-57,-16))


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
        new Thread(()->{
            if(parkingSpot == 1){
                for(int i = 1; i <= 1; i++){
                    LED.setPower(0.2);
                    sleep(1000);
                    LED.setPower(0);
                }
            }
            else if(parkingSpot==2){
                for(int i = 1; i <= 2; i++){
                    LED.setPower(0.2);
                    sleep(1000);
                    LED.setPower(0);
                }
            }
            else if(parkingSpot == 3){
                for(int i = 1; i <= 3; i++){
                    LED.setPower(0.2);
                    sleep(1000);
                    LED.setPower(0);
                }
            }
            else{
                for(int i = 1; i <= 5; i++){
                    LED.setPower(0.2);
                    sleep(1000);
                    LED.setPower(0);
                }
            }
        }).start();
        drive.setPoseEstimate(startPose);
        fBL.setPosition(.23);
        sleep(500);
        drive.followTrajectorySequence(start);


        /*
        Start: places on high
        cycle: places on medium
        cycle2: wants to place on low
        cycle3: places on medium (literally copy paste with slight modifications of cycle
         */
        if(parkingSpot == 1){
            drive.followTrajectory(parktwo);
            drive.followTrajectory(parkone);
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
            pusher.setPosition(1);
            fBL.setPosition(1);
        }
        if(parkingSpot == 2){
            drive.followTrajectory(parktwo);
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
            pusher.setPosition(1);
            fBL.setPosition(1);
        }
        if(parkingSpot ==3){
            drive.followTrajectory(parktwo);
            drive.followTrajectory(parkthree);
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
            pusher.setPosition(1);
            fBL.setPosition(1);

        }
        LED.setPower(1);








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