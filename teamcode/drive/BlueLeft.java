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

@Autonomous(name="Left",group="Linear OpMode")
public class BlueLeft extends LinearOpMode {

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
        pusher.setPosition(0);

        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(0));

        //claw.setPosition(1);


        //Traj Seq
        TrajectorySequence start = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    fBL.setPosition(0.4);
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(1);
                        rSlides.setPower(1);


                        while(lSlides.getCurrentPosition() < 2000){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                }) // drops off
                .lineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(52 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))//first pole-high



                .waitSeconds(0.3)

                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    pusher.setPosition(0.5);
                    sleep(1000);
                    open();
                    sleep(200);
                }) // drops off
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-33,-15,Math.toRadians(0)))




                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(start.end())
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
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
                    open();
                })
                .lineToSplineHeading(new Pose2d(-36, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(52 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //.addSpatialMarker(new Vector2d(-38, -13), ()->open())
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                    open();
                    fBL.setPosition(.83); //was .85
                    pusher.setPosition(1);

                })
                .lineTo(
                        new Vector2d(-52,-15),
                        SampleMecanumDrive.getVelocityConstraint(12 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//pickup
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0,()->close())


                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(1);
                        rSlides.setPower(1);


                        while(lSlides.getCurrentPosition() < 500){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                    fBL.setPosition(0.38);
                    pusher.setPosition(1);


                })
                .waitSeconds(1)


                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
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
                    pusher.setPosition(0);
                })

                .lineToSplineHeading(new Pose2d(-23, -15, Math.toRadians(-90)),
                    SampleMecanumDrive.getVelocityConstraint(52, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(25))//pickup))//second pole-medium
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{

                    pusher.setPosition(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->open())
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0,()-> pusher.setPosition(0))


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


        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(cycle.end())
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
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
                    open();
                })
                .lineToSplineHeading(new Pose2d(-36, -13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //.addSpatialMarker(new Vector2d(-38, -13), ()->open())
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                    open();
                    fBL.setPosition(.85);
                    pusher.setPosition(1);

                })
                .lineTo(
                        new Vector2d(-52,-12),
                        SampleMecanumDrive.getVelocityConstraint(10 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//pickup
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0,()->close())
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(1);
                        rSlides.setPower(1);


                        while(lSlides.getCurrentPosition() < 400){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                    fBL.setPosition(0.5);
                    pusher.setPosition(0);


                })
                .waitSeconds(1)


                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
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
                    pusher.setPosition(0);
                })

                .lineToSplineHeading(new Pose2d(-40, -15, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(52, Math.toRadians(100), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))//pickup))//second pole-medium
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    fBL.setPosition(0);
                    pusher.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->open())
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    close();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> fBL.setPosition(0.5))



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

        TrajectorySequence cycle3 = drive.trajectorySequenceBuilder(cycle.end())
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(-1);
                        rSlides.setPower(-1);


                        while(lSlides.getCurrentPosition() > 200){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                    open();
                })
                .lineToSplineHeading(new Pose2d(-36, -15, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(52 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))

                //.addSpatialMarker(new Vector2d(-38, -13), ()->open())
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                    open();
                    fBL.setPosition(.87); //was .81
                    pusher.setPosition(1);

                })
                .lineTo(
                        new Vector2d(-52,-16),
                        SampleMecanumDrive.getVelocityConstraint(12 , DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))//pickup
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0,()->close())


                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    new Thread(()->{


                        lSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        lSlides.setPower(1);
                        rSlides.setPower(1);


                        while(lSlides.getCurrentPosition() < 200){
                            continue;
                        }
                        lSlides.setPower(0);
                        rSlides.setPower(0);
                    }).start();
                    fBL.setPosition(0.36);
                    pusher.setPosition(0);


                })
                .waitSeconds(1)






                .lineToSplineHeading(new Pose2d(-23, -18, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(52, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))//pickup))//second pole-medium

                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{

                    pusher.setPosition(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->open())
                .waitSeconds(1)
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
                    pusher.setPosition(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(0,()-> pusher.setPosition(0))
                .waitSeconds(0.5)


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

        Trajectory parktwo = drive.trajectoryBuilder(cycle3.end())
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
        drive.followTrajectorySequence(cycle);
        drive.followTrajectorySequence(cycle3);

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