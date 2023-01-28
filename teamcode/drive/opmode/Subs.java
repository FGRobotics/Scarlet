package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Subs extends Thread {
    private DcMotor lSlides;
    private DcMotor rSlides;
    private int target;

    public Subs(DcMotor lSlides, DcMotor rSlides, int target){
        this.lSlides= lSlides;
        this.rSlides = rSlides;
        this.target = target;

    }
    public void run(){
        lSlides.setTargetPosition(target);
        rSlides.setTargetPosition(target);


        lSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lSlides.setPower(1);
        rSlides.setPower(1);


        while(lSlides.isBusy()){
                continue;
        }
        lSlides.setPower(0);
        rSlides.setPower(0);


    }
}