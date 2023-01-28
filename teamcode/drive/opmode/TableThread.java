package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TableThread extends Thread {
    private DcMotor turnTable;

    private int ticks;

    public TableThread (DcMotor turnTable, int ticks){
        this.turnTable = turnTable;
        this.ticks = ticks;
    }
    public void run(){
        turnTable.setTargetPosition(ticks);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setPower(-0.6);
        while(turnTable.isBusy()){
            continue;
        }
        turnTable.setPower(0);
    }
}