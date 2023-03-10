package org.firstinspires.ftc.teamcode.drive.opmode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {

    int parkingSpot;


    @Override
    public Mat processFrame(Mat input) {

        //entire thing has to be in a try catch other wise if a contour is not found it will throw and error and stop running
        Mat end = new Mat();
        try {
            end = input;

            Mat src = input;

            //Freight Frenzy CV goal- to detect a custom team shipping element and determine which tape mark it is at- the left, middle, or right


            //this is YCRCB - Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
            //this is YCRCB - Scalar scalarUpperYCrCb = new Scalar(255.0, 100.0, 170.0);

            //those are hard to figure out ^^^^^^^^^^^ but they are in the Y CR CB color space

            //forming a color range
            //pick a range of HSV colors that aren't too specific nor general- they should contain the color of object you are trying to find in the range


            Scalar lowerHSV = new Scalar(40.0, 40.0, 30.0);//HSV for now
            Scalar upperHSV = new Scalar(80.0, 255.0, 255.0);//HSV for now

            //Those are the boundaries of the accepted colors in HSV

            //open cv- Hue goes to 179, the other two go to 255
            //google - Hue goes to 360, the other two are percentages out of 100%

            //I found that translating them doesn't really work so I just made an easier range finder: https://tejusk2.github.io/FTCVISION/


            //gets the image ready for contour detection


            //Converts space to HSV
            Imgproc.cvtColor(src, src, Imgproc.COLOR_RGB2HSV);
            //filters out all colors not in this range
            Core.inRange(src, lowerHSV, upperHSV, src);
            // Remove Noise
            //choose one or the other or they cancel things out, I AM USING CLOSE and it is being used in the range finder

            Imgproc.morphologyEx(src, src, Imgproc.MORPH_OPEN, new Mat());
            // GaussianBlur
            Imgproc.blur(src, src, new Size(10, 10));


            //Finding Contours

            ArrayList<MatOfPoint> contours = new ArrayList<>();

            Mat hierarchey = new Mat();

            //finds contour
            Imgproc.findContours(src, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //Now that you have the arrayList of contours, it is up to you to do the rest, the rest of this code is specific to FREIGHT FRENZY
            //check out the drawing rectangles code below though


            //sorts through and finds largest one



            //Draw rectangle on largest contours

            //Drawing rectangles is actually pretty important I suggest that you learn this
            parkingSpot = 0;
            for(MatOfPoint contour : contours) {

                MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());


                Rect rect = Imgproc.boundingRect(areaPoints);
                double cntarea = Imgproc.contourArea(contour);
                double rectarea = rect.area();

                double extent = cntarea / rectarea;

                if(rect.width >= 100 && rect.width <= 300 && extent >= 0.86) {
                    Imgproc.putText(end, String.valueOf(extent), rect.tl(), 1,6, new Scalar(255,0,0));
                    Imgproc.rectangle(end, rect, new Scalar(255,0,0));
                    parkingSpot++;
                }

            }


        }catch (IndexOutOfBoundsException error){

        }
        return end;
    }
    public int getParkingSpot(){
        return parkingSpot;
    }


}