package org.firstinspires.ftc.teamcode.auto;
//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ColorPipeline extends OpenCvPipeline {
    boolean viewportPaused;
    public String ColorValue = "";
    private OpenCvWebcam webcam;
    private int TopLeftX = 0;
    private int TopLeftY = 0;
    private int Width = 0;
    private int Height = 0;
    public double totalRed, totalGreen, totalBlue, pixelRed, getPixelGreen, getPixelBlue;

    public ColorPipeline(OpenCvWebcam AOECam, int TopLeftX, int TopLeftY, int Width, int Height) {
        this.webcam = webcam;
        this.TopLeftX = TopLeftX;
        this.TopLeftY = TopLeftY;
        this.Width = Width;
        this.Height = Height;
    }

    @Override
    public Mat processFrame(Mat input) {
        double red = 0.0D;
        double green = 0.0D;
        double blue = 0.0D;
            for(int i = this.TopLeftX; i < this.TopLeftX + this.Width; ++i) {
                for(int j = this.TopLeftY; j < this.TopLeftY + this.Height; ++j) {
                    red += input.get(i, j)[0];
                    blue += input.get(i, j)[2];
                    green += input.get(i, j)[1];
                }
            }

            if (red > green && red > blue) {
                this.ColorValue = "red";
            } else if (green > red && green > blue) {
                this.ColorValue = "green";
            } else if (blue > red && blue > green) {
                this.ColorValue = "blue";
            }
            totalRed = red;
            totalGreen = green;
            totalBlue = blue;

            pixelRed = input.get(0,0)[0];
            getPixelGreen = input.get(0,0)[1];
            getPixelBlue = input.get(0,0)[2];

            Imgproc.rectangle(input, new Point((double)this.TopLeftX, (double)this.TopLeftY), new Point((double)(this.TopLeftX + this.Width), (double)(this.TopLeftY + this.Height)), new Scalar(0.0D, 255.0D, 0.0D), 4);

        return input;
    }


    public void onViewportTapped() {
        this.viewportPaused = !this.viewportPaused;
        if (this.viewportPaused) {
            this.webcam.pauseViewport();
        } else {
            this.webcam.resumeViewport();
        }

    }
}