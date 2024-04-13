package org.firstinspires.ftc.teamcode.mollusc.vision;

import org.opencv.imgproc.Imgproc;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetector {

    // Static auxiliary matrices since EOCV won't automatically release matrices per frame process.
    // Note: HSV hue [0, 179] (not [0, 359]).
    private static Mat blurred = new Mat();
    private static Mat hsv = new Mat();
    private static Mat mask1 = new Mat(), mask2 = new Mat();
    private static Mat hierarchy = new Mat();
    public static final Scalar RED_COLOR = new Scalar(255, 0, 0);
    public static final Scalar GREEN_COLOR = new Scalar(0, 255, 0);
    public static final Scalar BLUE_COLOR = new Scalar(0, 0, 255);
    public static final Scalar WHITE_COLOR = new Scalar(255, 255, 255);
    public static final Scalar GREY_COLOR = new Scalar(100, 100, 100);
    public static final Scalar BLACK_COLOR = new Scalar(0, 0, 0);

    public static Size defaultGaussianKernelSize = new Size(5, 5);
    public static int highlightThickness = 2;
    public static Scalar contourHighlightColor = RED_COLOR;
    public static Scalar objectHighlightColor = GREEN_COLOR;
    public static double textFontScale = 0.5;
    public static int textFontThickness = 1;

    public static List<VisionObject> coloredObjectCoordinates(
        Mat src_dest, 
        double minimumPixelTotality, 
        double minimumBoundingTotality, 
        boolean draw, 
        List<ColorRange> ranges_hsv
    ) {
        return coloredObjectCoordinates(src_dest, minimumPixelTotality, minimumBoundingTotality, draw, ranges_hsv.toArray(new ColorRange[0]));
    }
    public static List<VisionObject> coloredObjectCoordinates(
        Mat src_dest, 
        double minimumPixelTotality, 
        double minimumBoundingTotality, 
        boolean draw, 
        ColorRange ...ranges_hsv
    ) {
        return coloredObjectCoordinates(src_dest, minimumPixelTotality, minimumBoundingTotality, defaultGaussianKernelSize, draw, ranges_hsv);
    }
    public static List<VisionObject> coloredObjectCoordinates(
        Mat src_dest, 
        double minimumPixelTotality, 
        double minimumBoundingTotality, 
        Size gaussianKernelSize, 
        boolean draw, 
        ColorRange ...ranges_hsv
    ) {
        gaussian(src_dest, gaussianKernelSize);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);
        mask(hsv, ranges_hsv);

        List<MatOfPoint> contours = contours(mask1);

        List<VisionObject> objs = new ArrayList<>();
        double total = mask1.cols() * mask1.rows();
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);

            double pixelTotality = Math.abs(Imgproc.contourArea(contour)) / total;
            // double pixelTotality = Core.countNonZero(mask1.submat(rect)) / total;
            double boundingTotality = rect.width * rect.height / total;
            if (
                pixelTotality >= minimumPixelTotality 
                && boundingTotality >= minimumBoundingTotality
            ) {
                objs.add(
                    new VisionObject(
                        rect.x + rect.width / 2, 
                        rect.y + rect.height / 2, 
                        rect.width, 
                        rect.height, 
                        pixelTotality, 
                        boundingTotality
                    )
                );
            }
        }

        if (draw) {
            highlightContours(src_dest, contours, contourHighlightColor);
            for (VisionObject obj : objs) {
                ObjectDetector.highlightObject(src_dest, obj, objectHighlightColor);

                String text = String.format("(%d, %d, %f, %f)", obj.x, obj.y, obj.pixelTotality, obj.boundingTotality);
                Size textSize = Imgproc.getTextSize(text, Imgproc.FONT_HERSHEY_SIMPLEX, textFontScale, textFontThickness, null);
                int x = (int)Math.min(Math.max(textSize.width / 2, obj.x), src_dest.cols() - textSize.width / 2);
                int y = (int)Math.min(Math.max(textSize.height / 2, obj.y), src_dest.rows() - textSize.height / 2);
                simpleText(src_dest, text, x, y, true, textFontScale, WHITE_COLOR, textFontThickness + 1);
                simpleText(src_dest, text, x, y, true, textFontScale, BLACK_COLOR, textFontThickness);
            }
        }

        return objs;
    }

    // Applying a Gaussian blur to the image prior to contour detection can reduce noise.
    public static Mat gaussian(Mat src) {
        return gaussian(src, defaultGaussianKernelSize);
    }
    public static Mat gaussian(Mat src, Size gaussianKernelSize) {
        Imgproc.GaussianBlur(src, blurred, gaussianKernelSize, 0);
        return blurred;
    }

    public static Mat mask(Mat src, List<ColorRange> ranges) {
        return mask(src, ranges.toArray(new ColorRange[0]));
    }
    public static Mat mask(Mat src, ColorRange ...ranges) {
        mask1.setTo(new Scalar(0, 0, 0, 0));
        mask2.setTo(new Scalar(0, 0, 0, 0));
        for (ColorRange range : ranges) {
            Core.inRange(src, range.lowerBound, range.upperBound, mask2);

            if (mask1.cols() != mask2.cols() || mask1.rows() != mask2.rows()) {
                mask2.copyTo(mask1);
            }
            Core.bitwise_or(mask1, mask2, mask1);
        }
        return mask1;
    }

    public static List<MatOfPoint> contours(Mat src) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(src, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    public static void highlightObject(Mat src_dest, VisionObject obj, Scalar color) {
        int x = obj.x - obj.width / 2;
        int y = obj.y - obj.height / 2;
        rectangle(
            src_dest, 
            x, 
            y, 
            x + obj.width, 
            y + obj.height, 
            color, 
            highlightThickness
        );
    }

    public static void highlightContours(Mat src_dest, List<MatOfPoint> contours, Scalar color) {
        Imgproc.drawContours(src_dest, contours, -1, color, highlightThickness);
    }

    public static void rectangle(Mat src_dest, int x1, int y1, int x2, int y2, Scalar color, int thickness) {
        Point p1 = new Point(x1, y1);
        Point p2 = new Point(x2, y2);
        Imgproc.rectangle(
            src_dest,
            p1, 
            p2, 
            color,
            thickness
        );
    }

    public static void simpleText(
        Mat src_dest, 
        String text, 
        int x, 
        int y, 
        boolean center, 
        double scale, 
        Scalar color, 
        int thickness
    ) {
        if (center) {
            Size textSize = Imgproc.getTextSize(text, Imgproc.FONT_HERSHEY_SIMPLEX, scale, thickness, null);
            x -= textSize.width / 2;
            y += textSize.height / 2;
        }
        Imgproc.putText(src_dest, text, new Point(x, y), Imgproc.FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    }
}

/*
Copyright 2024 Trobotix 8696

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
