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

    // Static auxilary matrices since EOCV won't automatically release matrices per frame process.
    // Note: HSV hue [0, 179] (not [0, 359]).
    private static Mat blurred = new Mat();
    private static Mat hsv = new Mat();
    private static Mat mask1 = new Mat(), mask2 = new Mat();
    private static Mat hierarchy = new Mat();
    public static final Scalar GREEN_COLOR = new Scalar(0, 255, 0);
    public static final Scalar RED_COLOR = new Scalar(255, 0, 0);

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
        return coloredObjectCoordinates(src_dest, minimumPixelTotality, minimumBoundingTotality, new Size(5, 5), draw, ranges_hsv);
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

            double pixelTotality = Core.countNonZero(mask1.submat(rect)) / total;
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
            highlightContours(src_dest, contours, RED_COLOR);
            for (VisionObject obj : objs) {
                ObjectDetector.highlightObject(src_dest, obj, GREEN_COLOR);
            }
        }

        return objs;
    }

    // Applying a Gaussian blur to the image prior to contour detection can reduce noise.
    public static Mat gaussian(Mat src) {
        return gaussian(src, new Size(5, 5));
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
        Point p1 = new Point(obj.x - obj.width / 2, obj.y - obj.height / 2);
        Point p2 = new Point(p1.x + obj.width, p1.y + obj.height);
        Imgproc.rectangle(
            src_dest,
            p1, 
            p2, 
            color,
            2
        );
    }

    public static void highlightContours(Mat src_dest, List<MatOfPoint> contours, Scalar color) {
        Imgproc.drawContours(src_dest, contours, -1, color, 2);
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
