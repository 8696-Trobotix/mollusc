package org.firstinspires.ftc.teamcode.mollusc.auto.odometry;

import org.firstinspires.ftc.teamcode.mollusc.wrapper.Encoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DeadWheels {
    
    public Pose pose;
    private Encoder left, right, center;

    private double leftPrev, rightPrev, centerPrev;
    private double headingPrev;

    // Distance between left and right encoders.
    private double trackWidth;
    // Distance between the center encoder and the robot's center of rotation.
    // Should be negative if the center encoder is in the back, and positive if in the front (of the robot).
    private double centerOffset;

    public DeadWheels(
        Pose initialPose, 
        Encoder left, 
        Encoder right, 
        Encoder center, 
        double trackWidth, 
        double centerOffset
    ) {
        this.pose = new Pose(initialPose);
        this.left = left;
        this.right = right;
        this.center = center;
        this.trackWidth = trackWidth;
        this.centerOffset = centerOffset;

        this.leftPrev = left.getDisplacement();
        this.rightPrev = right.getDisplacement();
        this.centerPrev = center.getDisplacement();
        this.headingPrev = pose.z;
    }

    // Call continuously in control loop to get updated positions in `pose`.
    public void update() {
        // Note: variable name meanings do not match up between implementations below.

        // /*
        // Modified from https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/kinematics/HolonomicOdometry.java.

        double leftDis = left.getDisplacement();
        double rightDis = right.getDisplacement();
        double centerDis = center.getDisplacement();

        double deltaLeft = leftDis - leftPrev;
        double deltaRight = rightDis - rightPrev;
        double deltaCenter = centerDis - centerPrev;

        // Calculate change in heading using the difference between the displacements of the two parallel wheels.
        // This calculation cannot be done using the center wheel because linear displacement is not rotational displacement.
        double deltaHeading = AngleUnit.normalizeRadians((deltaLeft - deltaRight) / trackWidth);
        double heading = AngleUnit.normalizeRadians(headingPrev + deltaHeading);

        double deltaX = (deltaLeft + deltaRight) / 2;
        double deltaY = deltaCenter - (centerOffset * deltaHeading);

        double sinDeltaHeading = Math.sin(deltaHeading);
        double cosDeltaHeading = Math.cos(deltaHeading);

        // For small values of deltaHeading, limitations in floating-point calculations can lead to inaccuracies.
        // This section uses a Taylor series approximation when those small values are computed.
        // More importantly, it computes the pose exponential, 
        // which can provide more accurate approximations when traveling around curves.
        double s;
        double c;
        if (Math.abs(deltaHeading) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * deltaHeading * deltaHeading;
            c = 0.5 * deltaHeading;
        } else {
            s = sinDeltaHeading / deltaHeading;
            c = (1 - cosDeltaHeading) / deltaHeading;
        }

        double transformX = deltaX * s - deltaY * c;
        double transformY = deltaX * c + deltaY * s;

        leftPrev = leftDis;
        rightPrev = rightDis;
        centerPrev = centerDis;
        headingPrev = heading;

        pose.x += transformX * Math.cos(heading) - transformY * Math.sin(-heading);
        pose.y += transformX * Math.sin(-heading) + transformY * Math.cos(heading);
        pose.z = heading;
        // */

        /*
        // Use these calculations if the method above doesn't work.
        // Adapted from GM0.
        // This one uses standard Euler integration.

        double leftDis = left.getDisplacement();
        double rightDis = right.getDisplacement();
        double centerDis = center.getDisplacement();

        double deltaLeft = leftDis - leftPrev;
        double deltaRight = rightDis - rightPrev;
        double deltaCenter = centerDis - centerPrev;

        double deltaHeading = (deltaLeft - deltaRight) / trackWidth;
        double deltaX = (deltaLeft + deltaRight) / 2;
        double deltaY = deltaCenter - (centerOffset * deltaHeading);

        leftPrev = leftDis;
        rightPrev = rightDis;
        centerPrev = centerDis;

        pose.x += deltaX * Math.cos(pose.z) - deltaY * Math.sin(-pose.z);
        pose.y += deltaX * Math.sin(-pose.z) + deltaY * Math.cos(pose.z);
        pose.z += deltaHeading;
        */

        /*
        // Use these calculations if the above two methods don't work.
        // This one may not work.

        // Modified from https://github.com/Beta8397/virtual_robot/blob/master/TeamCode/src/org/firstinspires/ftc/teamcode/EncBot.java.

        double leftDis = left.getDisplacement();
        double rightDis = right.getDisplacement();
        double centerDis = center.getDisplacement();

        double deltaLeft = leftDis - leftPrev;
        double deltaRight = rightDis - rightPrev;
        double deltaCenter = centerDis - centerPrev;

        double deltaX = (deltaLeft + deltaRight) / 2;

        double deltaHeading = (deltaLeft - deltaRight) / trackWidth;
        double avgHeading = pose.z + deltaHeading / 2;

        double sinAvg = Math.sin(avgHeading);
        double cosAvg = Math.cos(avgHeading);

        leftPrev = leftDis;
        rightPrev = rightDis;
        centerPrev = centerDis;

        pose.x += deltaX * sinAvg - centerDis * cosAvg;
        pose.y += deltaX * cosAvg + centerDis * sinAvg;
        pose.z = AngleUnit.normalizeRadians(pose.z + deltaHeading);
        */
    }

    public void zero() {
        setPose(0, 0, 0);
        left.reset();
        right.reset();
        center.reset();
        leftPrev = rightPrev = centerPrev = 0;
    }

    public Pose getPose() {
        return new Pose(pose);
    }
    public void setPose(double x, double y, double z) {
        setPose(new Pose(x, y, z));
    }
    public void setPose(Pose pose) {
        this.pose = new Pose(pose);
        this.headingPrev = pose.z;
    }

    public Encoder getLeftEncoder() {
        return left;
    }
    public Encoder getRightEncoder() {
        return right;
    }
    public Encoder getCenterEncoder() {
        return center;
    }

    public double getTrackWidth() {
        return trackWidth;
    }
    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public double getCenterOffset() {
        return centerOffset;
    }
    public void setCenterOffset(double centerOffset) {
        this.centerOffset = centerOffset;
    }
}

/*
Copyright 2023 Trobotix 8696

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
