package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.DrivetrainBaseFourWheel;
import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.DeadWheels;
import org.firstinspires.ftc.teamcode.mollusc.utility.Configuration;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.Pose;
import org.firstinspires.ftc.teamcode.mollusc.utility.PID;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

public class MecanumAutoII implements Auto {

    public static double TIMEOUT = 5.0;
    public static int STATIC_TIMEOUT_MILLISECONDS = 500;

    public DrivetrainBaseFourWheel base;
    public DeadWheels deadWheels;
    public Interpreter interpreter;
    public PID drivePID, strafePID, turnPID;
    public double maximumPower;
    public double positionTolerance, headingTolerance;

    public MecanumAutoII(
        DrivetrainBaseFourWheel base, 
        DeadWheels deadWheels, 
        Interpreter interpreter, 
        PID drivePID, 
        PID strafePID, 
        PID turnPID, 
        double maximumPower, 
        double positionTolerance, 
        double headingTolerance
    ) {
        this.base = base;
        this.deadWheels = deadWheels;
        this.interpreter = interpreter;
        this.drivePID = drivePID;
        this.strafePID = strafePID;
        this.turnPID = turnPID;
        this.maximumPower = maximumPower;
        this.positionTolerance = positionTolerance;
        this.headingTolerance = AngleUnit.normalizeRadians(Math.toRadians(headingTolerance));
    }

    // Field-centric style automated drive with three dead wheel localizers.
    public void driveTo(Pose newPose) throws ParityException {
        LinearOpMode opMode = Configuration.useLinearOpMode();

        drivePID.restart();
        strafePID.restart();
        turnPID.restart();

        ElapsedTime runtime = new ElapsedTime();
        int previousTime = -STATIC_TIMEOUT_MILLISECONDS;
        boolean wasAtCorrectPosition = false;

//        Mollusc.opMode.telemetry.log().add("Driving to: " + newPose);

        while (opMode.opModeIsActive() && runtime.seconds() < TIMEOUT) {
            double[] powers = drivePowers(newPose);

//            Mollusc.opMode.telemetry.addData("Position", deadWheels.pose);
//            Mollusc.opMode.telemetry.addData("Powers", Arrays.toString(powers));
//            Mollusc.opMode.telemetry.update();

            base.frontLeft.setPower(powers[0]);
            base.frontRight.setPower(powers[1]);
            base.rearLeft.setPower(powers[2]);
            base.rearRight.setPower(powers[3]);

            int currentTime = (int)runtime.milliseconds();
            double a = newPose.x - deadWheels.pose.x, b = newPose.y - deadWheels.pose.y;
            boolean atCorrectPosition = positionTolerance >= a && positionTolerance >= b && headingTolerance >= Math.abs(AngleUnit.normalizeRadians(Math.toRadians(newPose.z) - deadWheels.pose.z));
            if (
                currentTime / STATIC_TIMEOUT_MILLISECONDS != previousTime / STATIC_TIMEOUT_MILLISECONDS // At least the static timeout duration has passed.
                && atCorrectPosition // Currently at correct position.
                && wasAtCorrectPosition // Was at correct position at a time equal to or greater than the static timeout duration ago.
            ) {

//                Mollusc.opMode.telemetry.log().add("Static stopped.");

                break;
            }
            wasAtCorrectPosition = atCorrectPosition;
            previousTime = currentTime;
        }

        base.frontLeft.setPower(0);
        base.frontRight.setPower(0);
        base.rearLeft.setPower(0);
        base.rearRight.setPower(0);
    }

    public double[] drivePowers(Pose newPose) {
        double drive = drivePID.out(newPose.x - deadWheels.pose.x);
        double strafe = strafePID.out(newPose.y - deadWheels.pose.y);
        double turn = turnPID.out(-1 * AngleUnit.normalizeRadians(Math.toRadians(newPose.z) - deadWheels.pose.z));

        double heading = deadWheels.pose.z;
        // Calculations based on GM0.
        double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
        // Normalize. Also prevents power values from exceeding 1.0.
        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double fl = (rotY + rotX + turn) / max * maximumPower;
        double fr = (rotY - rotX - turn) / max * maximumPower;
        double rl = (rotY - rotX + turn) / max * maximumPower;
        double rr = (rotY + rotX - turn) / max * maximumPower;

        deadWheels.update();

        return new double[] {fl, fr, rl, rr};
    }

    public void waitDelay(double seconds) throws ParityException {
        LinearOpMode opMode = Configuration.useLinearOpMode();
        ElapsedTime temp = new ElapsedTime();
        while (temp.seconds() < seconds) {
            opMode.idle();
        }
    }

    public void register() throws ParityException {
        Configuration.useLinearOpMode();
        interpreter.register("drive", (Object[] pose) -> {
            driveTo(
                new Pose(
                    Double.parseDouble((String)pose[0]), 
                    Double.parseDouble((String)pose[1]), 
                    Double.parseDouble((String)pose[2])
                )
            );
        }, String.class, String.class, String.class);
        interpreter.register("wait_seconds", (Object[] pose) -> {
            waitDelay((Double)pose[0]);
        }, Double.class);
        interpreter.register("set_timeout_seconds", (Object[] t) -> {
            TIMEOUT = (Double)t[0];
        }, Double.class);
        interpreter.register("set_static_timeout_milliseconds", (Object[] t) -> {
            STATIC_TIMEOUT_MILLISECONDS = (Integer)t[0];
        }, Integer.class);
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
