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

public class MecanumAuto {

    public static double TIMEOUT = 5.0;

    public DrivetrainBaseFourWheel base;
    public DeadWheels deadWheels;
    public PID drivePID, strafePID, turnPID;

    public MecanumAuto(
        DrivetrainBaseFourWheel base, 
        DeadWheels deadWheels, 
        Interpreter interpreter, 
        PID drivePID, 
        PID strafePID, 
        PID turnPID
    ) {
        this.base = base;
        this.deadWheels = deadWheels;
        this.interpreter = interpreter;
        this.drivePID = drivePID;
        this.strafePID = strafePID;
        this.turnPID = turnPID;

        register();
    }

    // Field-centric style automated drive with three dead wheel localizers.
    public void driveTo(Pose newPose) throws ParityException {
        LinearOpMode opMode = Configuration.useLinearOpMode();

        drivePID.restart();
        headingPID.restart();

        ElapsedTime runtime = new ElapsedTime();

        while (opMode.opModeIsActive() && runtime.seconds() < TIMEOUT) {
            double[] powers = drivePowers(newPose);

            base.frontLeft.setPower(powers[0]);
            base.frontRight.setPower(powers[1]);
            base.rearLeft.setPower(powers[2]);
            base.rearRight.setPower(powers[3]);
        }

        base.frontLeft.setPower(0);
        base.frontRight.setPower(0);
        base.rearLeft.setPower(0);
        base.rearRight.setPower(0);
    }

    public double[] drivePowers(Pose newPose) {
        double drive = drivePID.out(newPose.x - deadWheels.pose.x);
        double strafe = strafePID.out(newPose.y - deadWheels.pose.y);
        double turn = turnPID.out(newPose.z - Math.toDegrees(deadWheels.pose.z));

        double heading = deadWheels.pose.z;
        // Calculations based on GM0.
        double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
        // Normalize. Also prevents power values from exceeding 1.0.
        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double fl = (rotY + rotX + turn) / max;
        double fr = (rotY - rotX - turn) / max;
        double rl = (rotY - rotX + turn) / max;
        double rr = (rotY + rotX - turn) / max;

        deadWheels.update();

        return new double[] {fl, fr, rl, rr};
    }

    private void register() {
        interpreter.register("drive", (Object[] pose) -> {
            driveTo(new Pose((Integer)pose[0], (Integer)pose[1], (Integer)pose[2]));
        }, Integer.class, Integer.class, Integer.class);
        interpreter.register("drive", (Object[] pose) -> {
            driveTo(new Pose((Double)pose[0], (Double)pose[1], (Double)pose[2]));
        }, Double.class, Double.class, Double.class);
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
