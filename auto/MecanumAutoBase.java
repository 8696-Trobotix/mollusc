package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.DrivetrainBaseFourWheel;

import org.firstinspires.ftc.teamcode.mollusc.auto.interpreter.Interpreter;

import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.Pose;

import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;

import org.firstinspires.ftc.teamcode.mollusc.utility.VoltageCompensator;
import org.firstinspires.ftc.teamcode.mollusc.utility.PIDF;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class MecanumAutoBase {
    
    public double moveTimeoutSeconds = 5.0;
    public int staticTimeoutMilliseconds = 500;

    public DrivetrainBaseFourWheel base;
    public Interpreter interpreter = null;
    public PIDF drivePIDF, strafePIDF, turnPIDF;
    public VoltageCompensator c1 = null, c2 = null, c3 = null, c4 = null;
    public double maxPower = 1.0, maxPowerDeltaDurationSeconds = 0.5, closeEnough = 0.1;

    public ElapsedTime runtime = new ElapsedTime();
    private double previousTime = 0.0;

    public void resetPIDF() {
        drivePIDF.restart();
        strafePIDF.restart();
        turnPIDF.restart();
    }

    protected double utilDelta(double p, double d) {
        double currentTime = runtime.seconds();
        double dt = Math.min(currentTime - previousTime, 0.1);
        previousTime = currentTime;
        p = Math.abs(d) <= closeEnough ? p - d : p - Math.signum(d) / maxPowerDeltaDurationSeconds * dt;
        return p;
    }

    public void waitDelay(double seconds) throws ParityException {
        LinearOpMode opMode = Mollusc.useLinearOpMode("MecanumAuto waitDelay.");
        ElapsedTime temp = new ElapsedTime();
        while (temp.seconds() < seconds && !opMode.isStopRequested()) {
            opMode.idle();
        }
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
