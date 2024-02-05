package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.DrivetrainBaseFourWheel;

import org.firstinspires.ftc.teamcode.mollusc.auto.interpreter.Interpreter;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.DeadWheels;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.Pose;

import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;

import org.firstinspires.ftc.teamcode.mollusc.utility.VoltageCompensator;
import org.firstinspires.ftc.teamcode.mollusc.utility.Configuration;
import org.firstinspires.ftc.teamcode.mollusc.utility.PIDF;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

public class MecanumAutoII extends MecanumAutoBase implements Auto {
    
    private DeadWheels deadWheels;
    private double positionToleranceSq, headingTolerance;

    private double fl, fr, rl, rr;

    public MecanumAutoII(
        DrivetrainBaseFourWheel base, 
        DeadWheels deadWheels, 
        Interpreter interpreter, 
        PIDF drivePID, 
        PIDF strafePID, 
        PIDF turnPID, 
        PIDF voltageCompensatorPIDF, 
        double maxCurrent, 
        double positionTolerance, 
        double headingTolerance
    ) {
        this.base = base;
        this.deadWheels = deadWheels;
        this.interpreter = interpreter;
        this.drivePID = drivePID;
        this.strafePID = strafePID;
        this.turnPID = turnPID;
        this.positionToleranceSq = positionTolerance * positionTolerance;
        this.headingTolerance = AngleUnit.normalizeRadians(Math.toRadians(headingTolerance));

        PIDF emptyPIDF = new PIDF(0, 0, 0, 0);
        c1 = new VoltageCompensator(base.frontLeft, emptyPIDF, 0);
        c2 = new VoltageCompensator(base.frontRight, emptyPIDF, 0);
        c3 = new VoltageCompensator(base.rearLeft, emptyPIDF, 0);
        c4 = new VoltageCompensator(base.rearRight, emptyPIDF, 0);
    }

    public void setVoltageCompensator(PIDF voltageCompensatorPIDF, double maxCurrent) {
        c1 = new VoltageCompensator(base.frontLeft, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c2 = new VoltageCompensator(base.frontRight, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c3 = new VoltageCompensator(base.rearLeft, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c4 = new VoltageCompensator(base.rearRight, new PIDF(voltageCompensatorPIDF), maxCurrent);
    }

    // Field-centric style automated drive with three dead wheel localizers.
    public void driveTo(Pose newPose) throws ParityException {
        LinearOpMode opMode = Configuration.useLinearOpMode();

        resetPIDF();

        ElapsedTime runtime = new ElapsedTime();
        int previousTime = 0;

//        Mollusc.opMode.telemetry.log().add("Driving to: " + newPose);

        while (opMode.opModeIsActive() && runtime.seconds() < moveTimeoutSeconds) {
            drivePowers(newPose);

//            Mollusc.opMode.telemetry.addData("Position", deadWheels.pose);
//            Mollusc.opMode.telemetry.addData("Powers", Arrays.toString(powers));
//            Mollusc.opMode.telemetry.update();

            base.frontLeft.setPower(fl);
            base.frontRight.setPower(fr);
            base.rearLeft.setPower(rl);
            base.rearRight.setPower(rr);

            int currentTime = (int)runtime.milliseconds();
            double a = newPose.x - deadWheels.pose.x, b = newPose.y - deadWheels.pose.y;
            boolean atCorrectPosition = positionToleranceSq >= a * a + b * b && headingTolerance >= Math.abs(AngleUnit.normalizeRadians(Math.toRadians(newPose.z) - deadWheels.pose.z));
            if (!atCorrectPosition) {
                previousTime = currentTime;
            }
            if (currentTime >= previousTime + staticTimeoutMilliseconds) {
                break;
            }
        }

        base.frontLeft.setPower(0);
        base.frontRight.setPower(0);
        base.rearLeft.setPower(0);
        base.rearRight.setPower(0);
    }

    private void drivePowers(Pose newPose) {
        double drive = drivePID.out(newPose.x - deadWheels.pose.x);
        double strafe = strafePID.out(newPose.y - deadWheels.pose.y);
        double turn = turnPID.out(-1 * AngleUnit.normalizeRadians(Math.toRadians(newPose.z) - deadWheels.pose.z));

        double heading = deadWheels.pose.z;
        // Calculations based on GM0.
        double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
        // Normalize. Also prevents power values from exceeding 1.0.
        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        double voltage = VoltageCompensator.getVoltage();
        fl = c1.adjustPower((rotY + rotX + turn) / max * maximumPower, voltage);
        fr = c2.adjustPower((rotY - rotX - turn) / max * maximumPower, voltage);
        rl = c3.adjustPower((rotY - rotX + turn) / max * maximumPower, voltage);
        rr = c4.adjustPower((rotY + rotX - turn) / max * maximumPower, voltage);

        deadWheels.update();
    }

    public void resetPIDF() {
        drivePIDF.restart();
        strafePIDF.restart();
        turnPIDF.restart();
    }
    public double[] getDrivePowers(Pose newPose) {
        drivePowers(newPose);
        return new double [] {fl, fr, rl, rr};
    }

    public DeadWheels getDeadWheels() {
        return deadWheels;
    }

    public double getPositionTolerance() {
        return Math.sqrt(positionToleranceSq);
    }
    public void setPositionTolerance(double tolerance) {
        this.positionToleranceSq = tolerance * tolerance;
    }

    public double getHeadingToleranceRadians() {
        return headingTolerance;
    }
    public void setHeadingToleranceRadians(double radians) {
        this.headingTolerance = radians;
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
