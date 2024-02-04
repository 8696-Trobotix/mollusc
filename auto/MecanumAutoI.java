package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.DrivetrainBaseFourWheel;

import org.firstinspires.ftc.teamcode.mollusc.auto.interpreter.Interpreter;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.Pose;

import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;

import org.firstinspires.ftc.teamcode.mollusc.utility.VoltageCompensator;
import org.firstinspires.ftc.teamcode.mollusc.utility.Configuration;
import org.firstinspires.ftc.teamcode.mollusc.utility.PIDF;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumAutoI extends MecanumAutoBase implements Auto {
    
    private IMU imu;
    private double headingOffsetRadians = 0.0;
    private double powerTolerance;

    private double drive_fl, drive_fr, drive_rl, drive_rr, 
                   strafe_fl, strafe_fr, strafe_rl, strafe_rr, 
                   turn_fl, turn_fr, turn_rl, turn_rr;
    private double[] powers = new double[12];

    public MecanumAutoI(
        DrivetrainBaseFourWheel base, 
        Interpreter interpreter, 
        PIDF drivePIDF, 
        PIDF strafePIDF, 
        PIDF turnPIDF, 
        IMU imu, 
        PIDF voltageCompensatorPIDF, 
        double maxCurrent, 
        double powerTolerance
    ) {
        this.base = base;
        this.interpreter = interpreter;
        this.drivePIDF = drivePIDF;
        this.strafePIDF = strafePIDF;
        this.turnPIDF = turnPIDF;
        this.imu = imu;
        this.powerTolerance = powerTolerance;

        c1 = new VoltageCompensator(base.frontLeft, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c2 = new VoltageCompensator(base.frontRight, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c3 = new VoltageCompensator(base.rearLeft, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c4 = new VoltageCompensator(base.rearRight, new PIDF(voltageCompensatorPIDF), maxCurrent);
    }

    // Field-centric style automated drive.
    // Recommended to use MecanumAutoII instead, as the heading is also accounted for whilst driving.
    public void driveTo(Pose newPose) throws ParityException {
        LinearOpMode opMode = Configuration.useLinearOpMode();

        drivePIDF.restart();
        setPower(opMode, newPose, 0, 1, 2, 3);

        strafePIDF.restart();
        setPower(opMode, newPose, 4, 5, 6, 7);

        turnPIDF.restart();
        setPower(opMode, newPose, 8, 9, 10, 11);
    }

    private void setPower(LinearOpMode opMode, Pose newPose, int fli, int fri, int rli, int rri) {
        ElapsedTime runtime = new ElapsedTime();
        int previousTime = 0;

        while (opMode.opModeIsActive() && runtime.seconds() < moveTimeoutSeconds) {
            drivePowers(newPose);

            base.frontLeft.setPower(powers[fli]);
            base.frontRight.setPower(powers[fri]);
            base.rearLeft.setPower(powers[rli]);
            base.rearRight.setPower(powers[rri]);

            int currentTime = (int)runtime.milliseconds();
            double powerNet = Math.abs(powers[0]) + Math.abs(powers[1]) + Math.abs(powers[2]) + Math.abs(powers[3]);
            if (Math.abs(powerNet) > powerTolerance) {
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

    public double[] drivePowers(Pose newPose) {
        int[] positions = base.getEncoderCounts();
        double drive = drivePIDF.out(newPose.x - (positions[0] + positions[1] + positions[2] + positions[3]) / 4);
        double strafe = strafePIDF.out(newPose.y - (positions[0] + positions[1]) / 2);
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffsetRadians;
        double turn = turnPIDF.out(-1 * AngleUnit.normalizeRadians(Math.toRadians(newPose.z) - heading));

        double voltage = VoltageCompensator.getVoltage();

        double drive_max = Math.max(Math.abs(drive) + Math.abs(turn), 1);
        powers[0] = drive_fl = c1.adjustPower((drive + turn) / drive_max * maxPower, voltage);
        powers[1] = drive_fr = c2.adjustPower((drive - turn) / drive_max * maxPower, voltage);
        powers[2] = drive_rl = c3.adjustPower((drive + turn) / drive_max * maxPower, voltage);
        powers[3] = drive_rr = c4.adjustPower((drive - turn) / drive_max * maxPower, voltage);

        double strafe_max = Math.max(Math.abs(strafe) + Math.abs(turn), 1);
        powers[4] = strafe_fl = c1.adjustPower((strafe + turn) / strafe_max * maxPower, voltage);
        powers[5] = strafe_fr = c2.adjustPower((-strafe - turn) / strafe_max * maxPower, voltage);
        powers[6] = strafe_rl = c3.adjustPower((-strafe + turn) / strafe_max * maxPower, voltage);
        powers[7] = strafe_rr = c4.adjustPower((strafe - turn) / strafe_max * maxPower, voltage);

        powers[8] = turn_fl = c1.adjustPower(turn * maxPower, voltage);
        powers[9] = turn_fr = c2.adjustPower(-turn * maxPower, voltage);
        powers[10] = turn_rl = c3.adjustPower(turn * maxPower, voltage);
        powers[11] = turn_rr = c4.adjustPower(-turn * maxPower, voltage);
    }

    public double[] getDrivePowers() {
        return powers.clone();
    }

    public void getIMU() {
        return imu;
    }

    public double getHeadingOffsetRadians() {
        return headingOffsetRadians;
    }
    public void setHeadingOffsetRadians(double radians) {
        this.headingOffsetRadians = radians;
    }

    public double getPowerTolerance() {
        return powerTolerance;
    }
    public void setPowerTolerance(double powerTolerance) {
        this.powerTolerance = powerTolerance;
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
