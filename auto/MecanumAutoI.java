package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.DrivetrainBaseFourWheel;
import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;
import org.firstinspires.ftc.teamcode.mollusc.utility.Configuration;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.Pose;
import org.firstinspires.ftc.teamcode.mollusc.utility.PID;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumAutoI implements Auto {

    public DrivetrainBaseFourWheel base;
    public Interpreter interpreter;
    public PID drivePID, strafePID, turnPID;
    public IMU imu;
    public double heading = 0.0;

    public MecanumAutoI(
        DrivetrainBaseFourWheel base, 
        Interpreter interpreter, 
        PID drivePID, 
        PID strafePID, 
        PID turnPID, 
        IMU imu
    ) {
        this.base = base;
        this.interpreter = interpreter;
        this.drivePID = drivePID;
        this.strafePID = strafePID;
        this.turnPID = turnPID;
        this.imu = imu;
    }

    // Field-centric style automated drive with three dead wheel localizers.
    public void driveTo(Pose newPose) throws ParityException {
        LinearOpMode opMode = Configuration.useLinearOpMode();

        drivePID.restart();
        setPower(opMode, newPose, 0, 1, 2, 3);

        strafePID.restart();
        setPower(opMode, newPose, 4, 5, 6, 7);

        turnPID.restart();
        setPower(opMode, newPose, 8, 9, 10, 11);
    }

    private void setPower(LinearOpMode opMode, Pose newPose, int fli, int fri, int rli, int rri) {
        ElapsedTime runtime = new ElapsedTime();
        double powerNetPrev = 1.0;

        while (opMode.opModeIsActive() && runtime.seconds() < TIMEOUT) {
            double[] powers = drivePowers(newPose);

            base.frontLeft.setPower(powers[fli]);
            base.frontRight.setPower(powers[fri]);
            base.rearLeft.setPower(powers[rli]);
            base.rearRight.setPower(powers[rri]);

            double powerNet = Math.abs(powers[0]) + Math.abs(powers[1]) + Math.abs(powers[2]) + Math.abs(powers[3]);
            boolean t = (int)runtime.milliseconds() % STATIC_TIMOUT_MILLISECONDS == 0;
            if (
                Math.abs(powerNet) < 1E-6 
                && Math.abs(powerNetPrev) < 1E-6
                && t
            ) {
                break;
            } else if (t) {
                powerNetPrev = powerNet;
            }
        }

        base.frontLeft.setPower(0);
        base.frontRight.setPower(0);
        base.rearLeft.setPower(0);
        base.rearRight.setPower(0);
    }

    public double[] drivePowers(Pose newPose) {
        double drive = drivePID.out(newPose.x - (
            base.frontLeft.getCurrentPosition() + base.frontRight.getCurrentPosition()
            + base.rearLeft.getCurrentPosition() + base.rearRight.getCurrentPosition()) / 4);
        double strafe = strafePID.out(newPose.y - (
            base.frontLeft.getCurrentPosition() + base.rearRight.getCurrentPosition()) / 2);
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double turn = turnPID.out(AngleUnit.normalizeRadians(-1 * (Math.toRadians(newPose.z) - heading)));

        double drive_max = Math.max(Math.abs(drive) + Math.abs(turn), 1);
        double drive_fl = (drive + turn) / drive_max;
        double drive_fr = (drive - turn) / drive_max;
        double drive_rl = (drive + turn) / drive_max;
        double drive_rr = (drive - turn) / drive_max;

        double strafe_max = Math.max(Math.abs(strafe) + Math.abs(turn), 1);
        double strafe_fl = (strafe + turn) / strafe_max;
        double strafe_fr = (-strafe - turn) / strafe_max;
        double strafe_rl = (-strafe + turn) / strafe_max;
        double strafe_rr = (strafe - turn) / strafe_max;

        double turn_fl = turn;
        double turn_fr = -turn;
        double turn_rl = turn;
        double turn_rr = -turn;

        return new double[] {
            drive_fl, drive_fr, drive_rl, drive_rr, 
            strafe_fl, strafe_fr, strafe_rl, strafe_rr, 
            turn_fl, turn_fr, turn_rl, turn_rr
        };
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
            driveTo(new Pose((Integer)pose[0], (Integer)pose[1], (Integer)pose[2]));
        }, Integer.class, Integer.class, Integer.class);
        interpreter.register("drive", (Object[] pose) -> {
            driveTo(new Pose((Double)pose[0], (Double)pose[1], (Double)pose[2]));
        }, Double.class, Double.class, Double.class);
        interpreter.register("wait", (Object[] pose) -> {
            waitDelay((Double)pose[0]);
        }, Double.class);
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
