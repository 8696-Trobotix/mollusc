/*
Mecanum Robot Centric

Drivetrain hardware class.

vIX-XXX-XXIII
*/

package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumRobotCentric implements Drivetrain {

    public DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    public double turnSpeedMax = 0.9, strafeCorrection = 1.1;

    public MecanumRobotCentric(
        HardwareMap hardwareMap, 
        Telemetry telemetry, 
        String fl, DcMotorEx.Direction fld, 
        String fr, DcMotorEx.Direction frd, 
        String rl, DcMotorEx.Direction rld, 
        String rr, DcMotorEx.Direction rrd
    ) {
        // Connect motors.
        frontLeft  = hardwareMap.get(DcMotorEx.class, fl);
        frontRight = hardwareMap.get(DcMotorEx.class, fr);
        rearLeft   = hardwareMap.get(DcMotorEx.class, rl);
        rearRight  = hardwareMap.get(DcMotorEx.class, rr);

        // Set motor directions.
        frontLeft.setDirection(fld);
        frontRight.setDirection(frd);
        rearLeft.setDirection(rld);
        rearRight.setDirection(rrd);

        // Set motors to brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (telemetry != null) {
            telemetry.log().add("Initialized robot centric hardware.");
            telemetry.update();
        }
    }

    public void zeroEncoders() {
        // Reset encoder counts to zero and set run mode to by power.

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDriveParams(double turnSpeedMax, double strafeCorrection) {
        this.turnSpeedMax = turnSpeedMax;
        this.strafeCorrection = strafeCorrection;
    }

    public void drive(double drive, double strafe, double turn) {
        strafe *= strafeCorrection;
        turn   *= turnSpeedMax;

        // Quadratic controller sensitivity.
        drive  *= Math.abs(drive);
        strafe *= Math.abs(strafe);
        turn   *= Math.abs(turn);

        // Calculations.
        double max = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        fl = (drive + strafe + turn) / max;
        fr = (drive - strafe - turn) / max;
        rl = (drive - strafe + turn) / max;
        rr = (drive + strafe - turn) / max;

        // Act.
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        rearLeft.setPower(rl);
        rearRight.setPower(rr);
    }

    public int[] getEncoderCounts() {
        return new int[] {
            frontLeft.getCurrentPosition(), 
            frontRight.getCurrentPosition(), 
            rearLeft.getCurrentPosition(), 
            rearRight.getCurrentPosition()
        };
    }
}
