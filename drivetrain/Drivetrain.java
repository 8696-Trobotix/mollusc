package org.firstinspires.ftc.teamcode.mollusc.drivetrain;

import com.qualcomm.robotcore.hardware.IMU;

public interface Drivetrain {

    public void zeroEncoders();

    public void setDriveParams(double turnSpeedMax, double strafeCorrection);

    public void drive(double drive, double strafe, double turn);

    public int[] getEncoderCounts();

    public IMU getIMU();
}
