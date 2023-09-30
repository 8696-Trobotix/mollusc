public interface Drivetrain {

    public void zeroEncoders();

    public void setDriveParams(double, double);

    public void drive(double, double, double);

    public int[] getEncoderCounts();
}
