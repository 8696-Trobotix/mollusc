# mollusc
🐚 Trobotix common modules.

## Usage
> [!Note]
> Requires `minSdkVersion 23` to be changed to `minSdk 24` or `minSdkVersion 24` in `build.common.gradle`  
> if the robot controller version is less then 8.2.

> [!Warning]
> This was last tested for robot controller version 9.0.1, and may not work as intended for later versions.

Download or clone [pre-release v0.1.0](https://github.com/8696-Trobotix/mollusc/releases/tag/v0.1.0) into `teamcode`, optionally add it as a git submodule.  
The current state of the main branch is unstable. A series of refactorings and some additional features were added while this library was developed alongside the Trobotix codebase for the CENTERSTAGE season. These changes have not been tested, so use the main branch at one's own discretion and with caution. Also note that the supposed "tests" themselves have never been run nor tested and that the library in general does not strictly adhere to a conventional Java project structure.

## Tested Features
- An "intepreter" that reads, parses, and executes a file containing a primitive yet customizable command-like language to control autonomous movements.
- Three dead wheel system odometry calculations.
> Note:  
> From an initial starting orientation, forward movement corresponds to a positive X translation and rightward movement corresponds to a positive Y translation.
> ```
>                /\
>                | +x
>                |
>    -y     robot front     +y
> <---------           --------->
>           robot back
>                |
>                | -x
>                \/
> ```
- Mecanum drive controlled by dead wheels for autonomous.
- Field centric and robot centric TeleOp drivetrains.
- External asset loading.
- Program configuration via the Driver Station.
- Gamepad controls utilities.
- PIDF controller.
- Contour-based multiple object detector for CV pipelines.

## Untested Features
- Mecanum drive controlled by wheel encoders for autonomous.
- Some driver station configuration functions.
- Low-pass and Kalman filters.
- Threaded function execution wrapper.
- Voltage compensator.
- AprilTag detection EOCV pipeline.
