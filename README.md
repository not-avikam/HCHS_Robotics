## HCHS Robotics official GitHub. Team number 24659

## Autonomous
We use SparkFun’s Optical Odometry as our localizer, and are working on development of an autonomous at the time of writing.

## Intelligent Adjustment
In TeleOp, our odometry constantly updates the location of the robot. At some key
locations, our robot is programmed to automatically adjust to what task it expects we
would perform. For example, when approaching the observation zone, our robot
is developed to detect that the observation zone is approaching and automatically
adjust viper slide and claw positions for specimen pick up.

## Automatic Override
Driver 2 has the ability to select between preset claw positions. Driver 2 can use the d 
pad to choose between the preset for sample pickup, sample/specimen scoring, and 
specimen pickup. Additionally, driver 1 spinning the intake automatically adjusts the 
claw into sample pickup position.

## Manual Override
In manual override, driver 2 has the ability to adjust every component manually, in 
extenuating circumstances or failure of automatic adjustment.

## Color Recognition
A webcam mounted on the linear slides by the intake allows for color recognition of 
incoming samples. If the color is incorrect, an audio alert plays and the drivers’ 
vibrate. This allows a driver who may not have a clear view of a sample or one who 
may be confused to not make an error while picking up a sample. By mounting the 
webcam on the linear slide and specifying an ROI in the code, we are able to meet 
size requirements for this season.


## Below is the README provided by Pedro Pathing

## Welcome to the Offical Pedro Pathing Quickstart!  

Follow the steps on our [website](https://pedropathing.com/) to tune and setup!  
Feel Free to reach out on the [Offical Pedro Pathing Discord Server](https://discord.gg/2GfC4qBP5s)!
