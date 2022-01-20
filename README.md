# frc-66-2022-rapidreact

Welcome to the code repository of Team 66 Grizzly Robotics 2022 source code.

Below are the project guidelines for committing code.

## Constants
Constants should be in Hungarian notation and reserved to a Constants.java file. They should be `public static final` as they should not be modified, that would defeat the purpose of "constant". Below is an example of how they should be specified. The goal of using Hungarian notation instead of SCREAMING CAMEL CASE is for readability and constant control.

***Example:***
```
public static final boolean kEnableDebug = false;
public static final double kWheelDiameter = 3.5;
public static final boolean kReverseRightMotor = false;
```

## Class Names
Class names should start with a capital letter, contain no abbrevations, underscores, or special characters.

***Example:***  
```
Robot.java
Constants.java
Drivetrain.java
AutoRoutine.java
```
## Method & Instance Names
Method and Instance names should use the standard camel case convention. Member convention is also accepted.

***Example:***  
```java
private AutoRoutine autoRoutine = new AutoRoutine();

public void doThisRandomThing(){

}

public boolean returnFalse(){

  return false;
}
```
## Comments
Every public method should have a comment above stating what the method does, especially on methods where the contents may be hard to understand due to the complexity of the logic.

***Example:***  
```java
// Calculates the required voltage needed to move to the target without overshooting
public void calculateVoltageToTarget() {

}
```