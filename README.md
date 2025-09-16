# FRC Robot Project - Sim Skeleton

This is an FRC robot project using WPILib 2025, AdvantageKit, and CTRE Phoenix 6.

## Prerequisites

- WPILib 2025 installed (includes Java JDK)
- Visual Studio Code with WPILib extension (recommended)

## Build Instructions

### Windows

The project uses Gradle for building. The Java JDK is included with WPILib installation.

```bash
# Set JAVA_HOME to WPILib's JDK and build
JAVA_HOME="C:/Users/Public/wpilib/2025/jdk" ./gradlew build

# Or use the gradlew wrapper directly if JAVA_HOME is configured in your environment
./gradlew build
```

### Common Build Issues and Solutions

#### Issue 1: JAVA_HOME not set
**Error:** `ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH`

**Solution:**
- On Windows, WPILib's JDK is located at `C:\Users\Public\wpilib\2025\jdk`
- Set JAVA_HOME before running gradlew: `JAVA_HOME="C:/Users/Public/wpilib/2025/jdk" ./gradlew build`
- Or set it permanently in your system environment variables

#### Issue 2: Command execution in Windows
**Problem:** Windows command prompt and bash handle commands differently

**Solution:**
- Use `./gradlew` in Git Bash or WSL
- Use `gradlew.bat` in Windows Command Prompt
- In PowerShell, use `./gradlew` or `.\gradlew.bat`

### Important Notes

#### CTRE Phoenix 6 API
When using CTRE Phoenix 6 with TalonFX (Kraken X60):
- Status signals use typed units (Angle, AngularVelocity, Voltage, etc.) not raw doubles
- Use `.getValue().in(Units.YourUnit)` to get numeric values
- Current limits: Use `SupplyCurrentLimit` and `StatorCurrentLimit` (not threshold-based)
- FOC (Field Oriented Control) is available and recommended for better performance

Example:
```java
// Correct Phoenix 6 usage
StatusSignal<Angle> positionSignal = motor.getPosition();
double rotations = positionSignal.getValue().in(Units.Rotations);

// Incorrect (won't compile)
StatusSignal<Double> positionSignal = motor.getPosition(); // Type mismatch!
```