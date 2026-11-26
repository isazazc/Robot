# First drive on the current chassis

1. With power off, secure the hub, battery and loose wiring away from wheels and shafts. Decide which end is the front and label all four corners. Check that diagonally opposite mecanum wheels have the same handedness and adjacent wheels have opposite handedness.
2. In the Driver Station configuration, map the four motor ports to `frontLeft`, `frontRight`, `backLeft`, and `backRight`. Match names and case exactly. The two bring-up modes require only these entries.
3. Support the chassis with all wheels clear. Start **Wheel Check**, select each corner with D-pad left/right, and briefly hold the right bumper. Only the selected motor should turn. A positive command should spin each wheel in the direction that would propel that corner forward on the floor. Correct the hub port mapping if the wrong wheel turns. If the correct wheel turns backward, change its entry in `MecanumDrive.DIRECTIONS`. The front-left setting was flipped after the team's Wheel Check. Verify all four directions on the chassis.
4. Put the chassis on a clear floor. Start **Drive Only**, centre the sticks, release the right bumper, then hold it to drive. The left stick commands forward and sideways motion relative to the chassis. The right stick turns. Power is capped at 40% and holding the left trigger drops the cap to 20%. Release the bumper to request zero power and brake.
5. Test forward, reverse, right strafe, left strafe and both turn directions independently. If straight motion works and strafing does not, recheck wheel handedness and placement. Confirm the robot stops when you release the bumper and when you press Stop on the Driver Station.
6. Once straight driving is verified, the **Auto** OpMode is a 1.5-second straight-drive smoke test at 30% power. Allow open space for the run. It does not target a measured distance.

Open the repository root in Android Studio, select the TeamCode run configuration and deploy to the Control Hub. The full **Main TeleOp** still expects last year's mechanism and Limelight configuration. Its vision alignment and mechanism settings need separate testing on this chassis.

## Encoder and Pedro Pathing setup for this year's robot

This path test assumes the pictured four-wheel mecanum layout. Confirm that is what “cannon wheels” refers to before using sideways motion. It shares the four verified drive names and directions with Drive Only. There is no dependency on last year's mechanisms or camera. SDK 11.1 is retained and Pedro's FTC artifact is pinned to 2.0.6 from Maven Central. See the [matching source](https://github.com/Pedro-Pathing/PedroPathing/tree/v2.0.6).

1. Verify wheel directions and encoder wiring first. In **Wheel Check**, release the bumper and press A to zero the displayed offsets. With the chassis supported, mark each wheel at one reference point and turn it through several complete revolutions. Record signed count change divided by revolutions. During a positive motor test its count should increase. Zero counts with the wheel moving indicate a configuration or wiring problem. Check the result against the motor spec sheet and the displayed FTC motor-profile ticks per **gearbox output** revolution. If there's another ratio from gearbox to wheel, multiply the gearbox-output count by gearbox revolutions per wheel revolution. Use measured ticks per **wheel** revolution to avoid ambiguity about vendor CPR or gearbox ratios.
2. Start with effective inches per wheel tick equal to wheel circumference in inches divided by measured ticks per wheel revolution. Check that wheel scales are similar. Measure track width and wheelbase centre-to-centre. The Pedro 2.0.6 encoder localizer uses **sums**, so its initial straight scale is inches per wheel tick divided by four.
3. Calibrate on the actual floor at low power. Zero the display at the start of each trial and record all four deltas (FL, FR, BL, BR) and externally measured displacement or heading. Repeat in both directions and compare the results. Pedro X is forward at zero heading, Y is left, and positive rotation is counterclockwise. Use inches and radians. Fit the scales with these exact signs and denominators:
   - Forward scale = measured forward inches / (FL + FR + BL + BR).
   - Strafe scale = measured left inches / (-FL + FR + BL - BR).
   - Turn scale = measured CCW radians × (track width + wheelbase) / (-FL + FR - BL + BR).
   - Ignore near-zero denominators and redo any test with unexpected signs. Check repeatability with a tape measure and a floor heading reference. A robot-to-field rotation and translation preserve length. Tick conversion and sideways slip calibration are separate from that coordinate transform.
4. Measure representative forward and strafe speed after acceleration at the **same known power** in each direction before filling in Pedro's drivetrain velocity inputs, which are used for their directional ratio in this version. Enter the measured values in `PedroChassisCalibration` and switch `VERIFIED` on once the measurements and signs are checked. Avoid copying last year's gearing or tuning values. Initial Pedro feedback gains are library defaults and need tuning on the chassis.
5. Run **Pedro: Straight Test** on clear floor. Release and then hold right bumper to command one 12-inch straight segment at up to 25% power. Release the bumper or press Stop to end it early. It also stops after five seconds of following. Compare the final encoder estimate to actual measured displacement and heading. Repeat before adding strafe, turns or longer paths. A small estimated error can coexist with a large physical error if a wheel slips. Use short straight segments, low power and externally measured endpoints before depending on longer routes.

## BIOBUZZ AUTO: Leave and Park

This competition OpMode stays locked until Pedro is calibrated and a measured route is entered. It uses the drive motors and encoders alone. Follow the [current FIRST manual](https://ftc-resources.firstinspires.org/ftc/archive/2027/game) for starting placement, preloads and scoring.

### Survey one route per alliance

- Select a legal start near your loading zone on your own side. Agree on the partner's approach corridor and parking position. Verify that both robot envelopes fit at the end. No software collision detection is available.
- Put the back of your chassis against the wall and face directly inward. Define a wall-local frame with the wall at X=0, positive X inward, and positive Y to the robot's left. Put Y=0 behind the starting chassis centre. Measure everything in inches. All headings for this routine are zero.
- Use the measured plan-view footprint. The code models a rectangle centred on the robot's localization point. If the chassis has asymmetric protrusions, verify this model against the actual outline before assigning a route. Choose a straight inward departure point. Keep enough inward travel to clear the wall and any Pollen on the floor. Arrange the four preloads legally and check the wheels cannot trap them.
- Mark a parking target that leaves at least 50 mm to the perimeter and overlaps the zone by at least 50 mm along each axis. Measure the loading-zone rectangle including its tape. Measure a rectangular clear corridor enclosing both straight swept paths and the full robot, clear of Flowers, the Hive, the other field walls and the partner's planned occupancy. Choose a lane that fits this conservative rectangle check. Check heading and clearance physically too.
- In `BiobuzzAutoRoute.java`, assign the measured values to `RED` or `BLUE` using the constructor below. Initially both values are null. Survey each alliance separately. Use the whole chassis length and width. For the centred rectangle the starting X is half the chassis length.

```java
new BiobuzzAutoRoute(length, width,
        new BiobuzzAutoRoute.Point(startX, 0),
        new BiobuzzAutoRoute.Point(leaveX, 0),
        new BiobuzzAutoRoute.Point(parkX, parkY),
        new BiobuzzAutoRoute.Bounds(zoneMinX, zoneMinY, zoneMaxX, zoneMaxY),
        new BiobuzzAutoRoute.Bounds(clearMinX, clearMinY, clearMaxX, clearMaxY),
        leaveTimeoutSeconds, parkTimeoutSeconds)
```

### Run and qualify

1. Confirm this year's directions, encoder scales, and the short Pedro straight test on the same surface. For the first full-route practice, use provisional 10 s LEAVE and 12 s PARK timeouts on clear floor. After measuring clean runs, set each to the slowest clean duration plus 1 s. The sum plus the 0.3 s settle period must fit by 25 s. The OpMode holds power to 25% and ends powered motion by the 28 s loop deadline.
2. Select **BIOBUZZ: Leave and Park**, press INIT, then choose red with D-pad left or blue with D-pad right. Verify the displayed start and parking target. Confirm the assigned start and parking position with **A** before the countdown begins. Changing alliance clears confirmation. Press START at the match signal.
3. Verify a LEAVE → PARK → SETTLE → DONE sequence in telemetry. Target tolerance is 25 mm and 3°. At SETTLE, the motor outputs are zero with brake while pose-only updates check that the estimate stays within tolerance for 0.3 s. Stop, invalid estimate, departure from the surveyed corridor, timeout or early follower finish outside tolerance ends the routine with no corrective move. Transition reasons are also in Robot Controller logs under `BioBuzzAuto`. An encoder estimate is not proof of an actual score.
4. Test 10 runs per alliance on the actual or representative surface. Vary the legal lateral start up to 20 mm and use the normal battery range. Require at least 9 of 10 per alliance with wall clearance and loading-zone overlap by 25 s, still scoring at the 30 s video frame, and zero contacts. Run three partner trials per alliance and check both footprints at 30 s. On clear practice floor verify early Driver Station Stop and a deliberately too-short segment timeout. Restore the qualified timeout before matches.
5. This code targets **8 AUTO points** when the two physical criteria pass. Keep the future launcher route separate until it has passed physical Hive tip and parking trials.

Route and timing checks are in `tools/auto-tests/BiobuzzAutoChecks.java` and can be run with any JDK 17+ without a robot or Android SDK:

```sh
src=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
classes=$(mktemp -d)
javac -d "$classes" "$src/BiobuzzAutoRoute.java" "$src/BiobuzzAutoSequence.java" tools/auto-tests/BiobuzzAutoChecks.java
java -cp "$classes" org.firstinspires.ftc.teamcode.BiobuzzAutoChecks
```

## Powered encoder measurements with Drive Calibration

On a clear floor, tape the start centre and heading. Select **Drive Calibration**, press INIT and START, then centre the sticks and press **A** with the right bumper released. This zeros the display for one trial without resetting the motor encoders.

Hold the right bumper and use the normal sticks to drive at up to 20% power. For a calibration run, command one axis at a time. Release the bumper to brake. Wait for **RESULT**, then copy the four signed tick changes and measure the actual displacement or turn. The result stays on screen even if you reposition the robot. Centre the sticks and press A with the bumper released to begin the next trial. Use the scale formulas above. If it stays at **SETTLING**, check that the robot has stopped and the encoder readings are steady before recording results.
