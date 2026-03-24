
  Do you need wheel encoders for SLAM?                                                                                                                      
  Short answer: No, not strictly. But it depends on what's providing your odometry.                                                                         
  What encoders give you

  Encoders measure actual wheel rotation — ticks per revolution. From that you get real wheel velocity, and from that, linear velocity (vx) and yaw rate.
  This is a direct measurement of what the wheels actually did, not what you told them to do.

  What you have right now (without encoders)

  Your EKF has two sources:

  1. IMU gyro → yaw rate (real measurement, good)
  2. cmd_vel odom → vx and yaw rate (just echoing commands back, not a measurement at all)

  The critical gap is linear velocity (vx). Right now your only source for "how fast am I moving forward" is the cmd_vel echo. If you command 0.3 m/s but
  the robot actually moves at 0.2 m/s (battery low, carpet friction, incline), the EKF has no way to know. It just believes the command.

  How slam_toolbox compensates

  Here's the thing — slam_toolbox does its own scan matching. It takes two consecutive laser scans, aligns them, and figures out how the robot moved. This
  is essentially a visual/laser odometry that's quite accurate in feature-rich environments.

  slam_toolbox uses the EKF's odom → base_link TF only as an initial guess for scan matching. If the guess is within a reasonable margin (say 30-50% off),
  the scan matcher will converge to the correct answer anyway.

  So the pipeline is:
  Your odom (rough guess) → slam_toolbox uses it as seed → scan matching refines it → accurate pose

  When encoders DON'T matter much

  - Feature-rich environments (rooms with furniture, distinct walls, corners): Scan matching is strong. Even mediocre odometry is enough as an initial
  guess. This is your current use case — mapping a room.
  - Slow driving: At low speeds, the error between commanded and actual velocity is small per scan cycle. The scan matcher can handle it.
  - Short sessions: Odom drift accumulates over time. If you're mapping a single room in 5 minutes, the drift may stay within tolerable bounds.

  When encoders START to matter

  - Long corridors / featureless walls: The laser sees a flat wall with no distinguishing features. Scan matching can't tell if you moved 0.5m or 0.7m along
   the wall — all scan-to-scan alignments look equally good. Here, accurate linear velocity from encoders is the only thing that keeps the scale correct.
  - Fast driving or aggressive maneuvers: Large gaps between scans. The initial guess needs to be closer to truth for the scan matcher to converge.
  - Large environments: Mapping an entire floor of a building. Small per-step errors compound into meters of drift before a loop closure opportunity. Better
   odom delays when the map starts to "smear."
  - Pure rotation in open space: Spinning in the middle of a room where all walls are far away. Scan matching has low resolution at range. The IMU handles
  yaw well, but if the robot also translates during the spin, you need encoders for that.

  Drawbacks of wheel encoders

  Mechanical complexity:
  - Need encoder discs or magnetic rings mounted to the wheel shafts
  - Precise alignment required — a wobbling encoder disc gives noisy readings
  - More wires, more connectors, more points of failure
  - On a small hobby robot, physical space on the shaft can be tight

  They lie too:
  - Wheel slip: Encoder says the wheel turned 100 ticks, but it was spinning on a slippery surface. The robot didn't actually move that far. Encoders can't
  detect slip.
  - Tread wear: As rubber wears, the effective wheel radius changes. Your ticks-to-meters calibration drifts over weeks.
  - Uneven surfaces: A bump lifts a wheel momentarily — encoder counts but no ground contact.

  Calibration burden:
  - You need to accurately measure ticks-per-revolution, wheel diameter, and wheel separation
  - Small errors in these parameters cause the robot to think it's turning more/less than it is, or traveling faster/slower
  - These parameters change with tire pressure, load, temperature

  Arduino overhead:
  - Quadrature encoders need interrupt-driven counting (2 interrupts per encoder × 2 encoders = 4 interrupt pins)
  - On an Arduino Uno/Nano, you only have 2 hardware interrupt pins (D2, D3) — you'd need to use pin-change interrupts for the other 2, which are messier
  - At high RPM, interrupt frequency can be very high

  My take for your situation

  You're mapping a room with a slow-moving robot. You don't need encoders yet. Here's why:

  1. Your room has walls, furniture, corners — scan matching will work well
  2. The IMU gives good yaw rate (once we fix the bias/calibration)
  3. The cmd_vel linear velocity, while not a true measurement, is a reasonable approximation at low speeds
  4. Adding encoders right now would delay your goal of getting end-to-end SLAM working

  When to add them later: If you start mapping larger spaces (hallways, multiple rooms) and notice the map quality degrading — especially "stretching" or
  "compressing" along long featureless stretches — that's when encoders earn their keep.

  The priority order remains:
  1. Fix IMU (gyro bias calibration) — critical, your rotation is broken
  2. Fix EKF double-counting — quick config change
  3. Get SLAM mapping a room — your stated goal
  4. Add encoders — optimization for later

  Want to proceed with fixing the IMU and EKF issues?