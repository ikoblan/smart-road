# Algorithm and safety model

## Coordinates and paths

The visible world is [-60, 60] x [-60, 60] metres; positive y points down. The
junction is [-18, 18] x [-18, 18]. Six 6 m-wide lanes cross each road. Incoming
northbound-template offsets are -15 m (right), -9 m (straight), and -3 m (left).

The north approach enters with tangent (0, 1). A right turn is a quarter circle
with centre (-18, -18), radius 3, and angle increasing from 0 to pi/2. A left
turn has centre (18, -18), radius 21, and angle decreasing from pi to pi/2.
Three quarter rotations generate the other nine trajectories.

The path coordinate `distance` is true arc length. Within a circle, angle
changes by `ds / radius`. The tangent is analytic and continuous at the joins.
Consequently `ds = velocity * dt` has the same meaning on straights and turns.
No lane or route is changed after a vehicle is created.

## Footprint and clearance

A 4 x 2 m car sprite fits inside a circle of radius

```text
r = sqrt((4/2)^2 + (2/2)^2) = sqrt(5) = 2.2360679775 m
```

The safety gap is 1 m between enclosing circles. Thus the required centre
separation is `2r + 1 = 5.472135955 m`. An actual rotated rectangle is contained
inside its enclosing circle, making this conservative rather than optimistic.
Parallel lane centres are 6 m apart, leaving more than the required gap.

A following car maintains at least 8 m of arc-length headway, plus numerical
margins. The smallest centre separation at that headway is on the tight right
turn. When the points symmetrically straddle that whole quarter arc, it is

```text
sqrt(2) * (3 + (8 - 3*pi/2)/2) > 5.472135955 m.
```

The left-turn chord at 8 m headway is `42*sin(8/42)`, also above the threshold.
The independent reference check additionally samples all twelve complete
routes at that headway.

## Why a conflict matrix is sufficient here

The admission gate is 6.5 m before the physical junction boundary, and the
release boundary is 6.5 m beyond its far boundary along each route. Waiting
vehicles are kept upstream of the gate. The central arcs stay inside the
physical junction. Outside it, inbound and dedicated outbound lanes are
parallel with sufficient separation. Same-route following is handled
independently and there are no downstream cross-route lane merges.

The conflict matrix compares the **whole protected corridor** from gate to
release, not just the two intersection centre lines. It uses minimum distances
between polyline segments. Sampling every at most 0.5 m on a radius-R arc gives
a maximum chord deviation bounded by `h^2/(8R)`. With minimum R = 3 m, the sum
of two such errors is at most 0.020834 m. The additional conflict margin is
0.10 m, larger than that bound.

Two different routes are compatible only if their protected corridors remain
farther apart than both vehicle radii, the safety gap, and that margin.
Therefore their relative entry times need not be optimized for geometric
safety. This makes the policy simple and conservative, but reduces throughput
compared with accurate space-time reservations at individual conflict points.

The diagonal of the geometric conflict matrix is true. The scheduler handles
same-route vehicles as a special case: they may share the corridor only
because they are kept ordered and separated by the independent following
controller. Other conflicting routes remain blocked until the **last** active
reservation on the shared corridor is released.

## Scheduling and fairness

Only the leading unadmitted car on each route can request permission. Its
request becomes eligible within 10 m of the admission gate. Eligible requests
receive increasing tickets. On each tick, the scheduler considers these tickets
in ascending order and grants compatible requests.

If a request is blocked, the oldest such request creates a fairness barrier:
younger requests conflicting with it cannot be admitted. Nonconflicting
requests can still use otherwise idle corridors. Only one barrier is created;
propagating barriers through all blocked requests would unnecessarily serialize
traffic.

Under the assumptions that already-admitted vehicles eventually leave, all
vehicles cooperate, and the downstream road remains unobstructed, this prevents
the oldest waiting request from being perpetually displaced by fresh conflicting
traffic. It does not claim that every route gets equal throughput or that all
waiting-time distributions are optimal.

## Motion and following

A snapshot is taken before every fixed 1/120-second update. Each car determines
its setpoint, the nearest same-route leader, and any ungranted gate limit using
that same snapshot. The leader is treated conservatively as a stationary
obstacle when calculating an allowable speed.

For available room d and braking magnitude b, require

```text
v*dt + v^2/(2b) <= d
v_safe = b * (sqrt(dt^2 + 2*d/b) - dt)
```

This accounts for both the next movement step and full stopping distance. The
normal integrator changes speed by at most `acceleration*dt` upward and
`braking*dt` downward. It then advances by `v_next*dt`. A final distance cap is
an explicit fail-safe. Any braking beyond the normal bound is counted as a
hard-brake override; it is not silently described as realistic acceleration.

The removal boundary is not a braking target. A vehicle exits at speed and is
removed once its whole conservative envelope is outside the canvas. The exact
exit time is interpolated within the last tick.

## Independent swept monitor

For centres A and B moving over one tick, define

```text
r0 = A0 - B0
u  = (A1 - A0) - (B1 - B0)
t* = clamp(-dot(r0,u) / dot(u,u), 0, 1)
minimum chord-interpolated separation = length(r0 + t* u)
```

Zero relative motion is handled separately. To cover actual curved motion,
the monitor subtracts `(ds_A^2 + ds_B^2)/(8*R_min)` before subtracting the body
radii. A gap <= 1 m starts or continues a close-call episode. A negative gap
starts or continues a conservative collision episode. A continuing pair is
counted once, not once per simulation tick.

The monitor measures violations; it is not what grants crossing permission.
Safety prevention comes from the geometric reservations and following limits.
A nonzero monitor result must be investigated, not cleared or hidden.

## Limits

This is a kinematic, ideal-state educational model. It does not model steering
actuator dynamics, a bicycle model, wheel slip, sensing uncertainty, wireless
latency, pedestrians, emergencies, or uncooperative vehicles. Cornering speeds
are chosen simulation parameters, not validated road-driving recommendations.
Changing lane width, car size, gap, path geometry, or integration timestep
requires rechecking all geometric bounds and rerunning the tests.
