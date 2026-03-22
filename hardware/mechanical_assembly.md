# Mechanical Assembly

This guide walks through building the gimbal frame from aluminum extrusion and plate stock. No CNC machining is required — all cuts can be made with a hand saw or angle grinder. Drilling is the only machining operation needed.

---

## Frame Overview

The structure is a two-axis (pan, tilt) gimbal on a fixed base. The pan axis rotates the entire upper assembly about a vertical axis. The tilt axis rotates the camera-radar payload about a horizontal axis perpendicular to the look direction.

```
Top view (pan axis):

        [Camera + Radar payload]
               |
         [Tilt motor]
               |
         [Pan motor]  <-- rotates on bearings
               |
         [Base plate]  <-- bolted to ground
```

---

## Parts Preparation

Cut the 30x30 aluminum extrusion to the following lengths using a miter saw or angle grinder with a metal cutting disc:

| Piece | Length | Qty | Purpose |
|---|---|---|---|
| Base legs | 150 mm | 4 | Vertical supports lifting the pan motor off the ground |
| Base crossmembers | 200 mm | 2 | Horizontal base frame |
| Tilt arm | 120 mm | 2 | Connects tilt motor to payload plate |

Drill the 4 mm aluminum plate to the following hole pattern (all dimensions from bottom-left corner):

- Pan motor center hole: 100 mm x 100 mm (center of plate)
- Pan motor bolt pattern: M3 holes on 58 mm bolt circle, 4 holes evenly spaced
- Base extrusion bolt holes: 20 mm from each corner, M5

---

## Assembly Sequence

### Step 1 — Base frame

Assemble the four base legs and two crossmembers into a rectangular frame using M5 T-slot nuts and M5x12 button head bolts. Do not fully tighten yet — leave 1/4 turn loose until the frame is squared.

Check squareness by measuring both diagonals. They should be equal within 1 mm. Once square, fully tighten all M5 bolts to 3 Nm.

### Step 2 — Pan motor mounting

The GBM5208 pan motor mounts face-down through the center hole in the base plate. The motor stator (outer can) is fixed to the plate. The shaft points downward and is fixed to the frame — this means the plate and everything above it rotates around the stationary shaft.

Alternatively (simpler): mount the motor stator to the frame and let the rotor (shaft) drive a separate rotating plate above. This is called a "turntable" configuration and is easier to build.

Turntable configuration:
1. Fix motor stator to base plate with M3x10 bolts through the motor mounting holes
2. Press the 6804-2RS bearing onto the motor shaft
3. Fabricate a 3mm aluminum rotating plate (200x200mm) that rests on the bearing outer race
4. Connect rotating plate to motor shaft with a 6mm shaft collar

### Step 3 — Pan encoder mounting

The AMT21 pan encoder mounts to the underside of the base plate, centered on the motor shaft. The encoder body is fixed (does not rotate). The magnet attaches to the motor shaft with a 6mm set-screw collar. Gap between magnet top face and encoder bottom face: 0.5-1.5 mm. Use a 1mm shim to set this gap during assembly.

### Step 4 — Tilt axis

The tilt motor mounts to the rotating plate with the shaft horizontal, pointing toward the camera side. The motor stator is fixed to a bracket on the rotating plate. The rotor drives the tilt arm.

Tilt arm fabrication:
1. Cut two pieces of 2mm aluminum plate, 80mm x 40mm
2. Drill motor shaft holes centered on one end (6mm diameter)
3. Drill camera/radar mount holes on the other end to match your camera's 1/4-20 thread (or M4 threads)

### Step 5 — Tilt encoder

Same procedure as pan encoder. AMT21 body fixed to tilt bracket. Magnet on tilt motor shaft. Maintain 0.5-1.5mm gap.

### Step 6 — Payload mounting

The camera and radar are co-boresighted on the tilt arm. Mount camera first — use the camera's 1/4-20 tripod socket to the tilt arm plate with a 1/4-20 stainless bolt. The radar mounts adjacent to the camera with the antenna array facing the same direction as the camera lens. The radar center antenna should be within 30mm horizontally and 30mm vertically of the camera lens center. Record the exact offset — you will need it during extrinsic calibration.

### Step 7 — Cable management

Run all cables through the center of the extrusion frame where possible. Leave a service loop of approximately 100mm on all rotating axis cables — the pan axis cable bundle will twist as the system rotates. Use a cable drag chain if continuous 360-degree rotation is needed. For a +/- 170 degree pan range, a slack loop is sufficient.

---

## Weight Budget

| Component | Mass (approx) |
|---|---|
| Base frame (extrusion + plate) | 800 g |
| Pan motor GBM5208 | 195 g |
| Tilt motor GBM5208 | 195 g |
| ODrive S1 | 90 g |
| Jetson Orin Nano Dev Kit | 275 g |
| Camera + lens | 120 g |
| AWR1843BOOST radar | 80 g |
| Encoders x2 + IMU | 40 g |
| Wiring and connectors | 120 g |
| PSU (mounted separately) | 850 g |
| **Total (head unit)** | **~1,915 g** |

The head unit (everything above the base) weighs approximately 1.1 kg. This is well within the stall torque of the GBM5208 at 12 Nm — the motor operates at less than 10% of stall torque for repositioning movements.

---

## Center of Mass

Balance the tilt axis by adjusting the camera-radar payload fore/aft on the tilt arm until the assembly is neutral — it should hold any angle without motor power applied. An unbalanced tilt axis forces the motor to hold torque continuously, wastes power, and reduces dynamic response. Test balance by powering off the ODrive and checking whether the tilt arm swings to one end. Adjust the payload mounting position by up to 20mm fore/aft until balance is achieved.
