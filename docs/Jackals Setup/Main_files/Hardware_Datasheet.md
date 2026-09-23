# Hardware Datasheet — Base Vehicle

Both Jackal 93 (J100-0893) and Jackal 96 (J100-0896) are built on the stock **Clearpath Jackal UGV** platform. The specifications below are manufacturer-published, prior to CRAL sensor/compute additions. See [Integration](Integration.md) for what CRAL adds on top of this base, and [User Manual](User_Manual.md) for power-on/connection procedures.

## Dimensions & Weight

| | |
|---|---|
| **Length** | 508 mm (20 in) |
| **Width** | 430 mm (17 in) |
| **Height** | 250 mm (10 in) |
| **Ground clearance** | 65 mm (2.6 in) |
| **Weight** | 17 kg (37 lb) |
| **Maximum payload** | 20 kg (44 lb) — 10 kg (22 lb) rated for all-terrain use |

## Performance

| | |
|---|---|
| **Maximum speed** | 2.0 m/s (6.6 ft/s) |
| **Drivetrain** | High-torque 4x4 skid-steer |
| **Wheel diameter** | 194 mm |
| **Operating temperature** | -19°C to +45°C |
| **Environmental rating** | IP62 weatherproof casing |

## Power

| | |
|---|---|
| **Battery** | Lithium-ion, 270 Wh |
| **Run time** | 8 hrs (basic usage) / 2 hrs (heavy usage) |
| **Charge time** | ~4 hours |
| **User power available** | 16 A @ VBAT (25.6 V nominal), 7 A @ 12 V, 5 A @ 5 V |

## Stock Electronics & Chassis (as shipped — superseded on CRAL vehicles, see [Integration](Integration.md))

| | |
|---|---|
| **Onboard computer** | Ubuntu-based PC (Intel or Jetson option) + 32-bit microcontroller |
| **Communication** | Ethernet, USB 3.0 |
| **Built-in sensing** | Wheel odometry, IMU, GPS, battery status, motor currents |
| **Chassis** | Aluminum construction, IP62 weatherproof casing |

*Source: manufacturer-published specifications, [clearpathrobotics.com/jackal-small-unmanned-ground-vehicle](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) and [docs.clearpathrobotics.com](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal/).*
