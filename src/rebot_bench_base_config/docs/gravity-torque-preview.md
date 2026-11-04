# Leaned-back gravity torque preview

These values are predictions of the first static fit, **not measured torque at
these poses or physical load limits**. They precede the leaned-back data collection.
Units: joint positions rad; torque Nm, joint1…joint6, before gains/offsets.
Source calibration SHA256:
`379a31256148ed2323ffc6197db25c32e0adf5ad367a425dd5f443f295960b69`.

| Waypoint | J2 | J3 | J4 | Predicted torque J1…J6 (Nm) | J2/J3 exceeds9Nm |
|---|---:|---:|---:|---|---|
| Gravity 25 | 1.300 | 1.4726 | -1.3712 | 0.000 -3.927 6.061 1.045 -0.085 0.014 | none |
| Gravity 26 | 1.300 | 1.9948 | 0.8157 | 0.000 -0.764 2.898 -0.075 0.016 0.003 | none |
| Gravity 27 | 1.300 | 2.8346 | -0.8518 | 0.000 2.172 -0.038 1.617 -0.046 0.035 | none |
| Gravity 28 | 1.300 | 3.6432 | 0.7958 | 0.000 9.513 -7.379 -2.274 0.000 -0.044 | J2 |
| Gravity 29 | 1.600 | 1.2332 | -1.2206 | 0.000 -6.407 5.920 0.203 0.021 0.001 | none |
| Gravity 30 | 1.600 | 1.9362 | 0.9205 | 0.000 -5.458 4.972 0.472 0.112 0.016 | none |
| Gravity 31 | 1.600 | 2.8862 | -0.8571 | 0.000 -2.212 1.726 1.983 0.004 0.034 | none |
| Gravity 32 | 1.600 | 3.7851 | 0.8051 | 0.000 6.423 -6.909 -2.275 -0.007 -0.036 | none |
| Gravity 33 | 1.900 | 1.2650 | -1.3483 | 0.000 -7.818 4.754 -0.703 -0.036 -0.018 | none |
| Gravity 34 | 1.900 | 2.0127 | 0.9768 | 0.000 -9.072 6.008 0.837 -0.061 0.017 | J2 |
| Gravity 35 | 1.900 | 2.9349 | -0.7811 | 0.000 -6.385 3.321 2.149 -0.001 0.040 | none |
| Gravity 36 | 1.900 | 3.6974 | 0.7732 | 0.000 1.965 -5.028 -2.008 0.059 -0.036 | none |
| Gravity 37 | 2.150 | 1.7519 | -1.2994 | 0.000 -10.632 5.620 -0.088 -0.049 -0.001 | J2 |
| Gravity 38 | 2.150 | 1.8353 | 0.7607 | 0.000 -12.683 7.672 1.951 -0.007 0.040 | J2 |
| Gravity 39 | 2.150 | 2.9489 | -0.7197 | 0.000 -9.708 4.697 2.248 0.001 0.041 | J2 |
| Gravity 40 | 2.150 | 3.7373 | 0.7800 | 0.000 -1.303 -3.708 -1.767 -0.038 -0.024 | none |
| Gravity 41 | 2.300 | 1.9179 | -1.2724 | 0.000 -11.793 5.756 0.042 -0.057 -0.004 | J2 |
| Gravity 42 | 2.300 | 2.0405 | 0.9083 | 0.000 -13.399 7.362 1.654 0.043 0.033 | J2 |
| Gravity 43 | 2.300 | 3.0137 | -0.8021 | 0.000 -11.193 5.155 2.275 0.007 0.042 | J2 |
| Gravity 44 | 2.300 | 3.6878 | 0.8183 | 0.000 -3.650 -2.387 -1.552 -0.040 -0.026 | none |
| Gravity 45 | 2.400 | 2.0491 | -0.9857 | 0.000 -13.110 6.463 0.742 -0.077 0.011 | J2 |
| Gravity 46 | 2.400 | 2.0662 | 0.8894 | 0.000 -14.193 7.545 1.823 0.026 0.027 | J2 |
| Gravity 47 | 2.400 | 2.8050 | -0.6967 | 0.000 -13.145 6.498 2.251 -0.005 0.035 | J2 |
| Gravity 48 | 2.400 | 3.6912 | 0.8307 | 0.000 -5.008 -1.639 -1.353 0.085 -0.025 | none |
| Gravity Elbow Clearance | 1.900 | 2.0127 | -1.3483 | 0.000 -9.197 6.134 0.962 -0.038 0.015 | J2 |
| Gravity Retreat | 1.300 | 3.6912 | 0.8307 | 0.000 9.591 -7.456 -2.234 -0.008 -0.034 | J2 |

The [vendor electrical specifications](https://robstride.com/) list RS06 rated
load11Nm and peak36Nm, RS00 rated load5Nm and peak14Nm (accessed2026-11-01).
Rated load is specified at100rpm; it is not an established indefinite zero-speed
holding rating for this installation. J2/J3 use RS06 and J4 uses RS00.
J2 at Gravity38,41,42,43,45,46,47 exceeds even that11Nm rated load. The SDK's36Nm
encoding range is not a continuous torque rating.

A sampled/refined search of the fitted model over joint limits found reachable
collision-free gravity magnitudes16.823Nm(J2),8.016Nm(J3),2.294Nm(J4). These are
maxima found, not certified global upper bounds. Even the J2 lower bound rules
out a full-workspace clamp that is both above gravity with margin and below the
11Nm vendor rated load. No clamp change is part of the collection or fitting
utility; keep the100°C cutoff and HOME-before-disable policy. Normal position
control and brief collection are not proof of sustainable hands-off float.
