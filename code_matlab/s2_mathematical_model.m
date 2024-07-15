%% Intro
% Copyright 2024 Wojciech Babicki (wojciech.babicki@op.pl)

% Deriving state equations describing a two-wheeled balancing mobile robot.

% Input: T_theta T_psi
% State variables: x_v, v_v_x, theta_v_y, D_theta_v_y, psi_v_z, D_psi_v_z

% Convertion torque into volatage for motors and including friction forces
% (like static and viscotic) is added outside this model

%% prepare workspace
clear all
clc
close all

%% mathematical model (unlinear and 

% Time
syms t real

% Pitch angle of the vehicle and its derivative with respect to time.
syms theta_v_y(t)
D_theta_v_y = diff(theta_v_y, t)

% Yaw angle of the vehicle and its derivative with respect to time.
syms psi_v_z(t)
D_psi_v_z = diff(psi_v_z, t)

% Roll angle of the vehicle is zero, phi_x = 0.
% We assume the wheels never lose contact with the ground and the tires do not deform.

% Integral with respect to time of the abscissa component v_v_x of the velocity of the point
% lying in the middle of the segment connecting the wheels:
syms x_v(t)

% Abscissa component of the velocity of the point in the middle of the segment connecting the wheels
% syms v_v_x real
v_v_x = diff(x_v, t)

% Components: ordinate and altitude of the velocity of the center of the segment connecting the wheels
% are zero (assuming no wheel slippage and the wheels do not lose contact with the ground).
% v_axle_y = 0, v_axle_z = 0

% Geometric dimensions of the vehicle and its wheels.
syms r D L positive
% R - wheel radius
% D - the distance between wheel centers
% L - the distance between the axle and the center of gravity of the vehicle
% We assume that for theta_y = 0 the vehicle's CoG is located right above the wheels' axles.

% Principal moments of inertia of the vehicle (cart), excluding wheels
syms J_v_x J_v_y J_v_z positive 
% Let us assume that products of inertia are all equal to zero.
% Mass (weight) of the vehicle (cart), excluding wheels
syms m_v positive

% Principal moments of inertia of a wheel.
syms J_w_xz J_w_y positive 
% Let us assume that products of inertia are all equal to zero.
% Mass (weight) of a single wheel
syms m_w positive

% Gravity of Earth
syms g positive

% Torques produced by motor reducers driving the wheels.
% Sum and difference of torques driving the wheels.
syms T_theta(t) T_psi(t)
% syms T_L T_R real
% syms T_L(t) T_R(t)
T_L = (T_theta - T_psi) / 2
T_R = (T_theta + T_psi) / 2

% Components of the velocity of the center of gravity of the vehicle (cart).
v_cog_x = v_v_x + L * D_theta_v_y * cos(theta_v_y)
v_cog_y = L * D_psi_v_z * sin(theta_v_y)
v_cog_z = -L * D_theta_v_y * sin(theta_v_y)

% Velocities of the wheels' centers of gravity.
v_R_x = v_v_x + D / 2 * D_psi_v_z
v_L_x = v_v_x - D / 2 * D_psi_v_z

% Angles of rotation of the wheels.
theta_wR_y = x_v / r + psi_v_z * D / 2 / r
theta_wL_y = x_v / r - psi_v_z * D / 2 / r
% These angles are not equal to the angles measured by motor encoders.

omega_R_y = v_R_x / r
omega_L_y = v_L_x / r

E_ws_kinetic_translational = 1/2 * m_w * v_R_x^2 + 1/2 * m_w * v_L_x^2

E_ws_kinetic_rotational = 1/2 * J_w_xz * D_psi_v_z^2 + 1/2 * J_w_y * omega_R_y^2 + 1/2 * J_w_xz * D_psi_v_z^2 + 1/2 * J_w_y * omega_L_y^2 

E_vehicle_kinetic_translational = 1/2 * m_v * v_cog_x^2 + 1/2 * m_v * v_cog_y^2 + 1/2 * m_v * v_cog_z^2

E_vehicle_kinetic_rotational = 1/2 * J_v_x * (-D_psi_v_z * sin(theta_v_y))^2  + 1/2 * J_v_y * (D_theta_v_y)^2 + 1/2 * J_v_z * (D_psi_v_z * cos(theta_v_y))^2

E_vehicle_potential = m_v * g * L * cos(theta_v_y)

E_servomechanisms_potential = -T_R * (theta_wR_y - theta_v_y) - T_L * (theta_wL_y - theta_v_y)

lagrangian = E_ws_kinetic_translational + E_ws_kinetic_rotational ...
    + E_vehicle_kinetic_translational + E_vehicle_kinetic_rotational ...
    - E_vehicle_potential - E_servomechanisms_potential

lagrangian = simplify(lagrangian)

E1 = diff(diff(lagrangian, v_v_x), t) - diff(lagrangian, x_v) == 0

E2 = diff(diff(lagrangian, D_psi_v_z), t) - diff(lagrangian, psi_v_z) == 0

E3 = diff(diff(lagrangian, D_theta_v_y), t) - diff(lagrangian, theta_v_y) == 0

syms D2_psi_v_z_t D_psi_v_z_t psi_v_z_t real
syms D2_theta_v_y_t D_theta_v_y_t theta_v_y_t real
syms D_v_v_x_t v_v_x_t x_v_t real
syms T_theta_t T_psi_t real

E1 = subs(E1, ...
    {diff(psi_v_z, t, t), diff(psi_v_z, t), psi_v_z, diff(theta_v_y, t, t), diff(theta_v_y, t), theta_v_y, diff(v_v_x, t), v_v_x, x_v, T_theta, T_psi}, ...
    {D2_psi_v_z_t, D_psi_v_z_t, psi_v_z_t, D2_theta_v_y_t, D_theta_v_y_t, theta_v_y_t, D_v_v_x_t, v_v_x_t, x_v_t, T_theta_t, T_psi_t})

E2 = subs(E2, ...
    {diff(psi_v_z, t, t), diff(psi_v_z, t), psi_v_z, diff(theta_v_y, t, t), diff(theta_v_y, t), theta_v_y, diff(v_v_x, t), v_v_x, x_v, T_theta, T_psi}, ...
    {D2_psi_v_z_t, D_psi_v_z_t, psi_v_z_t, D2_theta_v_y_t, D_theta_v_y_t, theta_v_y_t, D_v_v_x_t, v_v_x_t, x_v_t, T_theta_t, T_psi_t})

E3 = subs(E3, ...
    {diff(psi_v_z, t, t), diff(psi_v_z, t), psi_v_z, diff(theta_v_y, t, t), diff(theta_v_y, t), theta_v_y, diff(v_v_x, t), v_v_x, x_v, T_theta, T_psi}, ...
    {D2_psi_v_z_t, D_psi_v_z_t, psi_v_z_t, D2_theta_v_y_t, D_theta_v_y_t, theta_v_y_t, D_v_v_x_t, v_v_x_t, x_v_t, T_theta_t, T_psi_t})

E1 = simplify(E1)
E2 = simplify(E2)
E3 = simplify(E3)

[D_v_v_x_f, D2_theta_v_y_f, D2_psi_v_z_f] = solve(E1, E2, E3, D_v_v_x_t, D2_theta_v_y_t, D2_psi_v_z_t)

D_v_v_x_f = simplify(D_v_v_x_f)
D2_theta_v_y_f = simplify(D2_theta_v_y_f)
D2_psi_v_z_f = simplify(D2_psi_v_z_f)

f = [v_v_x_t; D_v_v_x_f; D_theta_v_y_t; D2_theta_v_y_f; D_psi_v_z_t; D2_psi_v_z_f]
x = [x_v_t; v_v_x_t; theta_v_y_t; D_theta_v_y_t; psi_v_z_t; D_psi_v_z_t]
u = [T_theta_t; T_psi_t]
p = [g, L, D, r, J_v_x, J_v_y, J_v_z, m_v, J_w_xz, J_w_y, m_w]

D_f_D_x = jacobian(f, x);
D_f_D_u = jacobian(f, u);

A_s = subs(D_f_D_x, [x; u], [0; 0; 0; 0; 0; 0; 0; 0])
B_s = subs(D_f_D_u, [x; u], [0; 0; 0; 0; 0; 0; 0; 0])

A_s = simplify(A_s);
B_s = simplify(B_s);

run("s1_parameters.m");

A_n = subs(A_s, ...
    {g, L, D, r, J_v_y, J_v_z, m_v, J_w_xz, J_w_y, m_w}, ...
    {val_g, val_L, val_D, val_r, val_J_v_y, val_J_v_z, val_m_v, ...
    val_J_w_xz, val_J_w_y, val_m_w})

B_n = subs(B_s, ...
    {g, L, D, r, J_v_y, J_v_z, m_v, J_w_xz, J_w_y, m_w}, ...
    {val_g, val_L, val_D, val_r, val_J_v_y, val_J_v_z, val_m_v, ...
    val_J_w_xz, val_J_w_y, val_m_w})

A = double(A_n)
B = double(B_n)
% C = [1 1 1 1 1 1]

eig(A)
rank(ctrb(A, B))
% rank(obsv(A, C))

Q = [1 , 0 , 0 , 0 , 0 , 0 ,
     0 , 1 , 0 , 0 , 0 , 0 ,
     0 , 0 , 1 , 0 , 0 , 0 ,
     0 , 0 , 0 , 1 , 0 , 0 ,
     0 , 0 , 0 , 0 , 1 , 0 ,
     0 , 0 , 0 , 0 , 0 , 1 ];
R = eye(size(B, 2))
K = lqr(A, B, Q, R)

eig(A - B * K);

[x_v_0, v_v_x_0, theta_v_y_0, D_theta_v_y_0, psi_v_z_0, D_psi_v_z_0] = ...
    deal(0.2, 0, 0.04, 0, 1, 0);

x0 = [x_v_0; v_v_x_0; theta_v_y_0; D_theta_v_y_0; psi_v_z_0; D_psi_v_z_0];
