"""
The Scout Flight Controller (Modified for MAVLink UDP Control by John Seargeant)
by Tim Hanewich - github.com/TimHanewich
For more information: https://github.com/TimHanewich/scout

Copyright 2023 Tim Hanewich
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

########################################
###########   SETTINGS   #################
########################################

# --- Flight Controller Core Settings ---
# Motor GPIO's (not pin number, GPIO number) ###
gpio_motor1 = 2 # front left, clockwise
gpio_motor2 = 28 # front right, counter clockwise
gpio_motor3 = 15 # rear left, counter clockwise
gpio_motor4 = 16 # rear right, clockwise

# i2c pins used for MPU-6050
gpio_i2c_sda = 12
gpio_i2c_scl = 13

# throttle settings
throttle_idle:float = 0.14 # minimum throttle for motors to spin (no lift)
throttle_governor:float = 0.40 # Max throttle percentage (adjust for safety/power)

# Max attitude rate of change rates (degrees per second)
max_rate_roll:float = 30.0 # roll
max_rate_pitch:float = 30.0 # pitch
max_rate_yaw:float = 50.0 # yaw

# Desired Flight Controller Cycle time (Hz)
target_cycle_hz:float = 250.0

# PID Controller values (These likely need retuning for MAVLink control feel)
pid_roll_kp:float = 0.00043714285
pid_roll_ki:float = 0.00255
pid_roll_kd:float = 0.00002571429
pid_pitch_kp:float = pid_roll_kp
pid_pitch_ki:float = pid_roll_ki
pid_pitch_kd:float = pid_roll_kd
pid_yaw_kp:float = 0.001714287
pid_yaw_ki:float = 0.003428571
pid_yaw_kd:float = 0.0

# --- MAVLink Settings ---
MAVLINK_SYSTEM_ID = 1   # ID of this system (the drone)
MAVLINK_COMPONENT_ID = 1 # ID of this component (autopilot)
QGC_COMPUTER_IP = 'YOUR_QGC_COMPUTER_IP' # <<<--- IMPORTANT: SET THIS!!!
QGC_UDP_PORT = 14550 # Standard QGroundControl UDP port for receiving telemetry
PICO_UDP_PORT = 14551 # Port the Pico will listen on for commands from QGC

HEARTBEAT_RATE_HZ = 1
ATTITUDE_RATE_HZ = 20 # Send attitude data at 20Hz (adjust as needed)
# SCALED_IMU_RATE_HZ = 50 # Optional: Send IMU data more frequently

RC_OVERRIDE_TIMEOUT_MS = 1000 # Time in ms without RC Override msg before considering link lost

########################################
########################################
########################################

import machine
import time
import toolkit # Your existing toolkit
import math
import socket
import network # For Wi-Fi status checking
import errno   # For non-blocking socket errors

# Import MAVLink libraries
import mavlink
import mavcrc # Though mavlink.py uses it internally

# --- Global State Variables ---
is_armed:bool = False
last_rc_override_time_ms:int = 0
input_throttle_mav:float = 0.0 # MAVLink normalized throttle [0.0, 1.0]
input_roll_mav:float = 0.0     # MAVLink normalized roll [-1.0, 1.0]
input_pitch_mav:float = 0.0    # MAVLink normalized pitch [-1.0, 1.0]
input_yaw_mav:float = 0.0      # MAVLink normalized yaw rate [-1.0, 1.0]

# --- MAVLink Instance and Socket ---
mav: mavlink.MAVLink = None
udp_socket: socket.socket = None
qgc_address = None

# --- PID State Variables --- (Moved here to be accessible in FATAL_ERROR)
roll_last_integral:float = 0.0
roll_last_error:float = 0.0
pitch_last_integral:float = 0.0
pitch_last_error:float = 0.0
yaw_last_integral:float = 0.0
yaw_last_error:float = 0.0

# --- Motor PWM Objects --- (Moved here for FATAL_ERROR access)
M1:machine.PWM = None
M2:machine.PWM = None
M3:machine.PWM = None
M4:machine.PWM = None


# THE FLIGHT CONTROL LOOP
def run() -> None:
    global is_armed, last_rc_override_time_ms
    global input_throttle_mav, input_roll_mav, input_pitch_mav, input_yaw_mav
    global mav, udp_socket, qgc_address
    global roll_last_integral, roll_last_error, pitch_last_integral, pitch_last_error, yaw_last_integral, yaw_last_error
    global M1, M2, M3, M4 # Allow modification of global PWM objects

    print("Hello from Scout MAVLink!")

    # Onboard LED for status
    led = machine.Pin("LED", machine.Pin.OUT) # Use "LED" for Pico W onboard LED
    for x in range(4): # Faster flashing
        led.on()
        time.sleep(0.05)
        led.off()
        time.sleep(0.05)

    # Check Wi-Fi Connection (relies on boot.py having connected)
    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
       FATAL_ERROR("WiFi not connected. Ensure boot.py runs successfully.")
    print("WiFi Connected. IP:", wlan.ifconfig()[0])
    led.on() # LED on solid if WiFi is connected

    # --- Overclock ---
    try:
        machine.freq(250000000)
        print("Attempted overclock to 250MHz")
    except Exception as e:
        print(f"Warning: Could not overclock - {e}")

    # --- MAVLink and Network Setup ---
    if QGC_COMPUTER_IP == 'YOUR_QGC_COMPUTER_IP':
        FATAL_ERROR("QGC_COMPUTER_IP not set in the script!")

    mav = mavlink.MAVLink(srcSystem=MAVLINK_SYSTEM_ID, srcComponent=MAVLINK_COMPONENT_ID)
    qgc_address = (QGC_COMPUTER_IP, QGC_UDP_PORT)

    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(('', PICO_UDP_PORT)) # Listen on all interfaces on our port
        udp_socket.setblocking(False) # Make socket non-blocking
        print(f"MAVLink UDP socket bound to port {PICO_UDP_PORT}")
        print(f"Sending telemetry to {QGC_COMPUTER_IP}:{QGC_UDP_PORT}")
    except Exception as e:
        FATAL_ERROR(f"Failed to create or bind UDP socket: {e}")

    # --- IMU Setup ---
    print("Waiting 1 second for IMU to settle...")
    time.sleep(1)
    i2c = machine.I2C(0, sda = machine.Pin(gpio_i2c_sda), scl = machine.Pin(gpio_i2c_scl))
    mpu6050_address:int = 0x68
    try:
        i2c.writeto_mem(mpu6050_address, 0x6B, bytes([0x01])) # wake it up
        i2c.writeto_mem(mpu6050_address, 0x1A, bytes([0x05])) # set low pass filter
        i2c.writeto_mem(mpu6050_address, 0x1B, bytes([0x08])) # set gyro scale

        # Confirm IMU setup
        whoami = i2c.readfrom_mem(mpu6050_address, 0x75, 1)[0]
        lpf = i2c.readfrom_mem(mpu6050_address, 0x1A, 1)[0]
        gs = i2c.readfrom_mem(mpu6050_address, 0x1B, 1)[0]

        if whoami != 104: FATAL_ERROR(f"MPU-6050 WHOAMI failed! '{whoami}' returned.")
        if lpf != 0x05: FATAL_ERROR(f"MPU-6050 LPF did not set correctly. Set to '{lpf}'")
        if gs != 0x08: FATAL_ERROR(f"MPU-6050 Gyro Scale did not set correctly. '{gs}' returned.")
        print("MPU-6050 Initialized Successfully.")

    except OSError as e:
        FATAL_ERROR(f"I2C communication error during MPU6050 setup: {e}")
    except Exception as e:
        FATAL_ERROR(f"Error setting up MPU6050: {e}")

    # --- Measure Gyro Bias ---
    print("Measuring gyro bias...")
    gxs, gys, gzs = [], [], []
    measure_start_ms = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), measure_start_ms) < 2000: # Measure for 2 seconds
        try:
            gyro_data = i2c.readfrom_mem(mpu6050_address, 0x43, 6)
            gyro_x = (translate_pair(gyro_data[0], gyro_data[1]) / 65.5)
            gyro_y = (translate_pair(gyro_data[2], gyro_data[3]) / 65.5)
            gyro_z = (translate_pair(gyro_data[4], gyro_data[5]) / 65.5) * -1 # Inverted Z
            gxs.append(gyro_x)
            gys.append(gyro_y)
            gzs.append(gyro_z)
            time.sleep_ms(20) # ~50Hz sampling
        except OSError as e:
            print(f"Warning: I2C read error during gyro bias calc: {e}")
            time.sleep_ms(20) # Avoid busy-looping on error
    if not gxs or not gys or not gzs:
        FATAL_ERROR("Failed to collect gyro data for bias calculation.")
    gyro_bias_x = sum(gxs) / len(gxs)
    gyro_bias_y = sum(gys) / len(gys)
    gyro_bias_z = sum(gzs) / len(gzs)
    print(f"Gyro bias: X={gyro_bias_x:.3f}, Y={gyro_bias_y:.3f}, Z={gyro_bias_z:.3f}")

    # --- Motor PWM Setup ---
    try:
        M1 = machine.PWM(machine.Pin(gpio_motor1))
        M2 = machine.PWM(machine.Pin(gpio_motor2))
        M3 = machine.PWM(machine.Pin(gpio_motor3))
        M4 = machine.PWM(machine.Pin(gpio_motor4))
        M1.freq(250)
        M2.freq(250)
        M3.freq(250)
        M4.freq(250)
        print("Motor PWMs set up @ 250 Hz")
        # Ensure motors are off initially
        duty_0_percent = calculate_duty_cycle(0.0)
        M1.duty_ns(duty_0_percent)
        M2.duty_ns(duty_0_percent)
        M3.duty_ns(duty_0_percent)
        M4.duty_ns(duty_0_percent)
    except Exception as e:
        FATAL_ERROR(f"Failed to setup Motor PWMs: {e}")

    # --- Constants & Timing ---
    cycle_time_seconds:float = 1.0 / target_cycle_hz
    cycle_time_us:int = int(cycle_time_seconds * 1_000_000)
    max_throttle = throttle_governor if throttle_governor is not None else 1.0
    throttle_range:float = max_throttle - throttle_idle
    i_limit:float = 150.0 # PID I-term limiter

    # MAVLink timing
    last_heartbeat_ms:int = 0
    heartbeat_interval_ms:int = int(1000 / HEARTBEAT_RATE_HZ)
    last_attitude_ms:int = 0
    attitude_interval_ms:int = int(1000 / ATTITUDE_RATE_HZ)
    # last_scaled_imu_ms:int = 0 # Optional IMU timing
    # scaled_imu_interval_ms:int = int(1000 / SCALED_IMU_RATE_HZ) # Optional

    # Initial state
    is_armed = False # Start disarmed
    last_rc_override_time_ms = time.ticks_ms() # Avoid immediate timeout

    # --- INFINITE LOOP ---
    print("-- BEGINNING FLIGHT CONTROL LOOP --")
    led.off() # Turn off LED, will blink on heartbeat

    try:
        while True:
            loop_begin_us:int = time.ticks_us()
            current_time_ms:int = time.ticks_ms() # Cache current time

            # --- 1. Read Sensors ---
            gyro_x_raw, gyro_y_raw, gyro_z_raw = 0.0, 0.0, 0.0
            try:
                gyro_data = i2c.readfrom_mem(mpu6050_address, 0x43, 6)
                # Gyro rates in degrees/second, applying bias correction and orientation adjustments
                gyro_x_raw = ((translate_pair(gyro_data[0], gyro_data[1]) / 65.5) - gyro_bias_x) * -1 # Roll rate (inverted)
                gyro_y_raw = (translate_pair(gyro_data[2], gyro_data[3]) / 65.5) - gyro_bias_y      # Pitch rate
                gyro_z_raw = ((translate_pair(gyro_data[4], gyro_data[5]) / 65.5) * -1) - gyro_bias_z # Yaw rate (inverted)
            except OSError as e:
                print(f"Warning: I2C read error in loop: {e}")
                # Maybe implement a counter? If too many errors, trigger failsafe.
                # For now, we'll use the last valid reading (or zero if none yet)
                pass # Use previous/zero values on error

            # --- 2. Receive MAVLink Commands (Non-Blocking) ---
            try:
                while True: # Process all available packets
                    in_data, sender_addr = udp_socket.recvfrom(280) # MAVLink max packet size ~280
                    if in_data:
                        msgs = mav.parse_buffer(in_data)
                        if msgs:
                            for msg in msgs:
                                handle_mavlink_message(msg, sender_addr) # Process the message
                    else:
                        break # No more data in buffer for now
            except OSError as e:
                if e.errno != errno.EAGAIN: # EAGAIN means no data available, which is normal
                   print(f"UDP recv error: {e}")
            except Exception as e:
                 print(f"Error processing incoming MAVLink: {e}")


            # --- 3. Check RC Link Timeout ---
            if time.ticks_diff(current_time_ms, last_rc_override_time_ms) > RC_OVERRIDE_TIMEOUT_MS:
                if is_armed:
                    print("RC Override Timeout! Disarming for safety.")
                    is_armed = False
                    # Optional: Could implement a failsafe landing sequence here
                # Reset inputs to zero when link is lost, even if disarmed
                input_throttle_mav = 0.0
                input_roll_mav = 0.0
                input_pitch_mav = 0.0
                input_yaw_mav = 0.0
                # We still need heartbeats, so don't completely disable MAVLink

            # --- 4. Flight Logic ---
            if is_armed:
                # We are armed, run the PID controllers

                # Calculate adjusted desired throttle (based on MAVLink input)
                # Ensure throttle is within [idle, governor] range
                adj_throttle:float = throttle_idle + (throttle_range * input_throttle_mav)
                adj_throttle = max(throttle_idle, min(adj_throttle, max_throttle))

                # Calculate desired rates based on MAVLink input (scaled by max rates)
                # Note: MAVLink input is already [-1, 1]
                desired_rate_roll = input_roll_mav * max_rate_roll
                desired_rate_pitch = input_pitch_mav * max_rate_pitch
                desired_rate_yaw = input_yaw_mav * max_rate_yaw

                # Calculate errors (Setpoint - Actual)
                error_rate_roll:float = desired_rate_roll - gyro_x_raw
                error_rate_pitch:float = desired_rate_pitch - gyro_y_raw
                error_rate_yaw:float = desired_rate_yaw - gyro_z_raw

                # --- PID Calculations ---
                # Roll PID
                roll_p = error_rate_roll * pid_roll_kp
                roll_i = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds)
                roll_i = max(min(roll_i, i_limit), -i_limit) # Anti-windup
                roll_d = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds
                pid_roll = roll_p + roll_i + roll_d

                # Pitch PID
                pitch_p = error_rate_pitch * pid_pitch_kp
                pitch_i = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds)
                pitch_i = max(min(pitch_i, i_limit), -i_limit) # Anti-windup
                pitch_d = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds
                pid_pitch = pitch_p + pitch_i + pitch_d

                # Yaw PID
                yaw_p = error_rate_yaw * pid_yaw_kp
                yaw_i = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds)
                yaw_i = max(min(yaw_i, i_limit), -i_limit) # Anti-windup
                yaw_d = pid_yaw_kd * (error_rate_yaw - yaw_last_error) / cycle_time_seconds
                pid_yaw = yaw_p + yaw_i + yaw_d

                # --- Motor Mixing ---
                # Calculate individual motor throttle percentages (0.0 to 1.0 conceptually)
                # Clamp results between 0.0 and 1.0 before sending to duty cycle calculation
                t1:float = adj_throttle + pid_pitch + pid_roll - pid_yaw # Front Left (CW)
                t2:float = adj_throttle + pid_pitch - pid_roll + pid_yaw # Front Right (CCW)
                t3:float = adj_throttle - pid_pitch + pid_roll + pid_yaw # Rear Left (CCW)
                t4:float = adj_throttle - pid_pitch - pid_roll - pid_yaw # Rear Right (CW)

                # Clamp motor outputs between 0.0 and 1.0
                t1 = max(0.0, min(t1, 1.0))
                t2 = max(0.0, min(t2, 1.0))
                t3 = max(0.0, min(t3, 1.0))
                t4 = max(0.0, min(t4, 1.0))

                # Apply calculated duty cycles to motors
                M1.duty_ns(calculate_duty_cycle(t1))
                M2.duty_ns(calculate_duty_cycle(t2))
                M3.duty_ns(calculate_duty_cycle(t3))
                M4.duty_ns(calculate_duty_cycle(t4))

                # Save state for next loop's derivative and integral calculations
                roll_last_error = error_rate_roll
                pitch_last_error = error_rate_pitch
                yaw_last_error = error_rate_yaw
                roll_last_integral = roll_i
                pitch_last_integral = pitch_i
                yaw_last_integral = yaw_i

            else: # We are DISARMED
                # Turn motors off completely
                duty_0_percent = calculate_duty_cycle(0.0)
                M1.duty_ns(duty_0_percent)
                M2.duty_ns(duty_0_percent)
                M3.duty_ns(duty_0_percent)
                M4.duty_ns(duty_0_percent)

                # Reset PID integrals and errors to prevent issues when arming
                roll_last_integral = 0.0
                roll_last_error = 0.0
                pitch_last_integral = 0.0
                pitch_last_error = 0.0
                yaw_last_integral = 0.0
                yaw_last_error = 0.0


            # --- 5. Send MAVLink Telemetry (Rate-Limited) ---
            # Heartbeat
            if time.ticks_diff(current_time_ms, last_heartbeat_ms) >= heartbeat_interval_ms:
                last_heartbeat_ms = current_time_ms
                send_heartbeat()
                led.toggle() # Blink LED on heartbeat

            # Attitude
            if time.ticks_diff(current_time_ms, last_attitude_ms) >= attitude_interval_ms:
                last_attitude_ms = current_time_ms
                # Note: ATTITUDE expects radians and rad/s. Gyro values are currently deg/s.
                # We need actual attitude angles (roll, pitch, yaw) for the ATTITUDE message.
                # This requires an AHRS/filter (like complementary or Kalman).
                # For now, we'll send gyro rates *instead* of attitude angles, which isn't correct
                # for the ATTITUDE message but demonstrates sending.
                # TODO: Implement a simple AHRS to get roll/pitch angles.
                # Sending raw rates in attitude fields for now:
                send_attitude(current_time_ms,
                              math.radians(0), # Placeholder for actual roll angle
                              math.radians(0), # Placeholder for actual pitch angle
                              math.radians(0), # Placeholder for actual yaw angle
                              math.radians(gyro_x_raw), # Send roll rate
                              math.radians(gyro_y_raw), # Send pitch rate
                              math.radians(gyro_z_raw)) # Send yaw rate

            # Scaled IMU (Optional)
            # if time.ticks_diff(current_time_ms, last_scaled_imu_ms) >= scaled_imu_interval_ms:
            #     last_scaled_imu_ms = current_time_ms
            #     send_scaled_imu(current_time_ms, gyro_x_raw, gyro_y_raw, gyro_z_raw)


            # --- 6. Maintain Loop Timing ---
            loop_end_us:int = time.ticks_us()
            elapsed_us:int = time.ticks_diff(loop_end_us, loop_begin_us)
            sleep_us:int = cycle_time_us - elapsed_us
            if sleep_us > 0:
                time.sleep_us(sleep_us)
            # else:
            #     # Loop took too long! Consider increasing target_cycle_hz
            #     # or reducing work done per loop (e.g., lower telemetry rates)
            #     # print(f"Warning: Loop overrun by {-sleep_us} us")
            #     pass


    except Exception as e:
        # Attempt to disarm motors before dying
        FATAL_ERROR(f"Unhandled exception in main loop: {e}")


# --- MAVLink Message Handling ---

def handle_mavlink_message(msg, sender_addr):
    """Processes received MAVLink messages."""
    global is_armed, last_rc_override_time_ms
    global input_throttle_mav, input_roll_mav, input_pitch_mav, input_yaw_mav

    msg_id = msg.get_msgId()
    # print(f"Received MAVLink Msg ID: {msg_id} from {sender_addr}") # Debugging

    if msg_id == mavlink.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        # Update last received time
        last_rc_override_time_ms = time.ticks_ms()

        # Extract RAW PWM values (typically 1000-2000, or 0 if channel unused)
        # Mapping assumes standard Mode 2 transmitter:
        # Chan 1: Roll
        # Chan 2: Pitch
        # Chan 3: Throttle
        # Chan 4: Yaw
        roll_pwm = msg.chan1_raw
        pitch_pwm = msg.chan2_raw
        throttle_pwm = msg.chan3_raw
        yaw_pwm = msg.chan4_raw
        # We can use chan5_raw for arming switch if desired, but COMMAND_LONG is preferred
        # arm_switch_pwm = msg.chan5_raw

        # Normalize PWM values (1100-1900 is a common safe range)
        # Throttle [0.0, 1.0]
        input_throttle_mav = normalize(throttle_pwm, 1100.0, 1900.0, 0.0, 1.0)
        # Roll, Pitch, Yaw [-1.0, 1.0] (Centered around 1500)
        input_roll_mav = normalize(roll_pwm, 1100.0, 1900.0, -1.0, 1.0)
        # Pitch needs inversion for standard aircraft convention (forward stick = negative pitch)
        input_pitch_mav = normalize(pitch_pwm, 1100.0, 1900.0, -1.0, 1.0) * -1.0
        input_yaw_mav = normalize(yaw_pwm, 1100.0, 1900.0, -1.0, 1.0)

        # Clamp values just in case PWMs go out of expected range
        input_throttle_mav = max(0.0, min(input_throttle_mav, 1.0))
        input_roll_mav = max(-1.0, min(input_roll_mav, 1.0))
        input_pitch_mav = max(-1.0, min(input_pitch_mav, 1.0))
        input_yaw_mav = max(-1.0, min(input_yaw_mav, 1.0))

        # print(f"RC Override: T={throttle_pwm} R={roll_pwm} P={pitch_pwm} Y={yaw_pwm}") # Debug
        # print(f"Normalized: T={input_throttle_mav:.2f} R={input_roll_mav:.2f} P={input_pitch_mav:.2f} Y={input_yaw_mav:.2f}") # Debug


    elif msg_id == mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
        # Check if it's the ARM/DISARM command
        if msg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.target_system == MAVLINK_SYSTEM_ID: # Ensure command is for us
                ack_result = mavlink.MAV_RESULT_FAILED # Default to failed

                if msg.param1 == 1.0: # Request to ARM
                    # Safety check: Only arm if throttle is low (e.g., < 10%)
                    if input_throttle_mav < 0.10:
                        if not is_armed:
                           print("ARMING command received")
                           is_armed = True
                           ack_result = mavlink.MAV_RESULT_ACCEPTED
                        else:
                            ack_result = mavlink.MAV_RESULT_ACCEPTED # Already armed
                    else:
                        print("Arming denied: Throttle not low")
                        send_statustext("Arming denied: Throttle not low", mavlink.MAV_SEVERITY_WARNING)
                        ack_result = mavlink.MAV_RESULT_TEMPORARILY_REJECTED

                elif msg.param1 == 0.0: # Request to DISARM
                     if is_armed:
                         print("DISARMING command received")
                         is_armed = False
                         ack_result = mavlink.MAV_RESULT_ACCEPTED
                     else:
                         ack_result = mavlink.MAV_RESULT_ACCEPTED # Already disarmed

                else: # Invalid parameter
                    ack_result = mavlink.MAV_RESULT_UNSUPPORTED

                # Send ACK back to QGroundControl
                send_command_ack(msg.command, ack_result)

    elif msg_id == mavlink.MAVLINK_MSG_ID_HEARTBEAT:
        # QGC heartbeat received - could potentially use this to confirm GCS link
        pass # print(f"Heartbeat received from SysID {msg.get_srcSystem()} CompID {msg.get_srcComponent()}")


# --- MAVLink Sending Functions ---

def send_mavlink_message(msg):
    """Encodes and sends a MAVLink message object via UDP."""
    global mav, udp_socket, qgc_address
    if mav is None or udp_socket is None or qgc_address is None:
        print("Warning: MAVLink/Socket not initialized, cannot send message.")
        return
    try:
        buf = msg.pack(mav)
        udp_socket.sendto(buf, qgc_address)
        # print(f"Sent {msg.get_type()}") # Debug
    except Exception as e:
        print(f"Error sending MAVLink message ({msg.get_type()}): {e}")

def send_heartbeat():
    """Sends a MAVLink HEARTBEAT message."""
    # Determine system status
    status = mavlink.MAV_STATE_STANDBY
    if is_armed:
        status = mavlink.MAV_STATE_ACTIVE
    # In a real system, you'd add CALIBRATING, CRITICAL, EMERGENCY etc.

    # Determine base_mode
    base_mode = mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | mavlink.MAV_MODE_FLAG_SAFETY_ARMED if is_armed else mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
    # Add STABILIZE if PID loops are active (which they are when armed here)
    if is_armed:
         base_mode |= mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
    # In a real system, you'd add GUIDED, AUTO etc. based on flight mode

    msg = mav.heartbeat_encode(
        mavlink.MAV_TYPE_QUADROTOR,        # type
        mavlink.MAV_AUTOPILOT_GENERIC,     # autopilot type
        base_mode,                         # base_mode
        0,                                 # custom_mode (not used here)
        status                             # system_status
        # mavlink_version is automatically added
    )
    send_mavlink_message(msg)

def send_attitude(time_boot_ms, roll_rad, pitch_rad, yaw_rad, rollspeed_rads, pitchspeed_rads, yawspeed_rads):
     """
     Sends MAVLink ATTITUDE message.
     Requires angles and rates in RADIANS.
     """
     msg = mav.attitude_encode(
         time_boot_ms,
         roll_rad,
         pitch_rad,
         yaw_rad,
         rollspeed_rads,
         pitchspeed_rads,
         yawspeed_rads
     )
     send_mavlink_message(msg)

# Optional: Implement sending SCALED_IMU if needed
# def send_scaled_imu(time_boot_ms, gx_dps, gy_dps, gz_dps, ax_g=0.0, ay_g=0.0, az_g=0.0):
#     """ Sends MAVLink SCALED_IMU. Requires gyro in deg/s, accel in G's """
#     # Convert to required units: mG for accel, mrad/s for gyro
#     xacc_mg = int(ax_g * 1000)
#     yacc_mg = int(ay_g * 1000)
#     zacc_mg = int(az_g * 1000)
#     xgyro_mrads = int(math.radians(gx_dps) * 1000)
#     ygyro_mrads = int(math.radians(gy_dps) * 1000)
#     zgyro_mrads = int(math.radians(gz_dps) * 1000)
#     # Magnetometer and temperature are not read in this code yet
#     xmag_mgauss = 0
#     ymag_mgauss = 0
#     zmag_mgauss = 0
#     temp_cdeg = 0
#
#     msg = mav.scaled_imu_encode(
#         time_boot_ms,
#         xacc_mg, yacc_mg, zacc_mg,
#         xgyro_mrads, ygyro_mrads, zgyro_mrads,
#         xmag_mgauss, ymag_mgauss, zmag_mgauss #, temp_cdeg # Add temp if your mavlink version supports it
#     )
#     send_mavlink_message(msg)


def send_command_ack(command_id, result):
    """Sends a MAVLink COMMAND_ACK message."""
    msg = mav.command_ack_encode(
        command_id, # Command ID that is being acknowledged
        result      # MAV_RESULT enum value
    )
    send_mavlink_message(msg)
    print(f"Sent ACK for Cmd {command_id} with Result {result}")

def send_statustext(text, severity=mavlink.MAV_SEVERITY_INFO):
    """Sends a MAVLink STATUSTEXT message."""
    # STATUSTEXT message length is limited. Truncate if necessary.
    if len(text) > 50:
        text = text[:50]
    # Ensure text is bytes
    text_bytes = text.encode('utf-8')

    msg = mavlink.MAVLink_statustext_message(severity, text_bytes) # Use constructor directly
    send_mavlink_message(msg)


# --- UTILITY FUNCTIONS (Mostly from original code) ---

def calculate_duty_cycle(throttle:float, dead_zone:float = 0.03) -> int:
    """Determines the appropriate PWM duty cycle, in nanoseconds, for ESCs."""
    # This function seems specific to a 1000us-2000us signal range common for ESCs.
    duty_ceiling:int = 2000000 # Max duty cycle (2ms) = 100% throttle
    duty_floor:int = 1000000   # Min duty cycle (1ms) = 0% throttle

    # Apply dead zone if needed (often not necessary if controller handles it)
    # range_adj:float = 1.0 - dead_zone - dead_zone
    # percentage:float = min(max((throttle - dead_zone) / range_adj, 0.0), 1.0)
    percentage = max(0.0, min(throttle, 1.0)) # Assume input throttle is already 0-1

    dutyns:int = duty_floor + int((duty_ceiling - duty_floor) * percentage)

    # Clamp within absolute min/max pulse width
    dutyns = max(duty_floor, min(dutyns, duty_ceiling))

    return dutyns

def normalize(value:float, original_min:float, original_max:float, new_min:float, new_max:float) -> float:
    """Normalizes (scales) a value to within a specific range, clamping the output."""
    # Prevent division by zero
    if (original_max - original_min) == 0:
        return new_min

    # Normalize
    normalized = new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

    # Clamp the output to the new range
    if new_min < new_max:
        normalized = max(new_min, min(normalized, new_max))
    else: # Handle inverted range
        normalized = max(new_max, min(normalized, new_min))

    return normalized


def translate_pair(high:int, low:int) -> int:
    """Converts a byte pair (high, low) to a signed 16-bit integer."""
    value = (high << 8) + low
    # Convert to signed int if highest bit is set
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def FATAL_ERROR(msg:str) -> None:
    """Logs error, stops motors, and enters an infinite blink loop."""
    global M1, M2, M3, M4 # Access global motor objects

    em:str = "FATAL ERROR @ " + str(time.ticks_ms()) + " ms: " + msg
    print(em)
    try:
        toolkit.log(em) # Log to file if possible
    except Exception as log_e:
        print(f"Logging failed: {log_e}")

    # Attempt to send STATUSTEXT message before dying (if network is up)
    try:
       if mav and udp_socket and qgc_address:
            send_statustext("FATAL: " + msg[:40], mavlink.MAV_SEVERITY_CRITICAL)
            time.sleep_ms(10) # Allow time for UDP packet to send
    except Exception as mav_e:
        print(f"Failed to send MAVLink fatal error: {mav_e}")


    # --- CRITICAL: STOP MOTORS ---
    print("Attempting to stop motors...")
    try:
        # Use calculate_duty_cycle to get the correct 0% duty cycle value
        duty_0_percent = calculate_duty_cycle(0.0)
        if M1: M1.duty_ns(duty_0_percent)
        if M2: M2.duty_ns(duty_0_percent)
        if M3: M3.duty_ns(duty_0_percent)
        if M4: M4.duty_ns(duty_0_percent)
        # Deinit might be risky if called multiple times, just set duty to 0
        # if M1: M1.deinit()
        # if M2: M2.deinit()
        # if M3: M3.deinit()
        # if M4: M4.deinit()
        print("Motors commanded to stop.")
    except Exception as motor_e:
        print(f"Failed to stop motors cleanly: {motor_e}")


    # --- Infinite Error Blink ---
    led = machine.Pin("LED", machine.Pin.OUT)
    while True:
        led.on()
        time.sleep(0.2) # Fast blink for error
        led.off()
        time.sleep(0.2)

# --- Main Execution ---
if __name__ == "__main__":
    # Small delay to allow network potentially more time after boot.py
    time.sleep(2)
    run()
