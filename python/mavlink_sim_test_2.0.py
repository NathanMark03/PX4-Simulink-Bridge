from pymavlink import mavutil
import time
import numpy as np
from simulink_communicator import plant_data
from simulink_communicator import connect_socket

def int16_t(x):
    return np.uint16(x)

def int8_t(x):
    return np.uint8(x)

def update_hil_gps(
        time_since_boot, #uint64_t
        fix_type, #uint8_t
        lat, #int32_t
        lon, #int32_t
        alt, #int32_t
        eph, #uint16_t
        epv, #uint16_t
        vel, #uint16_t
        nv, #int16_t
        ve, #int16_t
        vd,#int16_t
        cog, #uint16_t
        sat_vis, #uint8_t
        id_ = 0, #uint8_t
        yaw = 0 #uint16_t
):
    vehicle.mav.hil_gps_send(
        time_since_boot,
        fix_type,
        lat,
        lon,
        alt,
        eph,
        epv,
        vel,
        nv,
        ve,
        vd,
        cog,
        sat_vis,
        id_,
        yaw
    )
    return

def update_hil(
        time_since_boot,   #uint64_t
        x_accel,           #float
        y_accel,           #float
        z_accel,           #float
        ang_speed_x,       #float
        ang_speed_y,       #float
        ang_speed_z,       #float
        mag_field_x,       #float
        mag_field_y,       #float
        mag_field_z,       #float
        abs_pressure,      #float
        airspeed_,         #float
        altitude_pressure, #float
        temp,              #float
        bitmap = 0      #uint32_t
):
    vehicle.mav.hil_sensor_send(
        time_since_boot,      #Timestamp (UNIX Epoch time or time since system boot), us
        x_accel,      #X acceleration, m/s/s
        y_accel,      #Y acceleration, m/s/s
        z_accel,      #Z acceleration, m/s/s
        ang_speed_x,      #Angular speed around X axis in body frame, rad/s
        ang_speed_y,      #Angular speed around Y axis in body frame, rad/s
        ang_speed_z,      #Angular speed around Z axis in body frame, rad/s
        mag_field_x,      #X Magnetic field, gauss
        mag_field_y,      #Y Magnetic field, gauss
        mag_field_z,      #Z Magnetic field, gauss
        abs_pressure,      #Absolute pressure, hPa
        airspeed_,      #Differential pressure (airspeed), hPa
        altitude_pressure,      #Altitude calculated from pressure
        temp,      #Temperature
        bitmap       #Bitmap for fields that have updated since last message
        )

def update_hil_state_quart(
        time_since_boot,    #uint64_t
        attitude_quat,  #float[4]
        rollspeed,  #float
        pitchspeed, #float
        yawspeed,   #float
        lat,    #int32_t
        lon,    #int32_t
        alt,    #int32_t
        vx, #int16_t
        vy, #int16_t
        vz, #int16_t
        ind_airspeed,   #uint16_t
        true_airspeed,  #uint16_t
        xacc,   #int16_t
        yacc,   #int16_t
        zacc    #int16_t
):
    
    vehicle.mav.hil_state_quaternion_send(
        time_since_boot,      #Timestamp (UNIX Epoch time or time since system boot), us
        attitude_quat,      #Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation), float[4]
        rollspeed,      #Body frame roll / phi angular speed, rad/s
        pitchspeed,      #Body frame pitch / theta angular speed, rad/s
        yawspeed,      #Body frame yaw / psi angular speed, rad/s
        lat,      #Latitude, degE7
        lon,      #Longitude, degE7
        alt,      #Altitude, degE7
        vx,      #Ground X Speed (Latitude), cm/s
        vy,      #Ground Y Speed (Longitude), cm/s
        vz,      #Ground Z Speed (Altitude), cm/s
        ind_airspeed,      #Indicated airspeed, cm/s
        true_airspeed,      #True airspeed, cm/s
        xacc,      #X acceleration, mG
        yacc,      #Y acceleration, mG
        zacc      #Z acceleration, mG
        )

if __name__ == "__main__":
    #set up model ip and port
    MODEL_IP = "127.0.0.1"
    MODEL_PORT = 5005
    MODEL_RECIEVE_PORT = 5006

    #connect to socket
    MODEL_SOCKET = connect_socket(MODEL_IP, MODEL_PORT)

    #time at boot
    time_boot_us = round(time.time() * 1e6) #in microseconds

    # ---------- Begin Connection Sequence ----------

    # ---------- Create vehicle instance ----------
    vehicle = mavutil.mavlink_connection('tcpin:localhost:4560')


    # ---------- Wait for heartbeat ----------
    vehicle.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

    # ---------- End Connection Sequence ----------


    # ---------- Begin main connection loop ----------
    while True:
        # Update time
        time_unix_us = round(time.time() * 1e6)
        time_since_boot_us = time_unix_us - time_boot_us
        time_since_boot_ms = round(time_since_boot_us / 1000)

        # ---------- Send time stamp ----------
        vehicle.mav.system_time_send(
            time_unix_us,   #Timestamp (UNIX epoch time), us
            time_since_boot_ms  #Timestamp (time since system boot), ms
        )

        # ---------- Send heartbeat ----------
        vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,       #Vehicle or component type.
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,  #Autopilot type / class.
            0,                                      #System mode bitmap.
            0,                                      #A bitfield for use for autopilot-specific flags
            0                                       #System status flag.
            )

        # ---------- Send hil hil_quat and gps data ----------
        #get data
        data = plant_data(MODEL_SOCKET)

        #define variables

        #hil
        x_accel = data[0]   #float
        y_accel = data[1]   #float
        z_accel = data[2]   #float
        ang_speed_x = data[3]   #float
        ang_speed_y = data[4]   #float
        ang_speed_z = data[5]   #float
        mag_field_x = data[6]   #float
        mag_field_y = data[7]   #float
        mag_field_z = data[8]   #float
        abs_pressure = data[9]  #float
        airspeed_ = data[10] #float
        altitude_pressure = data[11] #float
        temp_ = data[12]  #float

        #hil_quat
        attitude_quat = [data[13], data[14], data[15], data[16]] #float[4]
        rollspeed = data[17]  #float
        pitchspeed = data[18] #float
        yawspeed = data[19]   #float
        lat = data[20]    #int32_t
        lon = data[21]    #int32_t
        alt = data[22]    #int32_t
        vx = data[23] #int16_t
        vy = data[24] #int16_t
        vz = data[25] #int16_t
        ind_airspeed = data[26]   #uint16_t
        true_airspeed = data[27]  #uint16_t
        xacc = data[28]   #int16_t
        yacc = data[29]   #int16_t
        zacc = data[30]    #int16_t

        #gps
        lat = data[31] #int32_t
        lon = data[32] #int32_t
        alt = data[33] #int32_t
        eph = data[34] #uint16_t
        epv = data[35] #uint16_t
        vel = data[36] #uint16_t
        nv = data[37] #int16_t
        ve = data[38] #int16_t
        vd = data[39]   #int16_t
        cog = data[40] #uint16_t
        fix_type = data[41] #uint8_t
        sat_vis = data[42] #uint8_t

        #send data
        #send hil
        update_hil(
            time_since_boot_us,
            x_accel,
            y_accel,
            z_accel,
            ang_speed_x,
            ang_speed_y,
            ang_speed_z,
            mag_field_x,
            mag_field_y,
            mag_field_z,
            abs_pressure,
            airspeed_,
            altitude_pressure,
            temp_
        )
        
        #send quat
        update_hil_state_quart(
            time_since_boot_us,
            attitude_quat,
            rollspeed,
            pitchspeed,
            yawspeed,
            lat,
            lon,
            alt,
            vx,
            vy,
            vz,
            ind_airspeed,
            true_airspeed,
            xacc,
            yacc,
            zacc
        )

        #send gps
        update_hil_gps(
            time_since_boot_us,
            fix_type,
            lat,
            lon,
            alt,
            eph,
            epv,
            vel,
            nv,
            ve,
            vd,
            cog,
            sat_vis
        )
        time.sleep(0.5) #for debug

        #debug
        print(data)