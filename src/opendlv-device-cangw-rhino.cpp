/*
 * Copyright (C) 2019  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "fh16gw.hpp"

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>

#ifdef __linux__
    #include <linux/if.h>
    #include <linux/can.h>
#endif

#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

inline int encode(uint8_t *dst, uint8_t len);
int encode(uint8_t *dst, uint8_t len) {
    if ( (nullptr == dst) || (0 == len) ) return 0;
    // TODO: Provide logic to check what messages to actually encode; the code in the
    //       following is mainly template code for illustration.
 
    // Message to encode: FH16GW_WHEEL_SPEEDS1_FRAME_ID
    {
        fh16gw_wheel_speeds1_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::Wheels msg;
        tmp.front_axle1_wheel_speed_left = fh16gw_wheel_speeds1_front_axle1_wheel_speed_left_encode(msg.speedWheel111());
        tmp.front_axle1_wheel_speed_right = fh16gw_wheel_speeds1_front_axle1_wheel_speed_right_encode(msg.speedWheel112());
        tmp.drive_axle1_wheel_speed_left = fh16gw_wheel_speeds1_drive_axle1_wheel_speed_left_encode(msg.speedWheel121());
        tmp.drive_axle1_wheel_speed_right = fh16gw_wheel_speeds1_drive_axle1_wheel_speed_right_encode(msg.speedWheel122());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_wheel_speeds1_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_STEER_REQUEST_FRAME_ID
    {
        fh16gw_steer_request_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::SteeringRequest msg;
        tmp.steer_req_delta_trq = fh16gw_steer_request_steer_req_delta_trq_encode(msg.steeringDeltaTorque());
        tmp.steer_req_rwa = fh16gw_steer_request_steer_req_rwa_encode(msg.steeringRoadWheelAngle());
        tmp.enable_steer_req = fh16gw_steer_request_enable_steer_req_encode(msg.enableRequest());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_steer_request_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_ACCELERATION_REQUEST_FRAME_ID
    {
        fh16gw_acceleration_request_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::AccelerationRequest msg;
        tmp.acceleration_request_pedal = fh16gw_acceleration_request_acceleration_request_pedal_encode(msg.accelerationPedalPosition());
        tmp.enable_acc_request = fh16gw_acceleration_request_enable_acc_request_encode(msg.enableRequest());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_acceleration_request_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_WHEEL_SPEEDS2_FRAME_ID
    {
        fh16gw_wheel_speeds2_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::Wheels msg;
        tmp.drive_axle2_wheel_speed_left = fh16gw_wheel_speeds2_drive_axle2_wheel_speed_left_encode(msg.speedWheel131());
        tmp.drive_axle2_wheel_speed_right = fh16gw_wheel_speeds2_drive_axle2_wheel_speed_right_encode(msg.speedWheel132());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_wheel_speeds2_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_AXLE_LOADS_FRAME_ID
    {
        fh16gw_axle_loads_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::Axles msg;
        tmp.front_axle1_load = fh16gw_axle_loads_front_axle1_load_encode(msg.loadAxle11());
        tmp.drive_axle1_load = fh16gw_axle_loads_drive_axle1_load_encode(msg.loadAxle12());
        tmp.drive_axle2_load = fh16gw_axle_loads_drive_axle2_load_encode(msg.loadAxle13());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_axle_loads_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_STEERING_FRAME_ID
    {
        fh16gw_steering_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::Steering msg;
        tmp.road_wheel_angle = fh16gw_steering_road_wheel_angle_encode(msg.roadWheelAngle());
        tmp.steering_wheel_angle = fh16gw_steering_steering_wheel_angle_encode(msg.steeringWheelAngle());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_steering_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_DRIVELINE_FRAME_ID
    {
        fh16gw_driveline_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::Driveline msg;
        tmp.engine_speed = fh16gw_driveline_engine_speed_encode(msg.engineSpeed());
        tmp.engine_torque = fh16gw_driveline_engine_torque_encode(msg.engineTorque());
        tmp.current_gear = fh16gw_driveline_current_gear_encode(msg.currentGear());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_driveline_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_MANUAL_DRIVER_FRAME_ID
    {
        fh16gw_manual_driver_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::ManualControl msg;
        tmp.acceleration_pedal_position = fh16gw_manual_driver_acceleration_pedal_position_encode(msg.accelerationPedalPosition());
        tmp.brake_pedal_position = fh16gw_manual_driver_brake_pedal_position_encode(msg.brakePedalPosition());
        tmp.torsion_bar_torque = fh16gw_manual_driver_torsion_bar_torque_encode(msg.torsionBarTorque());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_manual_driver_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_VEHICLE_DYNAMICS_FRAME_ID
    {
        fh16gw_vehicle_dynamics_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::VehicleState msg;
        tmp.acceleration_x = fh16gw_vehicle_dynamics_acceleration_x_encode(msg.accelerationX());
        tmp.acceleration_y = fh16gw_vehicle_dynamics_acceleration_y_encode(msg.accelerationY());
        tmp.yaw_rate = fh16gw_vehicle_dynamics_yaw_rate_encode(msg.yawRate());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_vehicle_dynamics_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_VEHICLE_SPEED_FRAME_ID
    {
        fh16gw_vehicle_speed_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::Propulsion msg;
        tmp.vehicle_speed_prop_shaft = fh16gw_vehicle_speed_vehicle_speed_prop_shaft_encode(msg.propulsionShaftVehicleSpeed());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_vehicle_speed_pack(dst, &tmp, len);
        return size;
    }
    // Message to encode: FH16GW_BRAKE_REQUEST_FRAME_ID
    {
        fh16gw_brake_request_t tmp;
        // The following msg would have to be passed to this encoder externally.
        opendlv::proxy::rhino::BrakeRequest msg;
        tmp.brake_request = fh16gw_brake_request_brake_request_encode(msg.brake());
        tmp.enable_brake_request = fh16gw_brake_request_enable_brake_request_encode(msg.enableRequest());
        // The following statement packs the encoded values into a CAN frame.
        int size = fh16gw_brake_request_pack(dst, &tmp, len);
        return size;
    }
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ) {
        std::cerr << argv[0] << " translates messages from CAN to ODVD messages and vice versa." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --can=<name of the CAN device> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --can=can0 --verbose" << std::endl;
    }
    else {
        const std::string CANDEVICE{commandlineArguments["can"]};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        // Delegate to convert incoming CAN frames into ODVD messages that are broadcast into the OD4Session.
        opendlv::proxy::rhino::Wheels msgWheels;
        auto decode = [&od4, VERBOSE, &msgWheels](cluon::data::TimeStamp ts, uint16_t canFrameID, uint8_t *src, uint8_t len) {
            if ( (nullptr == src) || (0 == len) ) return;
            if (FH16GW_WHEEL_SPEEDS1_FRAME_ID == canFrameID) {
                fh16gw_wheel_speeds1_t tmp;
                if (0 == fh16gw_wheel_speeds1_unpack(&tmp, src, len)) {
                    msgWheels.speedWheel111(fh16gw_wheel_speeds1_front_axle1_wheel_speed_left_decode(tmp.front_axle1_wheel_speed_left));
                    msgWheels.speedWheel112(fh16gw_wheel_speeds1_front_axle1_wheel_speed_right_decode(tmp.front_axle1_wheel_speed_right));
                    msgWheels.speedWheel121(fh16gw_wheel_speeds1_drive_axle1_wheel_speed_left_decode(tmp.drive_axle1_wheel_speed_left));
                    msgWheels.speedWheel122(fh16gw_wheel_speeds1_drive_axle1_wheel_speed_right_decode(tmp.drive_axle1_wheel_speed_right));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msgWheels.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msgWheels, ts, 0);
                }
            }
            else if (FH16GW_WHEEL_SPEEDS2_FRAME_ID == canFrameID) {
                fh16gw_wheel_speeds2_t tmp;
                if (0 == fh16gw_wheel_speeds2_unpack(&tmp, src, len)) {
                    msgWheels.speedWheel131(fh16gw_wheel_speeds2_drive_axle2_wheel_speed_left_decode(tmp.drive_axle2_wheel_speed_left));
                    msgWheels.speedWheel132(fh16gw_wheel_speeds2_drive_axle2_wheel_speed_right_decode(tmp.drive_axle2_wheel_speed_right));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msgWheels.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msgWheels, ts, 0);
                }
            }
            else if (FH16GW_AXLE_LOADS_FRAME_ID == canFrameID) {
                fh16gw_axle_loads_t tmp;
                if (0 == fh16gw_axle_loads_unpack(&tmp, src, len)) {
                    opendlv::proxy::rhino::Axles msg;
                    msg.loadAxle11(fh16gw_axle_loads_front_axle1_load_decode(tmp.front_axle1_load));
                    msg.loadAxle12(fh16gw_axle_loads_drive_axle1_load_decode(tmp.drive_axle1_load));
                    msg.loadAxle13(fh16gw_axle_loads_drive_axle2_load_decode(tmp.drive_axle2_load));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msg, ts, 0);
                }
            }
            else if (FH16GW_STEERING_FRAME_ID == canFrameID) {
                fh16gw_steering_t tmp;
                if (0 == fh16gw_steering_unpack(&tmp, src, len)) {
                    opendlv::proxy::rhino::Steering msg;
                    msg.roadWheelAngle(fh16gw_steering_road_wheel_angle_decode(tmp.road_wheel_angle));
                    msg.steeringWheelAngle(fh16gw_steering_steering_wheel_angle_decode(tmp.steering_wheel_angle));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msg, ts, 0);
                }
            }
            else if (FH16GW_DRIVELINE_FRAME_ID == canFrameID) {
                fh16gw_driveline_t tmp;
                if (0 == fh16gw_driveline_unpack(&tmp, src, len)) {
                    opendlv::proxy::rhino::Driveline msg;
                    msg.engineSpeed(static_cast<float>(fh16gw_driveline_engine_speed_decode(tmp.engine_speed)));
                    msg.engineTorque(static_cast<float>(fh16gw_driveline_engine_torque_decode(tmp.engine_torque)));
                    msg.currentGear(static_cast<int8_t>(fh16gw_driveline_current_gear_decode(tmp.current_gear)));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msg, ts, 0);
                }
            }
            else if (FH16GW_MANUAL_DRIVER_FRAME_ID == canFrameID) {
                fh16gw_manual_driver_t tmp;
                if (0 == fh16gw_manual_driver_unpack(&tmp, src, len)) {
                    opendlv::proxy::rhino::ManualControl msg;
                    msg.accelerationPedalPosition(fh16gw_manual_driver_acceleration_pedal_position_decode(tmp.acceleration_pedal_position));
                    msg.brakePedalPosition(fh16gw_manual_driver_brake_pedal_position_decode(tmp.brake_pedal_position));
                    msg.torsionBarTorque(fh16gw_manual_driver_torsion_bar_torque_decode(tmp.torsion_bar_torque));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msg, ts, 0);
                }
            }
            else if (FH16GW_VEHICLE_DYNAMICS_FRAME_ID == canFrameID) {
                fh16gw_vehicle_dynamics_t tmp;
                if (0 == fh16gw_vehicle_dynamics_unpack(&tmp, src, len)) {
                    opendlv::proxy::rhino::VehicleState msg;
                    msg.accelerationX(fh16gw_vehicle_dynamics_acceleration_x_decode(tmp.acceleration_x));
                    msg.accelerationY(fh16gw_vehicle_dynamics_acceleration_y_decode(tmp.acceleration_y));
                    msg.yawRate(fh16gw_vehicle_dynamics_yaw_rate_decode(tmp.yaw_rate));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msg, ts, 0);
                }
            }
            else if (FH16GW_VEHICLE_SPEED_FRAME_ID == canFrameID) {
                fh16gw_vehicle_speed_t tmp;
                if (0 == fh16gw_vehicle_speed_unpack(&tmp, src, len)) {
                    opendlv::proxy::rhino::Propulsion msg;
                    msg.propulsionShaftVehicleSpeed(fh16gw_vehicle_speed_vehicle_speed_prop_shaft_decode(tmp.vehicle_speed_prop_shaft));
                    if (VERBOSE) {
                        std::stringstream sstr;
                        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                                   [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                   []() {});
                        std::cout << sstr.str() << std::endl;
                    }

                    od4.send(msg, ts, 0);
                }
            }
        };


//        // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
//        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);


#ifdef __linux__
        struct sockaddr_can address;
#endif
        int socketCAN;

        std::cerr << "[opendlv-device-cangw-rhino] Opening " << CANDEVICE << "... ";
#ifdef __linux__
        // Create socket for SocketCAN.
        socketCAN = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socketCAN < 0) {
            std::cerr << "failed." << std::endl;

            std::cerr << "[opendlv-device-cangw-rhino] Error while creating socket: " << strerror(errno) << std::endl;
        }

        // Try opening the given CAN device node.
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strcpy(ifr.ifr_name, CANDEVICE.c_str());
        if (0 != ioctl(socketCAN, SIOCGIFINDEX, &ifr)) {
            std::cerr << "failed." << std::endl;

            std::cerr << "[opendlv-device-cangw-rhino] Error while getting index for " << CANDEVICE << ": " << strerror(errno) << std::endl;
            return retCode;
        }

        // Setup address and port.
        memset(&address, 0, sizeof(address));
        address.can_family = AF_CAN;
        address.can_ifindex = ifr.ifr_ifindex;

        if (bind(socketCAN, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
            std::cerr << "failed." << std::endl;

            std::cerr << "[opendlv-device-cangw-rhino] Error while binding socket: " << strerror(errno) << std::endl;
            return retCode;
        }
        std::cerr << "done." << std::endl;
#else
        std::cerr << "failed (SocketCAN not available on this platform). " << std::endl;
        return retCode;
#endif

        struct can_frame frame;
        fd_set rfds;
        struct timeval timeout;
        struct timeval socketTimeStamp;
        int32_t nbytes = 0;

        while (od4.isRunning() && socketCAN > -1) {
#ifdef __linux__
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            FD_ZERO(&rfds);
            FD_SET(socketCAN, &rfds);

            select(socketCAN + 1, &rfds, NULL, NULL, &timeout);

            if (FD_ISSET(socketCAN, &rfds)) {
                nbytes = read(socketCAN, &frame, sizeof(struct can_frame));
                if ( (nbytes > 0) && (nbytes == sizeof(struct can_frame)) ) {
                    // Get receiving time stamp.
                    if (0 != ioctl(socketCAN, SIOCGSTAMP, &socketTimeStamp)) {
                        // In case the ioctl failed, use traditional vsariant.
                        cluon::data::TimeStamp now{cluon::time::now()};
                        socketTimeStamp.tv_sec = now.seconds();
                        socketTimeStamp.tv_usec = now.microseconds();
                    }
                    cluon::data::TimeStamp sampleTimeStamp;
                    sampleTimeStamp.seconds(socketTimeStamp.tv_sec)
                                   .microseconds(socketTimeStamp.tv_usec);
                    decode(sampleTimeStamp, frame.can_id, frame.data, frame.can_dlc);
                }
            }
#endif
        }

        std::clog << "[opendlv-device-cangw-rhino] Closing " << CANDEVICE << "... ";
        if (socketCAN > -1) {
            close(socketCAN);
        }
        std::clog << "done." << std::endl;

        retCode = 0;
    }
    return retCode;
}

