/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <iostream>
#include "nao_dcm_driver/nao.h"
#include <alerror/alerror.h>
#include <alcommon/albroker.h>
#include <algorithm>

Nao::Nao(boost::shared_ptr<AL::ALBroker> broker, const string &name)
    : AL::ALModule(broker,name),is_connected_(false)
{
    setModuleDescription("Nao Robot Module");

    functionName("brokerDisconnected", getName(), "Callback when broker disconnects!");
    BIND_METHOD(Nao::brokerDisconnected);
}

Nao::~Nao()
{
    if(is_connected_)
        disconnect();
}

bool Nao::initialize()
{
    // IMU Memory Keys
    const char* imu[] = {"Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
                                    "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value"};
    imu_names_ = vector<string>(imu, end(imu));

    // Sonar Memory Keys
    const char* sonar[] = {"Device/SubDeviceList/US/Left/Sensor/Value",
                                      "Device/SubDeviceList/US/Left/Sensor/Value1",
                                      "Device/SubDeviceList/US/Left/Sensor/Value2",
                                      "Device/SubDeviceList/US/Left/Sensor/Value3",
                                      "Device/SubDeviceList/US/Left/Sensor/Value4",
                                      "Device/SubDeviceList/US/Left/Sensor/Value5",
                                      "Device/SubDeviceList/US/Left/Sensor/Value6",
                                      "Device/SubDeviceList/US/Left/Sensor/Value7",
                                      "Device/SubDeviceList/US/Left/Sensor/Value8",
                                      "Device/SubDeviceList/US/Left/Sensor/Value9",
                                      "Device/SubDeviceList/US/Right/Sensor/Value",
                                      "Device/SubDeviceList/US/Right/Sensor/Value1",
                                      "Device/SubDeviceList/US/Right/Sensor/Value2",
                                      "Device/SubDeviceList/US/Right/Sensor/Value3",
                                      "Device/SubDeviceList/US/Right/Sensor/Value4",
                                      "Device/SubDeviceList/US/Right/Sensor/Value5",
                                      "Device/SubDeviceList/US/Right/Sensor/Value6",
                                      "Device/SubDeviceList/US/Right/Sensor/Value7",
                                      "Device/SubDeviceList/US/Right/Sensor/Value8",
                                      "Device/SubDeviceList/US/Right/Sensor/Value9"};
    sonar_names_ = vector<string>(sonar, end(sonar));

    // Foot Contact Memory Keys
    const char* fsr[] = {"Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value",
                                    "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
                                    "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",
                                    "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
                                    "Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value",
                                    "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value",
                                    "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
                                    "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",
                                    "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value",
                                    "Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value",
                                    "Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value",
                                    "Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value",
                                    "Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value",
                                    "Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value"};
    fsr_names_ = vector<string>(fsr, end(fsr));

    // Tactile Memory Keys
    const char* tactile[] = {"Device/SubDeviceList/Head/Touch/Front/Sensor/Value",
                                        "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value",
                                        "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value",
                                        "Device/SubDeviceList/LHand/Touch/Back/Sensor/Value",
                                        "Device/SubDeviceList/LHand/Touch/Left/Sensor/Value",
                                        "Device/SubDeviceList/LHand/Touch/Right/Sensor/Value",
                                        "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value",
                                        "Device/SubDeviceList/RHand/Touch/Left/Sensor/Value",
                                        "Device/SubDeviceList/RHand/Touch/Right/Sensor/Value"};
    tactile_names_ = vector<string>(tactile, end(tactile));

    // Bumper Memory Keys
    const char* bumper[] = {"Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
                                       "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
                                       "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
                                       "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value"};
    bumper_names_ = vector<string>(bumper, end(bumper));

    // Battery Memory Keys
    const char* battery[] = {"Device/SubDeviceList/Battery/Charge/Sensor/Value",
                                        "Device/SubDeviceList/Battery/Temperature/Sensor/Value"};
    battery_names_ = vector<string>(battery, end(battery));

    // LED Memory Keys
    const char* led[] = {"Device/SubDeviceList/ChestBoard/Led/Blue/Actuator/Value",
                                        "Device/SubDeviceList/ChestBoard/Led/Green/Actuator/Value",
                                        "Device/SubDeviceList/ChestBoard/Led/Red/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/108Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/144Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/216Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/252Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/288Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/324Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/36Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Left/72Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/108Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/144Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/216Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/252Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/288Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/324Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/36Deg/Actuator/Value",
                                        "Device/SubDeviceList/Ears/Led/Right/72Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/135Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/225Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/270Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/315Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/45Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Left/90Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/135Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/225Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/270Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/315Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/45Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Blue/Right/90Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/135Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/225Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/270Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/315Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/45Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Left/90Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/135Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/225Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/270Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/315Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/45Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Green/Right/90Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/135Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/225Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/270Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/315Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/45Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Left/90Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/0Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/135Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/180Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/225Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/270Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/315Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/45Deg/Actuator/Value",
                                        "Device/SubDeviceList/Face/Led/Red/Right/90Deg/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Front/Left/0/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Front/Left/1/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Front/Right/0/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Front/Right/1/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Middle/Left/0/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Middle/Right/0/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Rear/Left/0/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Rear/Left/1/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Rear/Left/2/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Rear/Right/0/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Rear/Right/1/Actuator/Value",
                                        "Device/SubDeviceList/Head/Led/Rear/Right/2/Actuator/Value",
                                        "Device/SubDeviceList/LFoot/Led/Blue/Actuator/Value",
                                        "Device/SubDeviceList/LFoot/Led/Green/Actuator/Value",
                                        "Device/SubDeviceList/LFoot/Led/Red/Actuator/Value",
                                        "Device/SubDeviceList/RFoot/Led/Blue/Actuator/Value",
                                        "Device/SubDeviceList/RFoot/Led/Green/Actuator/Value",
                                        "Device/SubDeviceList/RFoot/Led/Red/Actuator/Value"};
    led_names_ = vector<string>(led, end(led));

    // Joints Initialization
    const char* joint[] = {"HeadYaw",
                            "HeadPitch",
                            "LShoulderPitch",
                            "LShoulderRoll",
                            "LElbowYaw",
                            "LElbowRoll",
                            "LWristYaw",
                            "LHand",
                            "RShoulderPitch",
                            "RShoulderRoll",
                            "RElbowYaw",
                            "RElbowRoll",
                            "RWristYaw",
                            "RHand",
                            "LHipYawPitch",
                            "RHipYawPitch",
                            "LHipRoll",
                            "LHipPitch",
                            "LKneePitch",
                            "LAnklePitch",
                            "LAnkleRoll",
                            "RHipRoll",
                            "RHipPitch",
                            "RKneePitch",
                            "RAnklePitch",
                            "RAnkleRoll"};
    joint_names_ = vector<string>(joint, end(joint));
    
    for(vector<string>::iterator it=joint_names_.begin();it!=joint_names_.end();it++)
    {
        if((*it=="RHand" || *it=="LHand" || *it == "RWristYaw" || *it == "LWristYaw") && (body_type_ == "H21"))
        {
            joint_names_.erase(it);
            it--;
            continue;
        }
        joints_names_.push_back("Device/SubDeviceList/"+(*it)+"/Position/Sensor/Value");
        if(*it!="RHipYawPitch")
        {
            joint_temperature_names_.push_back("Device/SubDeviceList/"+(*it)+"/Temperature/Sensor/Value");
        }
    }
    number_of_joints_ = joint_names_.size();

    // DCM Motion Commands Initialization
    try
    {
        // Create Motion Command
        commands_.arraySetSize(4);
        commands_[0] = string("Joints");
        commands_[1] = string("ClearAll");
        commands_[2] = string("time-mixed");
        commands_[3].arraySetSize(number_of_joints_);

        // Create Joints Actuators Alias
        AL::ALValue commandAlias;
        commandAlias.arraySetSize(2);
        commandAlias[0] = string("Joints");
        commandAlias[1].arraySetSize(number_of_joints_);
        for(int i=0;i<number_of_joints_;i++)
        {
            commandAlias[1][i] = string("Device/SubDeviceList/"+joint_names_[i]+"/Position/Actuator/Value");
            commands_[3][i].arraySetSize(1);
            commands_[3][i][0].arraySetSize(2);
        }
        dcm_proxy_.createAlias(commandAlias);

        // Create Joints Hardness Alias
        commandAlias[0] = string("JointsHardness");
        commandAlias[1].arraySetSize(number_of_joints_-1);
        int k = 0;
        for(int i=0;i<number_of_joints_;i++)
        {
            if(joint_names_[i] == "RHipYawPitch")
            {
                k = -1;
                i++;
            }
            commandAlias[1][i+k] = string("Device/SubDeviceList/"+joint_names_[i]+"/Hardness/Actuator/Value");
        }
        dcm_proxy_.createAlias(commandAlias);

        // Create LEDs Alias
        commandAlias[0] = string("Leds");
        commandAlias[1].arraySetSize(89);
        for(int i=0;i<89;i++)
        {
            commandAlias[1][i] = led_names_[i];
        }
        dcm_proxy_.createAlias(commandAlias);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not initialize dcm aliases!\n\tTrace: %s",e.what());
        return false;
    }

    // Turn Stiffness On
    vector<float> stiff = vector<float>(number_of_joints_-1,1.0f);
    vector<int> times = vector<int>(number_of_joints_-1,0);
    DCMAliasTimedCommand("JointsHardness",stiff, times);
    stiffnesses_enabled_ = true;

    // Add diagnostic functions
    diagnostic_.setHardwareID(string("Nao")+version_+body_type_);
    diagnostic_.add("Joints Temperature", this, &Nao::checkTemperature);
    diagnostic_.add("Battery", this, &Nao::checkBattery);

    return true;
}

bool Nao::initializeControllers(controller_manager::ControllerManager& cm)
{
    if(!initialize())
    {
        ROS_ERROR("Initialization method failed!");
        return false;
    }

    // Initialize Controllers' Interfaces
    joint_angles_.resize(number_of_joints_);
    joint_velocities_.resize(number_of_joints_);
    joint_efforts_.resize(number_of_joints_);
    joint_commands_.resize(number_of_joints_);

    try
    {
        for(int i=0;i<number_of_joints_;i++)
        {
            hardware_interface::JointStateHandle state_handle(joint_names_[i], &joint_angles_[i],
                                                              &joint_velocities_[i], &joint_efforts_[i]);
            jnt_state_interface_.registerHandle(state_handle);

            hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(joint_names_[i]),
                                                       &joint_commands_[i]);
            jnt_pos_interface_.registerHandle(pos_handle);
        }

        registerInterface(&jnt_state_interface_);
        registerInterface(&jnt_pos_interface_);
    }
    catch(const ros::Exception& e)
    {
        ROS_ERROR("Could not initialize hardware interfaces!\n\tTrace: %s",e.what());
        return false;
    }
    ROS_INFO("Nao Module initialized!");
    return true;
}

bool Nao::connect(const ros::NodeHandle nh)
{
    // Initialize ROS nodes
    node_handle_ = nh;

    is_connected_ = false;

    // Load ROS Parameters
    loadParams();

    // Needed for Error Checking
    try
    {
        subscribeToMicroEvent("ClientDisconnected", "Nao", "brokerDisconnected");
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Could not subscribe to brokerDisconnected!\n\tTrace: %s",e.what());
    }

    // Initialize DCM Proxy
    try
    {
        dcm_proxy_ = AL::DCMProxy(getParentBroker());
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to connect to DCM Proxy!\n\tTrace: %s",e.what());
        return false;
    }

    // Initialize Memory Proxy
    try
    {
        memory_proxy_ = AL::ALMemoryProxy(getParentBroker());
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to connect to Memory Proxy!\n\tTrace: %s",e.what());
        return false;
    }

    is_connected_ = true;

    // Subscribe/Publish ROS Topics/Services
    subscribe();

    // Initialize Controller Manager and Controllers
    manager_ = new controller_manager::ControllerManager(this,node_handle_);
    if(!initializeControllers(*manager_))
    {
        ROS_ERROR("Could not load controllers!");
        return false;
    }
    ROS_INFO("Controllers successfully loaded!");
    return true;
}

void Nao::disconnect()
{
    if(!is_connected_)
        return;
    try
    {
        unsubscribeFromMicroEvent("ClientDisconnected", "Nao");
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to unsubscribe from subscribed events!\n\tTrace: %s",e.what());
    }
    is_connected_ = false;
}

void Nao::subscribe()
{
    // Subscribe/Publish ROS Topics/Services
    cmd_vel_sub_ = node_handle_.subscribe(prefix_+"cmd_vel", topic_queue_, &Nao::commandVelocity, this);

    imu_pub_ = node_handle_.advertise<sensor_msgs::Imu>(prefix_+"imu_data", topic_queue_);

    sonar_left_pub_ = node_handle_.advertise<sensor_msgs::Range>(prefix_+"sonar_left", topic_queue_);
    sonar_left_.header.frame_id = "SonarLeft";
    sonar_left_.radiation_type = sensor_msgs::Range::ULTRASOUND;
    sonar_left_.field_of_view = 1.04719755f;
    sonar_left_.min_range = 0.25;
    sonar_left_.max_range = 2.55;

    sonar_right_pub_ = node_handle_.advertise<sensor_msgs::Range>(prefix_+"sonar_right", topic_queue_);
    sonar_right_.header.frame_id = "SonarRight";
    sonar_right_.radiation_type = sensor_msgs::Range::ULTRASOUND;
    sonar_right_.field_of_view = 1.04719755f;
    sonar_right_.min_range = 0.25;
    sonar_right_.max_range = 2.55;

    sonar_switch_ = node_handle_.advertiseService<Nao, nao_dcm_msgs::BoolService::Request,
            nao_dcm_msgs::BoolService::Response>(prefix_+"Sonar/Enable", &Nao::switchSonar, this);

    fsrs_pub_ = node_handle_.advertise<nao_dcm_msgs::FSRs>(prefix_+"fsrs", topic_queue_);

    fsrs_switch_ = node_handle_.advertiseService<Nao, nao_dcm_msgs::BoolService::Request,
            nao_dcm_msgs::BoolService::Response>(prefix_+"FSRs/Enable", &Nao::switchFSR, this);

    bumpers_pub_ = node_handle_.advertise<nao_dcm_msgs::Bumper>(prefix_+"bumpers", topic_queue_);

    bumpers_switch_ = node_handle_.advertiseService<Nao, nao_dcm_msgs::BoolService::Request,
            nao_dcm_msgs::BoolService::Response>(prefix_+"Bumpers/Enable", &Nao::switchBumper, this);

    tactiles_pub_ = node_handle_.advertise<nao_dcm_msgs::Tactile>(prefix_+"tactiles", topic_queue_);

    tactiles_switch_ = node_handle_.advertiseService<Nao, nao_dcm_msgs::BoolService::Request,
            nao_dcm_msgs::BoolService::Response>(prefix_+"Tactiles/Enable", &Nao::switchTactile, this);

    stiffness_pub_ = node_handle_.advertise<std_msgs::Float32>(prefix_+"stiffnesses", topic_queue_);
    stiffness_.data = 1.0f;

    stiffness_switch_ = node_handle_.advertiseService<Nao, nao_dcm_msgs::BoolService::Request,
            nao_dcm_msgs::BoolService::Response>(prefix_+"Stiffnesses/Enable", &Nao::switchStiffnesses, this);
}

void Nao::loadParams()
{
    ros::NodeHandle n_p("~");
    // Load Server Parameters
    n_p.param("Version", version_, string("V4"));
    n_p.param("BodyType", body_type_, string("H21"));

    n_p.param("TactilesEnabled", tactiles_enabled_, true);
    n_p.param("BumpersEnabled", bumpers_enabled_, true);
    n_p.param("SonarEnabled", sonar_enabled_, true);
    n_p.param("FootContactsEnabled", foot_contacts_enabled_, true);

    n_p.param("PublishIMU", imu_published_, true);

    n_p.param("TopicQueue", topic_queue_, 50);

    n_p.param("Prefix", prefix_, string("nao_dcm"));
    prefix_ = prefix_+"/";

    n_p.param("LowCommunicationFrequency", low_freq_, 10.0);
    n_p.param("HighCommunicationFrequency", high_freq_, 100.0);
    n_p.param("ControllerFrequency", controller_freq_, 15.0);
    n_p.param("JointPrecision", joint_precision_, 0.00174532925);
    n_p.param("OdomFrame", odom_frame_, string("odom"));
}

void Nao::brokerDisconnected(const string& event_name, const string &broker_name, const string& subscriber_identifier)
{
    if(broker_name == "Nao Driver Broker")
        is_connected_ = false;
}

void Nao::DCMTimedCommand(const string &key, const AL::ALValue &value, const int &timeOffset, const string &type)
{
    try
    {
        // Create timed-command
        AL::ALValue command;
        command.arraySetSize(3);
        command[0] = key;
        command[1] = type;
        command[2].arraySetSize(1);
        command[2][0].arraySetSize(2);
        command[2][0][0] = value;
        command[2][0][1] = dcm_proxy_.getTime(timeOffset);

        // Execute timed-command
        dcm_proxy_.set(command);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not execute DCM timed-command!\n\t%s\n\n\tTrace: %s", key.c_str(), e.what());
    }
}

void Nao::DCMAliasTimedCommand(const string &alias, const vector<float> &values, const vector<int> &timeOffsets,
                               const string &type, const string &type2)
{
    try
    {
        // Create Alias timed-command
        AL::ALValue command;
        command.arraySetSize(4);
        command[0] = alias;
        command[1] = type;
        command[2] = type2;
        command[3].arraySetSize(values.size());
        int T = dcm_proxy_.getTime(0);
        for(int i=0;i<values.size();i++)
        {
            command[3][i].arraySetSize(1);
            command[3][i][0].arraySetSize(2);
            command[3][i][0][0] = values[i];
            command[3][i][0][1] = T+timeOffsets[i];
        }

        // Execute Alias timed-command
        dcm_proxy_.setAlias(command);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not execute DCM timed-command!\n\t%s\n\n\tTrace: %s", alias.c_str(), e.what());
    }
}

void Nao::insertDataToMemory(const string &key, const AL::ALValue &value)
{
    memory_proxy_.insertData(key,value);
}

AL::ALValue Nao::getDataFromMemory(const string &key)
{
    return memory_proxy_.getData(key);
}

void Nao::subscribeToEvent(const string &name, const string &callback_module, const string &callback_method)
{
    try
    {
        memory_proxy_.subscribeToEvent(name,callback_module,"",callback_method);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not subscribe to event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Nao::subscribeToMicroEvent(const string &name, const string &callback_module,
                                const string &callback_method, const string &callback_message)
{
    try
    {
        memory_proxy_.subscribeToMicroEvent(name,callback_module,callback_message,callback_method);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not subscribe to micro-event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Nao::unsubscribeFromEvent(const string &name, const string &callback_module)
{
    try
    {
        memory_proxy_.unsubscribeToEvent(name,callback_module);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not unsubscribe from event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Nao::unsubscribeFromMicroEvent(const string &name, const string &callback_module)
{
    try
    {
        memory_proxy_.unsubscribeToMicroEvent(name,callback_module);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not unsubscribe from micro-event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Nao::raiseEvent(const string &name, const AL::ALValue &value)
{
    try
    {
        memory_proxy_.raiseEvent(name,value);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not raise event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Nao::raiseMicroEvent(const string &name, const AL::ALValue &value)
{
    try
    {
        memory_proxy_.raiseMicroEvent(name,value);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not raise micro-event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Nao::declareEvent(const string &name)
{
    try
    {
        memory_proxy_.declareEvent(name);
    }
    catch(AL::ALError& e)
    {
        ROS_ERROR("Could not declare event '%s'.\n\tTrace: %s",name.c_str(),e.what());
    }
}

void Nao::run()
{
    boost::thread t1(&Nao::controllerLoop,this);
    boost::thread t2(&Nao::lowCommunicationLoop,this);
    boost::thread t3(&Nao::highCommunicationLoop,this);
    t1.join();
    t2.join();
    t3.join();
}

void Nao::lowCommunicationLoop()
{
    static ros::Rate rate(low_freq_);
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();

        if(!is_connected_)
            break;

        publishBaseFootprint(time);

        if(sonar_enabled_)
            checkSonar();

        if(foot_contacts_enabled_)
            checkFSR();

        if(tactiles_enabled_)
            checkTactile();

        if(bumpers_enabled_)
            checkBumper();

        if(stiffnesses_enabled_)
        {
            stiffness_.data = 1.0f;
        }
        else
        {
            stiffness_.data = 0.0f;
        }
        stiffness_pub_.publish(stiffness_);

        diagnostic_.update();

        rate.sleep();
    }
}

void Nao::highCommunicationLoop()
{
    static ros::Rate rate(high_freq_);
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();

        if(!is_connected_)
            break;

        if(imu_published_)
            publishIMU(time);

        try
        {
            dcm_proxy_.ping();
        }
        catch(const AL::ALError& e)
        {
            ROS_ERROR("Could not ping DCM proxy.\n\tTrace: %s",e.what());
            is_connected_ = false;
        }
        rate.sleep();
    }
}

void Nao::controllerLoop()
{
    static ros::Rate rate(controller_freq_+10.0);
    while(ros::ok())
    {
        ros::Time time = ros::Time::now();

        if(!is_connected_)
            break;

        readJoints();

        manager_->update(time,ros::Duration(1.0f/controller_freq_));

        writeJoints();

        rate.sleep();
    }
}

bool Nao::connected()
{
    return is_connected_;
}

void Nao::commandVelocity(const geometry_msgs::TwistConstPtr &msg)
{
    ROS_WARN("This function does nothing at the moment..");
}

void Nao::publishIMU(const ros::Time &ts)
{
    vector<float> memData;
    try
    {
        memData = memory_proxy_.getListData(imu_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get IMU data from Nao.\n\tTrace: %s",e.what());
        return;
    }

    if (memData.size() != imu_names_.size())
    {
        ROS_ERROR("IMU readings' size is not correct!");
        return;
    }

    imu_.header.stamp = ts;
    imu_.header.frame_id = "torso";

    float angleX = memData[0];
    float angleY = memData[1];
    float angleZ = memData[2];
    float gyroX = memData[3];
    float gyroY = memData[4];
    float gyroZ = memData[5];
    float accX = memData[6];
    float accY = memData[7];
    float accZ = memData[8];

    imu_.orientation = tf::createQuaternionMsgFromRollPitchYaw(angleX,angleY,angleZ);

    imu_.angular_velocity.x = gyroX;
    imu_.angular_velocity.y = gyroY;
    imu_.angular_velocity.z = gyroZ;

    imu_.linear_acceleration.x = accX;
    imu_.linear_acceleration.y = accY;
    imu_.linear_acceleration.z = accZ;

    // covariances unknown
    imu_.orientation_covariance[0] = 0;
    imu_.angular_velocity_covariance[0] = 0;
    imu_.linear_acceleration_covariance[0] = 0;

    imu_pub_.publish(imu_);
}

void Nao::publishBaseFootprint(const ros::Time &ts)
{
    string odom_frame, l_sole_frame, r_sole_frame, base_link_frame;
    try {
        odom_frame = base_footprint_listener_.resolve(odom_frame_);
        l_sole_frame = base_footprint_listener_.resolve("l_sole");
        r_sole_frame = base_footprint_listener_.resolve("r_sole");
        base_link_frame = base_footprint_listener_.resolve("base_link");
    }
    catch(ros::Exception& e)
    {
        ROS_ERROR("%s",e.what());
        return;
    }

    tf::StampedTransform tf_odom_to_base, tf_odom_to_left_foot, tf_odom_to_right_foot;
    double temp_freq = 1.0f/(10.0*high_freq_);
    if(!base_footprint_listener_.waitForTransform(odom_frame, l_sole_frame, ros::Time(0), ros::Duration(temp_freq)))
        return;
    try {
        base_footprint_listener_.lookupTransform(odom_frame, l_sole_frame, ros::Time(0), tf_odom_to_left_foot);
        base_footprint_listener_.lookupTransform(odom_frame, r_sole_frame, ros::Time(0), tf_odom_to_right_foot);
        base_footprint_listener_.lookupTransform(odom_frame, base_link_frame,  ros::Time(0), tf_odom_to_base);
    }
    catch (const tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        return ;
    }

    tf::Vector3 new_origin = (tf_odom_to_right_foot.getOrigin() + tf_odom_to_left_foot.getOrigin())/2.0;
    double height = std::min(tf_odom_to_left_foot.getOrigin().getZ(), tf_odom_to_right_foot.getOrigin().getZ());
    new_origin.setZ(height);

    double roll, pitch, yaw;
    tf_odom_to_base.getBasis().getRPY(roll, pitch, yaw);

    tf::Transform tf_odom_to_footprint(tf::createQuaternionFromYaw(yaw), new_origin);
    tf::Transform tf_base_to_footprint = tf_odom_to_base.inverse() * tf_odom_to_footprint;

    base_footprint_broadcaster_.sendTransform(tf::StampedTransform(tf_base_to_footprint, ts,
                                                                   base_link_frame, "base_footprint"));
}

void Nao::readJoints()
{
    vector<float> jointData;
    try
    {
        jointData = memory_proxy_.getListData(joints_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get joint data from Nao.\n\tTrace: %s",e.what());
        return;
    }

    for(short i = 0; i<jointData.size(); i++)
    {
        joint_angles_[i] = jointData[i];
        // Set commands to the read angles for when no command specified
        joint_commands_[i] = jointData[i];
    }
}

void Nao::writeJoints()
{
    // Update joints only when actual command is issued
    bool changed = false;
    for(int i=0;i<number_of_joints_;i++)
    {
        if(fabs(joint_commands_[i]-joint_angles_[i])>joint_precision_)
        {
            changed = true;
            break;
        }
    }
    // Do not write joints if no change in joint values
    if(!changed)
    {
        return;
    }

    try
    {
        int T = dcm_proxy_.getTime(0);
        for(int i=0;i<number_of_joints_;i++)
        {
            commands_[3][i][0][0] = float(joint_commands_[i]);
            commands_[3][i][0][1] = T+(int)(800.0f/controller_freq_);
        }
        
        dcm_proxy_.setAlias(commands_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not send joint commands to Nao.\n\tTrace: %s",e.what());
        return;
    }
}

bool Nao::switchSonar(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res)
{
    sonar_enabled_ = req.enable;
    return true;
}

bool Nao::switchFSR(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res)
{
    foot_contacts_enabled_ = req.enable;
    return true;
}

bool Nao::switchBumper(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res)
{
    bumpers_enabled_ = req.enable;
    return true;
}

bool Nao::switchTactile(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res)
{
    tactiles_enabled_ = req.enable;
    return true;
}

bool Nao::switchStiffnesses(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res)
{
    if(stiffnesses_enabled_!=req.enable && req.enable)
    {
        DCMAliasTimedCommand("JointsHardness",vector<float>(number_of_joints_,1.0f), vector<int>(number_of_joints_,0));
    }
    else if(stiffnesses_enabled_!=req.enable)
    {
        DCMAliasTimedCommand("JointsHardness",vector<float>(number_of_joints_,0.0f), vector<int>(number_of_joints_,0));
    }
    stiffnesses_enabled_ = req.enable;
}

void Nao::checkSonar()
{
    // Send Sonar Wave
    DCMTimedCommand("Device/SubDeviceList/US/Actuator/Value",4.0f,0);

    // Read Sonar Values
    AL::ALValue sonars;
    try
    {
        sonars = memory_proxy_.getListData(sonar_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get sonar values.\n\tTrace: %s",e.what());
        return;
    }

    // Select closer object detected
    float sonar_left = float(sonars[0]), sonar_right = float(sonars[10]);
    for(short i=1;i<10;i++)
    {
        if(float(sonars[i])>=0.0f && float(sonars[i])<=2.55f && float(sonars[i])<sonar_left)
        {
            sonar_left = float(sonars[i]);
        }
        if(float(sonars[10+i])>=0.0f && float(sonars[10+i])<=2.55f && float(sonars[10+i])<sonar_right)
        {
            sonar_right = float(sonars[10+i]);
        }
    }

    sonar_left_.range = sonar_left;
    sonar_left_pub_.publish(sonar_left_);

    sonar_right_.range = sonar_right;
    sonar_right_pub_.publish(sonar_right_);
}

void Nao::checkFSR()
{
    AL::ALValue fsrs;
    try
    {
        fsrs = memory_proxy_.getListData(fsr_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get FSR values.\n\tTrace: %s",e.what());
        return;
    }

    fsrs_.LeftFrontLeft = float(fsrs[0]);
    fsrs_.LeftFrontRight = float(fsrs[1]);
    fsrs_.LeftRearLeft = float(fsrs[2]);
    fsrs_.LeftRearRight = float(fsrs[3]);
    fsrs_.LeftTotalWeight = float(fsrs[4]);

    fsrs_.LeftCOPx = float(fsrs[10]);
    fsrs_.LeftCOPy = float(fsrs[11]);

    fsrs_.RightFrontLeft = float(fsrs[5]);
    fsrs_.RightFrontRight = float(fsrs[6]);
    fsrs_.RightRearLeft = float(fsrs[7]);
    fsrs_.RightRearRight = float(fsrs[8]);
    fsrs_.RightTotalWeight = float(fsrs[9]);

    fsrs_.RightCOPx = float(fsrs[12]);
    fsrs_.RightCOPy = float(fsrs[13]);

    fsrs_pub_.publish(fsrs_);
}

void Nao::checkTactile()
{
    AL::ALValue tactiles;
    try
    {
        tactiles = memory_proxy_.getListData(tactile_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get Tactile values.\n\tTrace: %s",e.what());
        return;
    }

    tactiles_.HeadTouchFront = int(float(tactiles[0]));
    tactiles_.HeadTouchMiddle = int(float(tactiles[1]));
    tactiles_.HeadTouchRear = int(float(tactiles[2]));

    tactiles_.LeftTouchBack = int(float(tactiles[3]));
    tactiles_.LeftTouchLeft = int(float(tactiles[4]));
    tactiles_.LeftTouchRight = int(float(tactiles[5]));

    tactiles_.RightTouchBack = int(float(tactiles[6]));
    tactiles_.RightTouchLeft = int(float(tactiles[7]));
    tactiles_.RightTouchRight = int(float(tactiles[8]));

    tactiles_pub_.publish(tactiles_);
}

void Nao::checkBumper()
{
    AL::ALValue bumpers;
    try
    {
        bumpers = memory_proxy_.getListData(bumper_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get Tactile values.\n\tTrace: %s",e.what());
        return;
    }

    bumpers_.LeftFootLeft = int(float(bumpers[0]));
    bumpers_.LeftFootRight = int(float(bumpers[1]));

    bumpers_.RightFootLeft = int(float(bumpers[2]));
    bumpers_.RightFootRight = int(float(bumpers[3]));

    bumpers_pub_.publish(bumpers_);
}

void Nao::checkTemperature(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    AL::ALValue temps;
    try
    {
        temps = memory_proxy_.getListData(joint_temperature_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get Temperature values.\n\tTrace: %s",e.what());
        return;
    }
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Joints Temperature: OK!");
    for(short i=0;i<temps.getSize();i++)
    {
        if(float(temps[i])>=70.0f)
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Joints Temperature: WARNING!");
        if(float(temps[i])>=85.0f)
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Joints Temperature: CRITICAL!");
        string joint_name = string(joint_temperature_names_[i]).erase(0,21);
        short l = joint_name.find('/',joint_name.find('/'));
        joint_name.erase(l,25);
        stat.add(joint_name,temps[i]);
    }
}

void Nao::checkBattery(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    AL::ALValue batt;
    try
    {
        batt = memory_proxy_.getListData(battery_names_);
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not get Battery values.\n\tTrace: %s",e.what());
        return;
    }
    int status = 0;
    string message = "Battery: "+boost::lexical_cast<string>(float(batt[0])*100.0f)+"\% charged!  ";
    if(float(batt[0])*100.0f<50.0f)
        status = 1;
    else if(float(batt[0])*100.0f<20.0f)
        status = 2;

    if(float(batt[1])>=60.0f)
    {
        status = 1;
        message += "Temperature: WARNING!";
    }
    else if(float(batt[1])>=70.0f)
    {
        status = 2;
        message += "Temperature: CRITICAL!";
    }
    else
    {
        message += "Temperature: OK!";
    }
    stat.summary(status,message);

    stat.add("Battery Charge",float(batt[0])*100.0f);
    stat.add("Battery Temperature", float(batt[1]));
}
