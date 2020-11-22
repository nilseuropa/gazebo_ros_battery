#ifndef _MOTOR_PLUGIN_H_
#define _MOTOR_PLUGIN_H_

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// Services
#include <gazebo_ros_battery/SetTemperature.h>
#include <gazebo_ros_battery/SetCharge.h>
#include <gazebo_ros_battery/Reset.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosBattery : public ModelPlugin {

    public:

      GazeboRosBattery();
      ~GazeboRosBattery();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:

      virtual void UpdateChild();
      virtual void FiniChild();
      ros::ServiceServer set_temperature;
      ros::ServiceServer set_charge_state;
      ros::ServiceServer reset_model;
      boost::mutex service_lock;

    private:

      GazeboRosPtr gazebo_ros_;
      event::ConnectionPtr update_connection_;
      physics::ModelPtr parent;
      ros::Publisher battery_state_publisher_;
      ros::Publisher battery_voltage_publisher_;
      std::vector<ros::Subscriber> current_draw_subscribers_;
      sensor_msgs::BatteryState battery_state_;
      common::Time last_update_time_;
      std::string battery_topic_;
      std::string consumer_topic_;
      std::string battery_voltage_topic_;
      std::string frame_id_;
      std::string plugin_name_;
      std::vector<double> drawn_currents_;
      // common parameters
      bool publish_voltage_;
      int technology_;
      int num_of_consumers_;
      int cell_count_;
      double update_rate_;
      double update_period_;
      double design_capacity_;
      double current_drawn_;
      double nominal_voltage_;
      double constant_voltage_;
      double cut_off_voltage_;
      double internal_resistance_;
      double lpf_tau_;
      // linear model params
      double lin_discharge_coeff_;
      bool use_nonlinear_model_;
      // nonlinear model params
      double polarization_constant_; // polarization constant [V/Ah] or pol. resistance [Ohm]
      double exponential_voltage_;   // exponential voltage [V]
      double exponential_capacity_;  // exponential capacity [1/(Ah)]
      double characteristic_time_;   // characteristic time [s] for charge-dependence
      double design_temperature_;         // Design temperature where pol. const is unchanged and voltages are nominal.
      double arrhenius_rate_polarization_; // Arrhenius rate of polarization constant [K]
      double capacity_temp_coeff_;      // Temperature dependence of capacity [Ah/K]
      double reversible_voltage_temp_;  // Linear temperature dependant voltage shift [V/K]
      // internal variables
      bool model_initialised_;
      bool internal_cutt_off_;
      bool battery_empty_;
      double voltage_;
      double charge_;
      double charge_memory_;
      double current_;
      double discharge_;
      double capacity_;
      double temperature_;
      double temp_set_;
      double temp_lpf_tau_;
      double current_lpf_;

      // model update
      void linear_discharge_voltage_update();
      void nonlinear_discharge_voltage_update();

      // Services
      bool SetCharge(gazebo_ros_battery::SetCharge::Request& req,
                                            gazebo_ros_battery::SetCharge::Response& res);

      bool ResetModel(gazebo_ros_battery::Reset::Request& req,
                                            gazebo_ros_battery::Reset::Response& res);

      bool SetTemperature(gazebo_ros_battery::SetTemperature::Request& req,
                                            gazebo_ros_battery::SetTemperature::Response& res);
      // Callback Queue
      ros::CallbackQueue queue_;
      std::thread callback_queue_thread_;
      boost::mutex lock_;
      void QueueThread();
      void currentCallback(const std_msgs::Float32::ConstPtr& current, int consumer_id);
  };

}

#endif
