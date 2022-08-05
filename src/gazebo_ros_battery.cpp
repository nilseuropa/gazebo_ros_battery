#include <algorithm>
#include <assert.h>
#include <sdf/sdf.hh>
#include <ros/ros.h>

#include "gazebo_ros_battery/gazebo_ros_battery.h"

namespace gazebo {

// Constructor
GazeboRosBattery::GazeboRosBattery() {
}

// Destructor
GazeboRosBattery::~GazeboRosBattery() {
	FiniChild();
}

// Load the controller
void GazeboRosBattery::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->parent = _parent;
		this->plugin_name_ = _sdf->GetAttribute("name")->GetAsString();

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, plugin_name_ ) );
    gazebo_ros_->isInitialized();

		// topic params
		gazebo_ros_->getParameter<int> ( num_of_consumers_, "num_of_consumers", 2 );

		gazebo_ros_->getParameter<std::string> ( consumer_topic_, "consumer_topic", "/battery/consumer" );
		gazebo_ros_->getParameter<std::string> ( frame_id_, "frame_id", "battery" );
		gazebo_ros_->getParameter<std::string> ( battery_topic_, "battery_topic", "/battery/status" );

		gazebo_ros_->getParameter<bool> ( publish_voltage_, "publish_voltage", true );
		if (publish_voltage_){
			gazebo_ros_->getParameter<std::string> ( battery_voltage_topic_, "battery_voltage_topic", "/battery/voltage" );
		}

		// common parameters
		gazebo_ros_->getParameter<int> ( technology_, "technology", sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION );
		gazebo_ros_->getParameter<int> ( cell_count_, "number_of_cells", 8 );
		gazebo_ros_->getParameter<double> ( update_rate_, "update_rate", 10.0 );
		gazebo_ros_->getParameter<double> ( design_capacity_, "design_capacity", 4.0 );
		gazebo_ros_->getParameter<double> ( nominal_voltage_, "nominal_voltage", 24.0 );
		gazebo_ros_->getParameter<double> ( cut_off_voltage_, "cut_off_voltage", 18.0 );
		gazebo_ros_->getParameter<double> ( constant_voltage_, "full_charge_voltage", 24.2 );
		gazebo_ros_->getParameter<double> ( lpf_tau_, "current_filter_tau", 1.0 );
		gazebo_ros_->getParameter<double> ( temp_lpf_tau_, "temperature_response_tau", 0.5 );
		gazebo_ros_->getParameter<double> ( internal_resistance_, "internal_resistance", 0.05 );

		// model specific params
		gazebo_ros_->getParameter<bool> ( use_nonlinear_model_, "use_nonlinear_model", true );
		if (!use_nonlinear_model_) {
			gazebo_ros_->getParameter<double> ( lin_discharge_coeff_, "linear_discharge_coeff", -1.0 );
		}
		else {
			gazebo_ros_->getParameter<double> ( polarization_constant_, "polarization_constant", 0.07 );
			gazebo_ros_->getParameter<double> ( exponential_voltage_, "exponential_voltage", 0.7 );
			gazebo_ros_->getParameter<double> ( exponential_capacity_, "exponential_capacity", 3.0 );
			gazebo_ros_->getParameter<double> ( characteristic_time_, "characteristic_time", 200.0 );
			gazebo_ros_->getParameter<double> ( design_temperature_, "design_temperature", 25.0 );
			gazebo_ros_->getParameter<double> ( arrhenius_rate_polarization_, "arrhenius_rate_polarization", 500.0 );
			gazebo_ros_->getParameter<double> ( reversible_voltage_temp_, "reversible_voltage_temp", 0.05 );
			gazebo_ros_->getParameter<double> ( capacity_temp_coeff_, "capacity_temp_coeff", 0.01 );
		}

		battery_state_.header.frame_id = frame_id_;
		battery_state_.power_supply_technology = technology_;
		battery_state_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
		battery_state_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
		battery_state_.design_capacity = design_capacity_;
		battery_state_.capacity = design_capacity_;
		// battery_state_.capacity   = last_full_capacity_;
		battery_state_.present = true;
		//battery_state_.temperature = temperature_ = design_temperature_;

		// start
		if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_; else this->update_period_ = 0.0;
		last_update_time_ = parent->GetWorld()->SimTime();

		// subscribers
		std::vector<ros::SubscribeOptions> subscribe_options;
		ROS_INFO_NAMED(plugin_name_, "%s: Creating %d consumer subscribers:", gazebo_ros_->info(), num_of_consumers_);
		for (int i=0; i<num_of_consumers_; i++)
		{
			subscribe_options.push_back(ros::SubscribeOptions::create<std_msgs::Float32> (
	        consumer_topic_+"/"+std::to_string(i),
					1,
	        boost::bind(&GazeboRosBattery::currentCallback, this, _1, i),
	        ros::VoidPtr(),
	        &queue_
	    ));
			ROS_INFO_NAMED(plugin_name_, "%s: Listening to consumer on: %s", gazebo_ros_->info(), (consumer_topic_+"/"+std::to_string(i)).c_str());
			current_draw_subscribers_.push_back(gazebo_ros_->node()->subscribe(subscribe_options[i]));
			drawn_currents_.push_back(0);
		}

		// publishers
    battery_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::BatteryState>(battery_topic_, 1);
    ROS_INFO_NAMED(plugin_name_, "%s: Advertising battery state on %s ", gazebo_ros_->info(), battery_topic_.c_str());
		if (publish_voltage_){
			battery_voltage_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Float32>(battery_voltage_topic_, 1);
			ROS_INFO_NAMED(plugin_name_, "%s: Advertising battery voltage on %s ", gazebo_ros_->info(), battery_voltage_topic_.c_str());
		}

		// start custom queue
    this->callback_queue_thread_ = std::thread ( std::bind ( &GazeboRosBattery::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosBattery::UpdateChild, this ) );

		// services
		this->set_charge_state = gazebo_ros_->node()->advertiseService(plugin_name_ + "/set_charge", &GazeboRosBattery::SetCharge, this);
		this->set_temperature = gazebo_ros_->node()->advertiseService(plugin_name_ + "/set_temperature", &GazeboRosBattery::SetTemperature, this);
		this->reset_model = gazebo_ros_->node()->advertiseService(plugin_name_ + "/reset", &GazeboRosBattery::ResetModel, this);

		this->model_initialised_ = false;
		Reset();
}

bool GazeboRosBattery::SetTemperature(gazebo_ros_battery::SetTemperature::Request& req, gazebo_ros_battery::SetTemperature::Response& res)
{
  service_lock.lock();
  temp_set_ = req.temperature.data;
	ROS_WARN_NAMED(plugin_name_, "%s: Temperature set: %f", gazebo_ros_->info(), temp_set_);
  service_lock.unlock();
  return true;
}

bool GazeboRosBattery::SetCharge(gazebo_ros_battery::SetCharge::Request &req, gazebo_ros_battery::SetCharge::Response &res)
{
  service_lock.lock();
  discharge_ = req.charge.data;
	ROS_WARN_NAMED(plugin_name_, "%s: Charge set: %f", gazebo_ros_->info(), design_capacity_ + discharge_);
  service_lock.unlock();
  return true;
}

bool GazeboRosBattery::ResetModel(gazebo_ros_battery::Reset::Request& req, gazebo_ros_battery::Reset::Response& res)
{
	service_lock.lock();
	Reset();
	service_lock.unlock();
	return true;
}

void GazeboRosBattery::Reset() {
  last_update_time_ = parent->GetWorld()->SimTime();
	charge_ = design_capacity_;
	voltage_ = constant_voltage_;
	temperature_ = design_temperature_;
	temp_set_ = design_temperature_;
	discharge_ = 0;
	current_drawn_ = 0;
	battery_empty_ = false;
	internal_cutt_off_ = false;
	model_initialised_ = true;
	ROS_WARN_NAMED(plugin_name_, "%s: Battery model reset.", gazebo_ros_->info());
}

// simple linear discharge model
void GazeboRosBattery::linear_discharge_voltage_update() {
	voltage_ = constant_voltage_ + lin_discharge_coeff_ * (1 - discharge_ / design_capacity_) - internal_resistance_ * current_lpf_;
}

// Temperature dependent parameters:
//    QT = Q + dQdT*(t-t0)
//    KT = K * np.exp(Kalpha*(1/t-1/t0))
//    E0T = E0 + Tshift*(t-t0)
// v(i) = E0T - KT*QT/(QT-it)*(i*Ctime/3600.0+it) + A*np.exp(-B*it)
//
//  where
//  E0: constant voltage [V]
//  K:  polarization constant [V/Ah] or pol. resistance [Ohm]
//  Q:  maximum capacity [Ah]
//  A:  exponential voltage [V]
//  B:  exponential capacity [A/h]
//  it: already extracted capacity [Ah] (= - discharge)
//  id: current * characteristic time [Ah]
//  i:  battery current [A]
//  T0: design temperature [Celsius]
// Tpol: name of the Vulkan first officer aboard Enterprise NX-01 [Celsius]
// Tshift: temperature offset [Celsius]

void GazeboRosBattery::nonlinear_discharge_voltage_update() {
	double t = temperature_+273.15;
	double t0 = design_temperature_+273.15;
	double E0T = constant_voltage_ + reversible_voltage_temp_*(t-t0); // TODO: Don't increase for t>t0 ?
	double QT = design_capacity_ + capacity_temp_coeff_*(t-t0);       // TODO: Don't increase for t>t0 ?
	double KT = polarization_constant_ * exp(arrhenius_rate_polarization_*(1/t-1/t0));
    voltage_ = E0T
             - KT * QT/(QT + discharge_) * (current_lpf_*(characteristic_time_/3600.0) - discharge_)
             + exponential_voltage_*exp(-exponential_capacity_*-discharge_);
}

// Plugin update function
void GazeboRosBattery::UpdateChild() {
    common::Time current_time = parent->GetWorld()->SimTime();
    double dt = ( current_time - last_update_time_ ).Double();

    if ( dt > update_period_ && model_initialised_ ) {

				double n = dt / temp_lpf_tau_;
				temperature_ = temperature_ + n * (temp_set_ - temperature_);

				if (!battery_empty_) {
			    	// LPF filter on current
					double k = dt / lpf_tau_;
					current_lpf_ = current_lpf_ + k * (current_drawn_ - current_lpf_);
					// Accumulate discharge (negative: 0 to design capacity)
					discharge_ = discharge_ - GZ_SEC_TO_HOUR(dt * current_lpf_);
					if (!internal_cutt_off_) {
						if (use_nonlinear_model_){
							nonlinear_discharge_voltage_update();
						}
						else {
							linear_discharge_voltage_update();
						}
						charge_ = design_capacity_ + discharge_; // discharge is negative
						charge_memory_ = charge_;
					}
					if (voltage_<=cut_off_voltage_ && !internal_cutt_off_) {
						discharge_ = 0;
						voltage_ = 0;
						internal_cutt_off_ = true;
						charge_ = charge_memory_;
						ROS_WARN_NAMED(plugin_name_, "%s: Battery voltage cut off.", gazebo_ros_->info());
					}
				}

				if (!battery_empty_ && charge_<=0) {
					discharge_ = 0;
					current_lpf_ = 0;
					voltage_ = 0;
					battery_empty_ = true;
					ROS_WARN_NAMED(plugin_name_, "%s: Battery discharged.", gazebo_ros_->info());
				}
				// state update
				battery_state_.header.stamp = ros::Time::now();
				battery_state_.voltage    = voltage_;
				battery_state_.current    = current_lpf_;
				battery_state_.charge     = charge_;
				battery_state_.percentage = (charge_/design_capacity_)*100;
				//battery_state_.temperature = temperature_;
				battery_state_publisher_.publish( battery_state_ );
				std_msgs::Float32 battery_voltage_;
				battery_voltage_.data = voltage_;
				if (publish_voltage_) battery_voltage_publisher_.publish( battery_voltage_ );
				last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboRosBattery::FiniChild() {
    gazebo_ros_->node()->shutdown();
}

void GazeboRosBattery::currentCallback(const std_msgs::Float32::ConstPtr& current, int consumer_id) {

	current_drawn_ = 0;
	drawn_currents_[consumer_id]=current->data;
	for (int i=0; i<num_of_consumers_; i++) current_drawn_+=drawn_currents_[i];
	// ROS_WARN("Total current drawn: %f", current_drawn_);

	// temporary solution for simple charging
	if (current_drawn_ <=0){
		battery_state_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
		if (internal_cutt_off_) voltage_ = cut_off_voltage_ + 0.05;
		internal_cutt_off_ = false;
	}
	else {
		battery_state_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

void GazeboRosBattery::QueueThread() {
    static const double timeout = 0.01;
    while ( gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosBattery )
// eof_ns
}
