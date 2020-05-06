#include "AP_OpticalFlow_MAVLink.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "OpticalFlow.h"

#define OPTFLOW_MAV_TIMEOUT_SEC 0.5f

extern const AP_HAL::HAL& hal;


AP_OpticalFlow_MAVLink* AP_OpticalFlow_MAVLink::detect(OpticalFlow& _frontend)
{
	// we assume mavlink messages will be sent into this driver
	AP_OpticalFlow_MAVLink* sensor = new AP_OpticalFlow_MAVLink(_frontend);
	return sensor;
}


// read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_MAVLink::update()
{
	// record gyro values as long as they are being used
	// the sanity check of dt below ensures old gyro values are not used
	if (gyro_sum_count < 1000) {
		const Vector3f& gyro = get_ahrs().get_gyro();
		gyro_sum.x += gyro.x;
		gyro_sum.y += gyro.y;
		gyro_sum_count++;
	}

	// return without updating state if no readings
	if (count == 0) {
		return;
	}

	struct OpticalFlow::OpticalFlow_state state {};

	state.surface_quality = quality_sum / count;

	// calculate dt
	const float dt = (latest_frame_us - prev_frame_us) * 1.0e-6;
	prev_frame_us = latest_frame_us;

	// sanity check dt
	if (is_positive(dt) && (dt < OPTFLOW_MAV_TIMEOUT_SEC)) {
		// calculate flow values
		const float flow_scale_factor_x = 1.0f + 0.001f * _flowScaler().x;
		const float flow_scale_factor_y = 1.0f + 0.001f * _flowScaler().y;

		// copy flow rates to state structure
		state.flowRate = { ((float)flow_sum.x / count) * flow_scale_factor_x * dt,
						   ((float)flow_sum.y / count) * flow_scale_factor_y * dt };

		// copy average body rate to state structure
		state.bodyRate = { gyro_sum.x / gyro_sum_count, gyro_sum.y / gyro_sum_count };

		_applyYaw(state.flowRate);
		_applyYaw(state.bodyRate);
	}
	else {
		// first frame received in some time so cannot calculate flow values
		state.flowRate.zero();
		state.bodyRate.zero();
	}

	_update_frontend(state);

	// reset local buffers
	flow_sum.zero();
	quality_sum = 0;
	count = 0;

	// reset gyro sum
	gyro_sum.zero();
	gyro_sum_count = 0;
}

void AP_OpticalFlow_MAVLink::handle_msg(const mavlink_message_t &msg)
{
	mavlink_optical_flow_t packet;
	mavlink_msg_optical_flow_decode(&msg, &packet);

	// record time message was received
	// ToDo: add jitter correction
	latest_frame_us = AP_HAL::micros64();

	// add sensor values to sum
	flow_sum.x += packet.flow_x;
	flow_sum.y += packet.flow_y;
	quality_sum += packet.quality;
	count++;

	// take sensor id from message
	sensor_id = packet.sensor_id;
    
}
