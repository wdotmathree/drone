#include "secrets.hpp"

#include <iostream>
#include <mutex>

#include <i2c_master.hpp>
#include <interrupt.hpp>
#include <madgwick_filter.hpp>
#include <pid.hpp>
#include <udp_socket.hpp>
#include <wifi_ap.hpp>

#include <driver/ledc.h>

#define ACC_SCALE 4
#define GYRO_SCALE 500
#define CALIBRATION_ITS 600

#define NW_GPIO 16
#define NE_GPIO 17
#define SE_GPIO 18
#define SW_GPIO 19

espp::I2cMasterBus
	bus(espp::I2cMasterBus::Config{
		.sda_io_num = 14,
		.scl_io_num = 15,
		.clk_speed = 400000,
	});
std::shared_ptr<espp::I2cMasterDevice<uint8_t>> imu_device;

espp::MadgwickFilter filter; // Default beta = 0.1

int16_t buf[8];
int32_t bias_tmp[6] = {0};
int16_t bias[6];

bool init(void) {
	std::error_code ec;
	if (!bus.init(ec)) {
		std::cerr << "Failed to initialize I2C bus: " << ec.message() << std::endl;
		return false;
	}
	imu_device = bus.add_device<uint8_t>(
		espp::I2cMasterDevice<uint8_t>::Config{
			.device_address = 0x68,
			.scl_speed_hz = 400000,
		},
		ec
	);
	if (ec) {
		std::cerr << "Failed to add I2C device: " << ec.message() << std::endl;
		return false;
	}
	buf[0] = 0x00;
	buf[1] = 0x78;
	buf[2] = 0x44;
	buf[3] = 0x09;
	buf[4] = 0x08;
	buf[5] = 0x08;
	imu_device->write_register(0x38, (uint8_t *)buf + 0, 1, ec); // Disable interrupts
	imu_device->write_register(0x23, (uint8_t *)buf + 2, 1, ec); // Enable accelerometer and gyroscope in FIFO
	imu_device->write_register(0x6a, (uint8_t *)buf + 4, 1, ec); // Enable and reset FIFO
	imu_device->write_register(0x6b, (uint8_t *)buf + 6, 1, ec); // Wake up the device, disable temperature, PLL with x gyroscope
	imu_device->write_register(0x1b, (uint8_t *)buf + 8, 1, ec); // 500dps gyroscope scale
	imu_device->write_register(0x1c, (uint8_t *)buf + 10, 1, ec); // 4g accelerometer scale

	return true;
}

extern "C" void app_main(void) {
	std::error_code ec;
	if (!init())
		return;

	// Initialize WiFi AP
	espp::WifiAp wifi_ap({
		.ssid = WIFI_AP_SSID,
		.password = WIFI_AP_PASSWORD,
		.channel = 1,
		.max_number_of_stations = 1,
	});
	wifi_ap.start();

	// Calibrate
	vTaskDelay(10);
	for (int i = 0; i < CALIBRATION_ITS; i++) {
		imu_device->read_register(0x3b, (uint8_t *)buf, 14, ec);
		if (ec) {
			i--;
			continue;
		}

		for (int i = 0; i < 3; i++)
			bias_tmp[i] += (int16_t)__bswap16(buf[i]);
		for (int i = 3; i < 6; i++)
			bias_tmp[i] += (int16_t)__bswap16(buf[i + 1]);
	}
	for (int i = 0; i < 6; i++)
		bias[i] = bias_tmp[i] / CALIBRATION_ITS;

	// Initialize LEDC for motor control
	ledc_timer_config_t cfg = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = 100,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ledc_timer_config(&cfg);
	ledc_channel_config_t nw_cfg = {
		.gpio_num = NW_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ledc_channel_config(&nw_cfg);
	ledc_channel_config_t ne_cfg = {
		.gpio_num = NE_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_1,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ledc_channel_config(&ne_cfg);
	ledc_channel_config_t se_cfg = {
		.gpio_num = SE_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_2,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ledc_channel_config(&se_cfg);
	ledc_channel_config_t sw_cfg = {
		.gpio_num = SW_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_3,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ledc_channel_config(&sw_cfg);

	// Initialize PIDs (default values)
	espp::Pid::Config pit_cfg = {
		.kp = 0.5f,
		.ki = 0.0f,
		.kd = 0.0f,
		.integrator_min = -1.0f,
		.integrator_max = 1.0f,
		.output_min = -1.0f,
		.output_max = 1.0f,
	};
	espp::Pid pit_pid(pit_cfg);
	espp::Pid::Config rol_cfg = {
		.kp = 0.5f,
		.ki = 0.0f,
		.kd = 0.0f,
		.integrator_min = -1.0f,
		.integrator_max = 1.0f,
		.output_min = -1.0f,
		.output_max = 1.0f,
	};
	espp::Pid rol_pid(rol_cfg);
	espp::Pid::Config yaw_cfg = {
		.kp = 0.5f,
		.ki = 0.0f,
		.kd = 0.0f,
		.integrator_min = -1.0f,
		.integrator_max = 1.0f,
		.output_min = -1.0f,
		.output_max = 1.0f,
	};
	espp::Pid yaw_pid(yaw_cfg);
	espp::Pid::Config acc_cfg = {
		.kp = 0.5f,
		.ki = 0.0f,
		.kd = 0.0f,
		.integrator_min = -1.0f,
		.integrator_max = 1.0f,
		.output_min = 0.0f,
		.output_max = 1.0f,
	};
	espp::Pid acc_pid(acc_cfg);

	// Controller packet handlers
	std::atomic<bool> ready = false;
	espp::UdpSocket socket({.log_level = espp::Logger::Verbosity::WARN});
	espp::Task::BaseConfig task_config{
		.name = "UdpSocketTask",
		.stack_size_bytes = 4096,
		.priority = 1,
	};
	espp::UdpSocket::ReceiveConfig receive_config{
		.port = 12345,
		.buffer_size = 1024,
		.is_multicast_endpoint = false,
		.on_receive_callback = [&](std::vector<uint8_t> &data, const espp::Socket::Info &sender_info) {
			if (data.size() < 1)
				return std::nullopt;

			if (data[0] == 'R') { // Ready command
				ready.store(true);
				printf("Controller is ready!\n");
			} else if (data[0] == 'S') { // Stop command
				ready.store(false);
				printf("Controller is stopped!\n");
			} else if (data[0] == 'C') {
				float *options = (float *)(data.data() + 2);
				if (data[1] == 'P') {
					pit_cfg.kp = options[0];
					pit_cfg.ki = options[1];
					pit_cfg.kd = options[2];
					pit_cfg.integrator_min = options[3];
					pit_cfg.integrator_max = options[4];
					pit_pid.set_config(pit_cfg);
				} else if (data[1] == 'R') {
					rol_cfg.kp = options[0];
					rol_cfg.ki = options[1];
					rol_cfg.kd = options[2];
					rol_cfg.integrator_min = options[3];
					rol_cfg.integrator_max = options[4];
					rol_pid.set_config(rol_cfg);
				} else if (data[1] == 'Y') {
					yaw_cfg.kp = options[0];
					yaw_cfg.ki = options[1];
					yaw_cfg.kd = options[2];
					yaw_cfg.integrator_min = options[3];
					yaw_cfg.integrator_max = options[4];
					yaw_pid.set_config(yaw_cfg);
				} else if (data[1] == 'A') {
					acc_cfg.kp = options[0];
					acc_cfg.ki = options[1];
					acc_cfg.kd = options[2];
					acc_cfg.integrator_min = options[3];
					acc_cfg.integrator_max = options[4];
					acc_pid.set_config(acc_cfg);
				}
			}
			return std::nullopt;
		},
	};
	socket.start_receiving(task_config, receive_config);

	// Get precise time
	uint64_t prev = esp_timer_get_time();
	// vTaskDelay(1);
	while (true) {
		// Shutdown engines if controller is not ready
		if (!ready.load()) {
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

			vTaskDelay(100 / portTICK_PERIOD_MS);
			prev = esp_timer_get_time();
			continue;
		}

		// Get sensor data
		imu_device->read_register(0x3b, (uint8_t *)buf, 14, ec);
		if (ec)
			continue;
		uint64_t now = esp_timer_get_time();

		// Convert to float
		float az = (float)(int16_t)__bswap16(buf[0]) / (32768.0f / ACC_SCALE);
		float ay = (float)(int16_t)__bswap16(buf[1]) / (32768.0f / ACC_SCALE);
		float ax = -(float)(int16_t)__bswap16(buf[2]) / (32768.0f / ACC_SCALE);
		float gz = (float)((int16_t)__bswap16(buf[4]) - bias[3]) / (32768.0f / GYRO_SCALE) * (M_PI / 180.0f);
		float gy = (float)((int16_t)__bswap16(buf[5]) - bias[4]) / (32768.0f / GYRO_SCALE) * (M_PI / 180.0f);
		float gx = -(float)((int16_t)__bswap16(buf[6]) - bias[5]) / (32768.0f / GYRO_SCALE) * (M_PI / 180.0f);

		// Normalize accelerometer values and update filter
		float mag = espp::fast_inv_sqrt(ax * ax + ay * ay + az * az);
		filter.update((now - prev) / 1000000.0f, ax * mag, ay * mag, az * mag, gx, gy, gz);
		prev = now;

		// Should be (pitch, roll, yaw)
		filter.get_euler(gx, gy, gz);
		// printf("%11.6f %11.6f %11.6f\n", gx, gy, gz);

		// Mock values for testing
		float req_gx = 0; // Requested pitch
		float req_gy = 0; // Requested roll
		float req_gz = 0; // Requested yaw
		float req_va = 0; // Requested vertical acceleration

		// Current values
		float cur_pit = gx;
		float cur_rol = gy;
		float cur_yaw = gz;
		float cur_acc = espp::fast_cos(gx) * espp::fast_cos(gy) * az;

		// Input into PID controller
		float pit_out = pit_pid(req_gx - cur_pit);
		float rol_out = rol_pid(req_gy - cur_rol);
		float yaw_out = yaw_pid(req_gz - cur_yaw);
		float acc_out = acc_pid(req_va - cur_acc);

		// Mix and clamp outputs
		// Need sqrt because thrust is proportional to square of speed
		float nw = std::sqrtf(std::clamp(acc_out + pit_out + rol_out + yaw_out, 0.0f, 1.0f));
		float ne = std::sqrtf(std::clamp(acc_out + pit_out - rol_out - yaw_out, 0.0f, 1.0f));
		float se = std::sqrtf(std::clamp(acc_out - pit_out - rol_out + yaw_out, 0.0f, 1.0f));
		float sw = std::sqrtf(std::clamp(acc_out - pit_out + rol_out - yaw_out, 0.0f, 1.0f));

		// Print outputs
		printf("NW: %11.6f, NE: %11.6f, SE: %11.6f, SW: %11.6f\n", nw, ne, se, sw);

		// Set outputs to motors
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (nw + 1) * 51);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (ne + 1) * 51);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (se + 1) * 51);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, (sw + 1) * 51);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
	}
}
