// MESSAGE PUPPETCOPTER_IMU PACKING

#define MAVLINK_MSG_ID_PUPPETCOPTER_IMU 120

typedef struct __mavlink_puppetcopter_imu_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float roll; ///< Roll angle (rad, -pi..+pi)
 float pitch; ///< Pitch angle (rad, -pi..+pi)
 float yaw; ///< Yaw angle (rad, -pi..+pi)
 float xgyro; ///< Angular speed around X axis (millirad /sec)
 float ygyro; ///< Angular speed around Y axis (millirad /sec)
 float zgyro; ///< Angular speed around Z axis (millirad /sec)
 float xacc; ///< X acceleration (mm/s^2)
 int16_t yacc; ///< Y acceleration (mm/s^2)
 int16_t zacc; ///< Z acceleration (mm/s^2)
} mavlink_puppetcopter_imu_t;

#define MAVLINK_MSG_ID_PUPPETCOPTER_IMU_LEN 36
#define MAVLINK_MSG_ID_120_LEN 36



#define MAVLINK_MESSAGE_INFO_PUPPETCOPTER_IMU { \
	"PUPPETCOPTER_IMU", \
	10, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_puppetcopter_imu_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_puppetcopter_imu_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_puppetcopter_imu_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_puppetcopter_imu_t, yaw) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_puppetcopter_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_puppetcopter_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_puppetcopter_imu_t, zgyro) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_puppetcopter_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_puppetcopter_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_puppetcopter_imu_t, zacc) }, \
         } \
}


/**
 * @brief Pack a puppetcopter_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xacc X acceleration (mm/s^2)
 * @param yacc Y acceleration (mm/s^2)
 * @param zacc Z acceleration (mm/s^2)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_puppetcopter_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float roll, float pitch, float yaw, float xgyro, float ygyro, float zgyro, float xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, xacc);
	_mav_put_int16_t(buf, 32, yacc);
	_mav_put_int16_t(buf, 34, zacc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_puppetcopter_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_PUPPETCOPTER_IMU;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 126);
}

/**
 * @brief Pack a puppetcopter_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xacc X acceleration (mm/s^2)
 * @param yacc Y acceleration (mm/s^2)
 * @param zacc Z acceleration (mm/s^2)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_puppetcopter_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float roll,float pitch,float yaw,float xgyro,float ygyro,float zgyro,float xacc,int16_t yacc,int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, xacc);
	_mav_put_int16_t(buf, 32, yacc);
	_mav_put_int16_t(buf, 34, zacc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_puppetcopter_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_PUPPETCOPTER_IMU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 126);
}

/**
 * @brief Encode a puppetcopter_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param puppetcopter_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_puppetcopter_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_puppetcopter_imu_t* puppetcopter_imu)
{
	return mavlink_msg_puppetcopter_imu_pack(system_id, component_id, msg, puppetcopter_imu->time_boot_ms, puppetcopter_imu->roll, puppetcopter_imu->pitch, puppetcopter_imu->yaw, puppetcopter_imu->xgyro, puppetcopter_imu->ygyro, puppetcopter_imu->zgyro, puppetcopter_imu->xacc, puppetcopter_imu->yacc, puppetcopter_imu->zacc);
}

/**
 * @brief Send a puppetcopter_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xacc X acceleration (mm/s^2)
 * @param yacc Y acceleration (mm/s^2)
 * @param zacc Z acceleration (mm/s^2)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_puppetcopter_imu_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float xgyro, float ygyro, float zgyro, float xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, xgyro);
	_mav_put_float(buf, 20, ygyro);
	_mav_put_float(buf, 24, zgyro);
	_mav_put_float(buf, 28, xacc);
	_mav_put_int16_t(buf, 32, yacc);
	_mav_put_int16_t(buf, 34, zacc);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUPPETCOPTER_IMU, buf, 36, 126);
#else
	mavlink_puppetcopter_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUPPETCOPTER_IMU, (const char *)&packet, 36, 126);
#endif
}

#endif

// MESSAGE PUPPETCOPTER_IMU UNPACKING


/**
 * @brief Get field time_boot_ms from puppetcopter_imu message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_puppetcopter_imu_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from puppetcopter_imu message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_puppetcopter_imu_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from puppetcopter_imu message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_puppetcopter_imu_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from puppetcopter_imu message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_puppetcopter_imu_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field xgyro from puppetcopter_imu message
 *
 * @return Angular speed around X axis (millirad /sec)
 */
static inline float mavlink_msg_puppetcopter_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ygyro from puppetcopter_imu message
 *
 * @return Angular speed around Y axis (millirad /sec)
 */
static inline float mavlink_msg_puppetcopter_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field zgyro from puppetcopter_imu message
 *
 * @return Angular speed around Z axis (millirad /sec)
 */
static inline float mavlink_msg_puppetcopter_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field xacc from puppetcopter_imu message
 *
 * @return X acceleration (mm/s^2)
 */
static inline float mavlink_msg_puppetcopter_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yacc from puppetcopter_imu message
 *
 * @return Y acceleration (mm/s^2)
 */
static inline int16_t mavlink_msg_puppetcopter_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field zacc from puppetcopter_imu message
 *
 * @return Z acceleration (mm/s^2)
 */
static inline int16_t mavlink_msg_puppetcopter_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Decode a puppetcopter_imu message into a struct
 *
 * @param msg The message to decode
 * @param puppetcopter_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_puppetcopter_imu_decode(const mavlink_message_t* msg, mavlink_puppetcopter_imu_t* puppetcopter_imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	puppetcopter_imu->time_boot_ms = mavlink_msg_puppetcopter_imu_get_time_boot_ms(msg);
	puppetcopter_imu->roll = mavlink_msg_puppetcopter_imu_get_roll(msg);
	puppetcopter_imu->pitch = mavlink_msg_puppetcopter_imu_get_pitch(msg);
	puppetcopter_imu->yaw = mavlink_msg_puppetcopter_imu_get_yaw(msg);
	puppetcopter_imu->xgyro = mavlink_msg_puppetcopter_imu_get_xgyro(msg);
	puppetcopter_imu->ygyro = mavlink_msg_puppetcopter_imu_get_ygyro(msg);
	puppetcopter_imu->zgyro = mavlink_msg_puppetcopter_imu_get_zgyro(msg);
	puppetcopter_imu->xacc = mavlink_msg_puppetcopter_imu_get_xacc(msg);
	puppetcopter_imu->yacc = mavlink_msg_puppetcopter_imu_get_yacc(msg);
	puppetcopter_imu->zacc = mavlink_msg_puppetcopter_imu_get_zacc(msg);
#else
	memcpy(puppetcopter_imu, _MAV_PAYLOAD(msg), 36);
#endif
}
