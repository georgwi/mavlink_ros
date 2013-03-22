// MESSAGE PUPPETCOPTER_IMU PACKING

#define MAVLINK_MSG_ID_PUPPETCOPTER_IMU 120

typedef struct __mavlink_puppetcopter_imu_t
{
 float pitch; ///< Pitch angle
 float roll; ///< Roll angle
 float yaw; ///< Yaw angle
 float xgyro; ///< Angular velocity around x
 float ygyro; ///< Angular velocity around y
 float zgyro; ///< Angular velocity around z
 float xacc; ///< Linear acceleration in x direction
 float yacc; ///< Linear acceleration in x direction
 float zacc; ///< Linear acceleration in x direction
 float slot; ///< Empty slot for tests
} mavlink_puppetcopter_imu_t;

#define MAVLINK_MSG_ID_PUPPETCOPTER_IMU_LEN 40
#define MAVLINK_MSG_ID_120_LEN 40



#define MAVLINK_MESSAGE_INFO_PUPPETCOPTER_IMU { \
	"PUPPETCOPTER_IMU", \
	10, \
	{  { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_puppetcopter_imu_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_puppetcopter_imu_t, roll) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_puppetcopter_imu_t, yaw) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_puppetcopter_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_puppetcopter_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_puppetcopter_imu_t, zgyro) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_puppetcopter_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_puppetcopter_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_puppetcopter_imu_t, zacc) }, \
         { "slot", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_puppetcopter_imu_t, slot) }, \
         } \
}


/**
 * @brief Pack a puppetcopter_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch Pitch angle
 * @param roll Roll angle
 * @param yaw Yaw angle
 * @param xgyro Angular velocity around x
 * @param ygyro Angular velocity around y
 * @param zgyro Angular velocity around z
 * @param xacc Linear acceleration in x direction
 * @param yacc Linear acceleration in x direction
 * @param zacc Linear acceleration in x direction
 * @param slot Empty slot for tests
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_puppetcopter_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float pitch, float roll, float yaw, float xgyro, float ygyro, float zgyro, float xacc, float yacc, float zacc, float slot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_float(buf, 0, pitch);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xgyro);
	_mav_put_float(buf, 16, ygyro);
	_mav_put_float(buf, 20, zgyro);
	_mav_put_float(buf, 24, xacc);
	_mav_put_float(buf, 28, yacc);
	_mav_put_float(buf, 32, zacc);
	_mav_put_float(buf, 36, slot);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 40);
#else
	mavlink_puppetcopter_imu_t packet;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.slot = slot;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_PUPPETCOPTER_IMU;
	return mavlink_finalize_message(msg, system_id, component_id, 40, 42);
}

/**
 * @brief Pack a puppetcopter_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitch Pitch angle
 * @param roll Roll angle
 * @param yaw Yaw angle
 * @param xgyro Angular velocity around x
 * @param ygyro Angular velocity around y
 * @param zgyro Angular velocity around z
 * @param xacc Linear acceleration in x direction
 * @param yacc Linear acceleration in x direction
 * @param zacc Linear acceleration in x direction
 * @param slot Empty slot for tests
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_puppetcopter_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float pitch,float roll,float yaw,float xgyro,float ygyro,float zgyro,float xacc,float yacc,float zacc,float slot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_float(buf, 0, pitch);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xgyro);
	_mav_put_float(buf, 16, ygyro);
	_mav_put_float(buf, 20, zgyro);
	_mav_put_float(buf, 24, xacc);
	_mav_put_float(buf, 28, yacc);
	_mav_put_float(buf, 32, zacc);
	_mav_put_float(buf, 36, slot);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 40);
#else
	mavlink_puppetcopter_imu_t packet;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.slot = slot;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_PUPPETCOPTER_IMU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 40, 42);
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
	return mavlink_msg_puppetcopter_imu_pack(system_id, component_id, msg, puppetcopter_imu->pitch, puppetcopter_imu->roll, puppetcopter_imu->yaw, puppetcopter_imu->xgyro, puppetcopter_imu->ygyro, puppetcopter_imu->zgyro, puppetcopter_imu->xacc, puppetcopter_imu->yacc, puppetcopter_imu->zacc, puppetcopter_imu->slot);
}

/**
 * @brief Send a puppetcopter_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param pitch Pitch angle
 * @param roll Roll angle
 * @param yaw Yaw angle
 * @param xgyro Angular velocity around x
 * @param ygyro Angular velocity around y
 * @param zgyro Angular velocity around z
 * @param xacc Linear acceleration in x direction
 * @param yacc Linear acceleration in x direction
 * @param zacc Linear acceleration in x direction
 * @param slot Empty slot for tests
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_puppetcopter_imu_send(mavlink_channel_t chan, float pitch, float roll, float yaw, float xgyro, float ygyro, float zgyro, float xacc, float yacc, float zacc, float slot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];
	_mav_put_float(buf, 0, pitch);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xgyro);
	_mav_put_float(buf, 16, ygyro);
	_mav_put_float(buf, 20, zgyro);
	_mav_put_float(buf, 24, xacc);
	_mav_put_float(buf, 28, yacc);
	_mav_put_float(buf, 32, zacc);
	_mav_put_float(buf, 36, slot);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUPPETCOPTER_IMU, buf, 40, 42);
#else
	mavlink_puppetcopter_imu_t packet;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.slot = slot;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUPPETCOPTER_IMU, (const char *)&packet, 40, 42);
#endif
}

#endif

// MESSAGE PUPPETCOPTER_IMU UNPACKING


/**
 * @brief Get field pitch from puppetcopter_imu message
 *
 * @return Pitch angle
 */
static inline float mavlink_msg_puppetcopter_imu_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field roll from puppetcopter_imu message
 *
 * @return Roll angle
 */
static inline float mavlink_msg_puppetcopter_imu_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from puppetcopter_imu message
 *
 * @return Yaw angle
 */
static inline float mavlink_msg_puppetcopter_imu_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field xgyro from puppetcopter_imu message
 *
 * @return Angular velocity around x
 */
static inline float mavlink_msg_puppetcopter_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ygyro from puppetcopter_imu message
 *
 * @return Angular velocity around y
 */
static inline float mavlink_msg_puppetcopter_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field zgyro from puppetcopter_imu message
 *
 * @return Angular velocity around z
 */
static inline float mavlink_msg_puppetcopter_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field xacc from puppetcopter_imu message
 *
 * @return Linear acceleration in x direction
 */
static inline float mavlink_msg_puppetcopter_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yacc from puppetcopter_imu message
 *
 * @return Linear acceleration in x direction
 */
static inline float mavlink_msg_puppetcopter_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field zacc from puppetcopter_imu message
 *
 * @return Linear acceleration in x direction
 */
static inline float mavlink_msg_puppetcopter_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field slot from puppetcopter_imu message
 *
 * @return Empty slot for tests
 */
static inline float mavlink_msg_puppetcopter_imu_get_slot(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
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
	puppetcopter_imu->pitch = mavlink_msg_puppetcopter_imu_get_pitch(msg);
	puppetcopter_imu->roll = mavlink_msg_puppetcopter_imu_get_roll(msg);
	puppetcopter_imu->yaw = mavlink_msg_puppetcopter_imu_get_yaw(msg);
	puppetcopter_imu->xgyro = mavlink_msg_puppetcopter_imu_get_xgyro(msg);
	puppetcopter_imu->ygyro = mavlink_msg_puppetcopter_imu_get_ygyro(msg);
	puppetcopter_imu->zgyro = mavlink_msg_puppetcopter_imu_get_zgyro(msg);
	puppetcopter_imu->xacc = mavlink_msg_puppetcopter_imu_get_xacc(msg);
	puppetcopter_imu->yacc = mavlink_msg_puppetcopter_imu_get_yacc(msg);
	puppetcopter_imu->zacc = mavlink_msg_puppetcopter_imu_get_zacc(msg);
	puppetcopter_imu->slot = mavlink_msg_puppetcopter_imu_get_slot(msg);
#else
	memcpy(puppetcopter_imu, _MAV_PAYLOAD(msg), 40);
#endif
}
