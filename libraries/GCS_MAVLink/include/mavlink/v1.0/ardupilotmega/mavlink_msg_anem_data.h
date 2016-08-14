// MESSAGE ANEM_DATA PACKING

#define MAVLINK_MSG_ID_ANEM_DATA 188

MAVPACKED(
typedef struct __mavlink_anem_data_t {
 float speed_kt; /*< anemometer speed (knots)*/
 int16_t raw_dir; /*< raw anemometer direction (counts)*/
 int16_t cal_dir; /*< anemometer direction calibration (counts)*/
 int16_t dir_cd; /*< anemometer direction calibration (centi-degrees)*/
}) mavlink_anem_data_t;

#define MAVLINK_MSG_ID_ANEM_DATA_LEN 10
#define MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN 10
#define MAVLINK_MSG_ID_188_LEN 10
#define MAVLINK_MSG_ID_188_MIN_LEN 10

#define MAVLINK_MSG_ID_ANEM_DATA_CRC 51
#define MAVLINK_MSG_ID_188_CRC 51



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ANEM_DATA { \
	188, \
	"ANEM_DATA", \
	4, \
	{  { "speed_kt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_anem_data_t, speed_kt) }, \
         { "raw_dir", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_anem_data_t, raw_dir) }, \
         { "cal_dir", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_anem_data_t, cal_dir) }, \
         { "dir_cd", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_anem_data_t, dir_cd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ANEM_DATA { \
	"ANEM_DATA", \
	4, \
	{  { "speed_kt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_anem_data_t, speed_kt) }, \
         { "raw_dir", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_anem_data_t, raw_dir) }, \
         { "cal_dir", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_anem_data_t, cal_dir) }, \
         { "dir_cd", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_anem_data_t, dir_cd) }, \
         } \
}
#endif

/**
 * @brief Pack a anem_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param raw_dir raw anemometer direction (counts)
 * @param cal_dir anemometer direction calibration (counts)
 * @param dir_cd anemometer direction calibration (centi-degrees)
 * @param speed_kt anemometer speed (knots)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_anem_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t raw_dir, int16_t cal_dir, int16_t dir_cd, float speed_kt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANEM_DATA_LEN];
	_mav_put_float(buf, 0, speed_kt);
	_mav_put_int16_t(buf, 4, raw_dir);
	_mav_put_int16_t(buf, 6, cal_dir);
	_mav_put_int16_t(buf, 8, dir_cd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ANEM_DATA_LEN);
#else
	mavlink_anem_data_t packet;
	packet.speed_kt = speed_kt;
	packet.raw_dir = raw_dir;
	packet.cal_dir = cal_dir;
	packet.dir_cd = dir_cd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ANEM_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANEM_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN, MAVLINK_MSG_ID_ANEM_DATA_LEN, MAVLINK_MSG_ID_ANEM_DATA_CRC);
}

/**
 * @brief Pack a anem_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param raw_dir raw anemometer direction (counts)
 * @param cal_dir anemometer direction calibration (counts)
 * @param dir_cd anemometer direction calibration (centi-degrees)
 * @param speed_kt anemometer speed (knots)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_anem_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t raw_dir,int16_t cal_dir,int16_t dir_cd,float speed_kt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANEM_DATA_LEN];
	_mav_put_float(buf, 0, speed_kt);
	_mav_put_int16_t(buf, 4, raw_dir);
	_mav_put_int16_t(buf, 6, cal_dir);
	_mav_put_int16_t(buf, 8, dir_cd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ANEM_DATA_LEN);
#else
	mavlink_anem_data_t packet;
	packet.speed_kt = speed_kt;
	packet.raw_dir = raw_dir;
	packet.cal_dir = cal_dir;
	packet.dir_cd = dir_cd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ANEM_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ANEM_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN, MAVLINK_MSG_ID_ANEM_DATA_LEN, MAVLINK_MSG_ID_ANEM_DATA_CRC);
}

/**
 * @brief Encode a anem_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param anem_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_anem_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_anem_data_t* anem_data)
{
	return mavlink_msg_anem_data_pack(system_id, component_id, msg, anem_data->raw_dir, anem_data->cal_dir, anem_data->dir_cd, anem_data->speed_kt);
}

/**
 * @brief Encode a anem_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param anem_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_anem_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_anem_data_t* anem_data)
{
	return mavlink_msg_anem_data_pack_chan(system_id, component_id, chan, msg, anem_data->raw_dir, anem_data->cal_dir, anem_data->dir_cd, anem_data->speed_kt);
}

/**
 * @brief Send a anem_data message
 * @param chan MAVLink channel to send the message
 *
 * @param raw_dir raw anemometer direction (counts)
 * @param cal_dir anemometer direction calibration (counts)
 * @param dir_cd anemometer direction calibration (centi-degrees)
 * @param speed_kt anemometer speed (knots)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_anem_data_send(mavlink_channel_t chan, int16_t raw_dir, int16_t cal_dir, int16_t dir_cd, float speed_kt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ANEM_DATA_LEN];
	_mav_put_float(buf, 0, speed_kt);
	_mav_put_int16_t(buf, 4, raw_dir);
	_mav_put_int16_t(buf, 6, cal_dir);
	_mav_put_int16_t(buf, 8, dir_cd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANEM_DATA, buf, MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN, MAVLINK_MSG_ID_ANEM_DATA_LEN, MAVLINK_MSG_ID_ANEM_DATA_CRC);
#else
	mavlink_anem_data_t packet;
	packet.speed_kt = speed_kt;
	packet.raw_dir = raw_dir;
	packet.cal_dir = cal_dir;
	packet.dir_cd = dir_cd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANEM_DATA, (const char *)&packet, MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN, MAVLINK_MSG_ID_ANEM_DATA_LEN, MAVLINK_MSG_ID_ANEM_DATA_CRC);
#endif
}

/**
 * @brief Send a anem_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_anem_data_send_struct(mavlink_channel_t chan, const mavlink_anem_data_t* anem_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_anem_data_send(chan, anem_data->raw_dir, anem_data->cal_dir, anem_data->dir_cd, anem_data->speed_kt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANEM_DATA, (const char *)anem_data, MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN, MAVLINK_MSG_ID_ANEM_DATA_LEN, MAVLINK_MSG_ID_ANEM_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_ANEM_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_anem_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t raw_dir, int16_t cal_dir, int16_t dir_cd, float speed_kt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, speed_kt);
	_mav_put_int16_t(buf, 4, raw_dir);
	_mav_put_int16_t(buf, 6, cal_dir);
	_mav_put_int16_t(buf, 8, dir_cd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANEM_DATA, buf, MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN, MAVLINK_MSG_ID_ANEM_DATA_LEN, MAVLINK_MSG_ID_ANEM_DATA_CRC);
#else
	mavlink_anem_data_t *packet = (mavlink_anem_data_t *)msgbuf;
	packet->speed_kt = speed_kt;
	packet->raw_dir = raw_dir;
	packet->cal_dir = cal_dir;
	packet->dir_cd = dir_cd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ANEM_DATA, (const char *)packet, MAVLINK_MSG_ID_ANEM_DATA_MIN_LEN, MAVLINK_MSG_ID_ANEM_DATA_LEN, MAVLINK_MSG_ID_ANEM_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE ANEM_DATA UNPACKING


/**
 * @brief Get field raw_dir from anem_data message
 *
 * @return raw anemometer direction (counts)
 */
static inline int16_t mavlink_msg_anem_data_get_raw_dir(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field cal_dir from anem_data message
 *
 * @return anemometer direction calibration (counts)
 */
static inline int16_t mavlink_msg_anem_data_get_cal_dir(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field dir_cd from anem_data message
 *
 * @return anemometer direction calibration (centi-degrees)
 */
static inline int16_t mavlink_msg_anem_data_get_dir_cd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field speed_kt from anem_data message
 *
 * @return anemometer speed (knots)
 */
static inline float mavlink_msg_anem_data_get_speed_kt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a anem_data message into a struct
 *
 * @param msg The message to decode
 * @param anem_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_anem_data_decode(const mavlink_message_t* msg, mavlink_anem_data_t* anem_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	anem_data->speed_kt = mavlink_msg_anem_data_get_speed_kt(msg);
	anem_data->raw_dir = mavlink_msg_anem_data_get_raw_dir(msg);
	anem_data->cal_dir = mavlink_msg_anem_data_get_cal_dir(msg);
	anem_data->dir_cd = mavlink_msg_anem_data_get_dir_cd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ANEM_DATA_LEN? msg->len : MAVLINK_MSG_ID_ANEM_DATA_LEN;
        memset(anem_data, 0, MAVLINK_MSG_ID_ANEM_DATA_LEN);
	memcpy(anem_data, _MAV_PAYLOAD(msg), len);
#endif
}
