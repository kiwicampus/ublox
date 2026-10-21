# MGA-ACK (0x13 0x60)
# Multiple GNSS Acknowledge message
#
# This message is sent by a u-blox receiver to acknowledge the receipt of an
# assistance message. Acknowledgments are enabled by setting ackAiding in
# CFG-NAVX5 (CFG-NAVSPG-ACKAIDING on protocol versions above 23.01).
#

uint8 CLASS_ID = 19
uint8 MESSAGE_ID = 96

uint8 type                  # Type of acknowledgment
uint8 TYPE_NOT_USED = 0     # The message was not used by the receiver, see
                            # info_code
uint8 TYPE_ACCEPTED = 1     # The message was accepted for use by the receiver

uint8 version               # Message version (0x00 for this version)

uint8 info_code             # What the receiver chose to do with the message
uint8 INFO_ACCEPTED = 0         # The receiver accepted the data
uint8 INFO_NO_TIME = 1          # The receiver does not know the time so it
                                # cannot use the data. Send MGA-INI-TIME_UTC
                                # first
uint8 INFO_BAD_VERSION = 2      # The message version is not supported
uint8 INFO_BAD_SIZE = 3         # The message size does not match the version
uint8 INFO_DB_STORE_FAILED = 4  # The data could not be stored to the database
uint8 INFO_NOT_READY = 5        # The receiver is not ready to use the data
uint8 INFO_UNKNOWN_TYPE = 6     # The message type is unknown

uint8 msg_id                # UBX message ID of the acknowledged message
uint8[4] msg_payload_start  # The first 4 bytes of the acknowledged message's
                            # payload
