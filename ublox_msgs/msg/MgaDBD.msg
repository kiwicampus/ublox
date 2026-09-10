# MGA-DBD (0x13 0x80)
# Navigation database dump/restore
#
# Polling this message makes the receiver return its whole internal navigation
# database (ephemeris, almanac, time, position, health, ionosphere model, etc.)
# as a series of MGA-DBD messages. Sending those same messages back into any
# receiver restores/seeds its database. The payload is an opaque u-blox blob;
# it is not meant to be parsed by the host, only replayed verbatim.

uint8 CLASS_ID = 19
uint8 MESSAGE_ID = 128

uint8[] data            # opaque navigation database blob
