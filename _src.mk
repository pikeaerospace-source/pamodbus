# /********************************************************************************
# * ╔═╗┬┬┌─┌─┐  ╔═╗┌─┐┬─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐                                       *
# * ╠═╝│├┴┐├┤   ╠═╣├┤ ├┬┘│ │└─┐├─┘├─┤│  ├┤                                        *
# * ╩  ┴┴ ┴└─┘  ╩ ╩└─┘┴└─└─┘└─┘┴  ┴ ┴└─┘└─┘                                       *
# * ╦═╗┌─┐┌─┐┌─┐┌─┐┬─┐┌─┐┬ ┬  ╔═╗┌─┐                                              *
# * ╠╦╝├┤ └─┐├┤ ├─┤├┬┘│  ├─┤  ║  │ │                                              *
# * ╩╚═└─┘└─┘└─┘┴ ┴┴└─└─┘┴ ┴  ╚═╝└─┘o                                             *
# *                                                                               *
# * Copyright © 2022 Pike Aerospace Research Co.                                  *
# *                                                                               *
# ********************************************************************************/
# pamodbus - Pike Aero Modbus Library
#
# This module provides a lightweight MODBUS RTU/TCP implementation.
# Master and slave functionality is controlled by preprocessor defines:
#   PAMODBUS_ENABLE_MASTER=1  - Enable master mode (request building + response parsing)
#   PAMODBUS_ENABLE_SLAVE=1   - Enable slave mode (request parsing + response building)
# Both are enabled by default.

SRC_PAMODBUS=${SRC_ROOT}/pamodbus
INC += -I $(SRC_PAMODBUS)/include

SRCS_CC += $(SRC_PAMODBUS)/src/pamodbus.c
SRCS_CC += $(SRC_PAMODBUS)/src/pdu.c
SRCS_CC += $(SRC_PAMODBUS)/src/framer_rtu.c
SRCS_CC += $(SRC_PAMODBUS)/src/framer_tcp.c
SRCS_CC += $(SRC_PAMODBUS)/src/crc16.c