# DroneCAN/libcanard base
canard.c, canard.h, and canard_internals.h all come from libcanard https://github.com/dronecan/libcanard

dsdlc_generated hold the generated DSDL files (see https://github.com/dronecan/DSDL and https://github.com/dronecan/dronecan_dsdlc)

canard_stm32_driver.c/.h is our implementation. It's similar to the stm32 driver in the libcanard repo but we used HAL instead of the register addresses.

node_settings.h has the macro for NODE_ID which is important to set for each node

To add handling for receiving a new message ID:
1. Add to shouldAcceptTransfer
2. Add to onTransferReceived
3. Write the relevant handle_ function (Check existing function implementations for how to do this)

To transmit a broadcast new message:
1. Write the send_ function (Check existing functoin implementations for how to do this)
2. Call that function at a frequency within the main loop

To respond to a message:
1. Follow the steps for handling a message receive
2. Then call canardRequestOrRespond at the end of the handler similar to handle_GetNodeInfo implementation

To transmit a request message:
1. Follow the steps for transmitting a broadcast message.
2. Use canardRequestOrRespond() with canardRequest selected instead of canardBroadcast()

Also relevant: https://dronecan.github.io/Specification/7._List_of_standard_data_types/
