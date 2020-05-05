GPS_Data_logger.py interfaces with the uBlox neo-7m via UART
at a baud rate of 9600 kb/s

Incoming data must be read in packets of 32 bits

The message is structured as  $MESSAGEID, DATA....
more info available at https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf

each message ID determines what the data means and how it must be read.

TODO: create data structures based off IDs

