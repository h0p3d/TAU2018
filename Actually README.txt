RF Sense Protocols for Silicon Labs EFR32G14 Board
Hope Dargan (hoped@mit.edu)
June 2018

Looking at the main method of the main.c file you will see that there are four modes in an infinite loop. 
The purpose of this document is to explain what each mode does.

PacketTx mode:
	This is the transmission mode. It sends the radio into an idle state and then transmits the desired packet. 
	It is important to note that only the first packet sent each wakeup cycle will be a wakeup packet. All other packets
sent during the wake cycle will be id packets.
	If sleep mode ended due to RFsense energy, on wakeup send a wakeup packet followed by an id packet (contained in txData array.) 
The board that caused the wakeup will then be able to respond to the id packet. If woken up by the sleep timer, send a 
wake packet (contained in txWakeup array.)  
 
PacketRx mode:
	 Simply idles the radio and then prepares to receive a packet. I think that the radio cannot both transmit and receive 
at the same time, which is why the radio must idle before entering either mode.
	In order to see the meat of what receive mode does, go to the RAIL_CbGeneric method. When an RAIL_EVENT_RX_PACKET_RECEIVED 
event is triggered the following occurs. If the packet is successfully received, it checks that the packet is an id packet by
first comparing length (all id packets have the same length) and then by comparing the header (the first HEADER_LENGTH 
entries of txData array is the header. Must be the same for all id packets.) If both match, the program checks to make sure
the id packet has not already been added to the packetArray (stores id packets received for current wake session) before
adding it. This prevents unnecessary log entries.
	   
Sleep mode:
	Before going to sleep, the board stores all of the id packets received during the current wake cycle using the logPackets method. 
Sleep mode sends the board to sleep for SLEEP_TIME_MS using the sleep method. It can be woken and put into wake 
mode in one of two ways. 1) The sleep timer runs out. 2) RFsense is triggered and wakes the board up.
	Snooze mode is an optional feature of sleep mode, enabled by toggling SNOOZE_ENABLED. If snooze mode is enabled, 
the board tracks the number of times it was woken by rfsense without receiving any id packets with the variable rfWakeCount.
If rfWakeCount exceeds wakeLimit, the board is put to sleep for SNOOZE_TIME_MS. In snooze mode the board can only be woken
by the sleep timer running out, RFsense is disabled. Snooze mode is intended to save battery by preventing unnecessary 
rfSense wakeups due to proximity to cell towers or the like.

Wake Mode: 
	Wake mode essentially enables transmit mode and receive mode for a period of WAKE_TIME_MS using the timer_set method. 
When the board wakes, it resets various counters for receive and transmit mode, enables packetTx so the wakeup packet can be sent.
When the wake timer expires, it triggers sleep mode.

Testing Mode:
	Testing mode was created specifically for use with the Silicon Labs development kit board EFR32G14. 
It enables the use of the buttons on the board – button 0 (BP0) sends the board into transmit mode 
(as long as it is not currently sleeping) and button 1 (BP1) sends the board into sleep mode in order to start the 
sleep / wake up cycle.
	Testing mode records various types of data and displays it to make experimentation easier.	

To Dos:
	The only capability the rfsense program lacks is a way to transmit packetLog data to a base station. 
For now, testing mode prints the data to the screen along with other statistics during testing.

Note About Simplicity Studios: 
	First, all of the code I have written is contained in the main.c file. 
	The two most important parts of the project are the main.c file and the RFsense.isc file. The .isc file generates the 
radio configuration, the main file enables the protocols. 
	
	In order to copy the project, 
go to simplicity studios and copy and paste the desired project, then go into the .isc file and click generate so 
the radio configuration code is generated and then you can debug and test the project. 

	In order to run the project, 
just right click the project folder and then select Debug As… This will load the project onto the board of your choice. 
You can disconnect the board from the debugger once the code is loaded and use other applications like the energy profiler 
but you cannot use both at the same time.

	In order to see testing mode data: 
You can see the data when the board is running by going to debug adapters (clearly visible from launcher perspective), 
right clicking one of the boards, selecting launch console and then hitting enter on the serial 1 display.

A note about the console – 
it works while using debug or energy profiler mode. However, enter Energy profile mode before launching the console.
If the console has been launched first, starting energy capture will sometimes just show a blank screen for the board that 
has its console open. The way to fix this is by closing simplicity studios, unplugging the boards from the computer, plugging 
them in again and then opening Simplicity studios. The boards will still have the same applications uploaded even if they lose 
power.  Then go back to energy profiler first and start energy capture before launching the console again.
