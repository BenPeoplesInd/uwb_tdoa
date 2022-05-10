# Overview

In 2018, we developed a project that never went live.  (Well, it went live, but not with our technology).   I cannot go into details of that project, BUT we can publish our methods and code and whatnot.

# Infrastructure

For this project, we needed to track several dozen mobile users who all had on wearable technology.   So we could track these devices.    The devices needed to react to their location, and our strategy at the time was basically to put DWM1000 modules in each of the devices along with an ESP32.   The DWM1000 modules would transmit UWB pings, these would be picked up by ceiling mounted Anchors, the data sent to a central computer that would calculate each device's location and then transmit it back over WiFi.   

Several aspects of this did not go well, BUT the localization stuff more or less worked by the end of it.

### TDOA approach

There are a few different ways to do localization with UWB devices.   One involves positioning anchors in space, which if you have enough of them can figure out where they are relative to each other via UWB pings, and then sending a ping and getting a pong back from the anchors.   This lets you figure out your distance to each anchor, and then multilateralize your way to a position fix.

Because we had a target of about 200 devices, and needed maybe 1 update per second to actually work, we couldn't do this.   So we implemented TDOA.   TDOA stands for Time Difference of Arrival.

Essentially, the mobile device ("device") sends out a ping that says "I am device 37, ping 1".   Each anchor receives this ping and transmits its payload along with the timestamp at which it recieved it to a central server.   That server knows the location of each anchor as well as the propogation delay of its cabling (that is: the anchors who have longer cables to the clock distribution device will be further behind anchors with shorter cables-- we'll get to this toplogy in a moment).    These offsets are taken into account, the pings are plugged into an algorithm, and the algorithm produces a position fix.   

### LUT approach

In addition to solving the position fixes algorithmically, you can also just make a giant lookup table.  For each position in the space, the TDOA values are known to each anchor.   Depending on your required precision and how many bytes you want to store, you can make fairly compact LUTs.   If your anchors are permanently installed, you can use the same hardware and send pings out from the anchors to the devices-- each device can use a LUT to determine its location.    These tables can get rather large-- several megabytes to several hundred megabytes-- but this is a reasonable chunk of flash and avoids doing complex math on a small device.

# Hardware

## Anchors

Anchors are a specalized bit of hardware that is not really your stock DWM1000 module, in particular they involve replacing the crystal in the module with an external clock source, so while it's possible to use the stock DWM modules (we did), they require modification to work.

An anchor has two external signals coming into the DWM module: a clock and a sync pulse.   When the DWM receives a sync pulse, it resets its clock counter to 0.   Using this, you can get all of the DWM modules in all of the anchors to be counting in sync (because you distribute a 38.4MHz clock to all of the modules in the system).   

So it's really a two-part system: you have a clock and sync distribution system (which is possible to do over cat5+ cabling) and some sort of data network (which could be WiFi, but in our build was Ethernet.   We also made the modules PoE powered so we just had to run the two cables to them.  Because we were running them as 10/100 ethernet, we COULD actually have run a single ethernet cable to the anchors and had splitters at the other end that combined the Ethernet, PoE, and clock/sync signals into a single cable.   But we did not.

The anchors we built consist of an ESP32 module + Ethernet PHY, a clock jitter cleaner (SI5317D-C-GM), a high speed analog comparator (for sync signal, which is a GPIO-style signal, but transmitted balanced), and the DWM1000.    

The DWM1000 is attached to the ESP32 via SPI, and it just polls the DWM to see if it got a packet, if it did, it pulls the data and transmits it over to the server.   It's a lossy process: many packets are lost, but if you have enough anchors and devices transmit often enough, you frequently get fixes (and we can hit the 1 fix per second target).

## Clock Distributor

This is a device that generates a 38.4MHz signal and then fans it out to a bunch of outputs and transmits it as a balanced signal.   We used 2-channel LVDS drivers for each output.   One takes the single-ended clock signal and balances it, the other takes the sync signal.

## Device

The mobile device (whatever is being tracked) consists of whatever hardware you want that can talk over SPI to a DWM1000.   In our case it was again ESP32s.    We needed to get the position data back to the device, so we had that transmitted back via WiFi.    It just sets up the DWM1000 and then transmits a pulse 5-10 times per second that includes the MAC address of the device and a sequence number.   The server software matched these up and ran quickfix on them.

# Commissioning

## Anchors 

The first step in our process was to determine where each of the anchors were.   We dropped a plumb bob and measured them directly.   However, it's possible to write an algorithm that will use the ping-pong method to figure out where all the anchors are without having to measure them.   I'd recommend it's worth it to figure that out if you're doing portable deployments.   5 minutes of running the ping-pong algorithm and you'll have accurate fixes on all the anchors.  

Once you know where the anchors are, you send pings out from each anchor, which is picked up by adjacent anchors.   From your map of where the anchors are physically, you have an expected delay between nearby anchors.  So, say that you expect the ping to be recieved at anchors 2 and 3, and the ping should be 100 counts different between them.   When the ping comes in, you find it's 110 counts different-- this indicates that the cabling between 2 and 3 is introducing some additional delay.

This is because the electrical signals of both the clock and the sync take time to transmit down their wires (about 64% of the speed of light), and this can vary if there are sharp bends in the wire, and based on the terminations done, etc.   SO, the only way to accurately adjust for this is to measure the cable delay.   

You have to work through each anchor in sequence, measuring all of the other anchors and their deviations from the expected values.   From that you can work out a map of the cable delays to each anchor and then resolve that into offsets that you apply to the timing values from that anchor.  

# Quickfix 

We hired Schuyler Erle to implement a TDOA algorithm for us, and we ended up with Quickfix: https://github.com/schuyler/quickfix

This is a lovely little library that takes a table of TDOA values, anchor positions, and rather rapidly outputs a position fix.   There is a C example here: https://github.com/schuyler/quickfix/tree/master/demo as well as an implementation of it as a python library for testing purposes.

# What's next?

I'm going to dig through my notes and files and see what I can post here that would be useful.   I think if I were doing it today we'd be using a differnet microcontroller in the anchors.   If there's interest we might put anchors back into production, they are the tricky bit.  



