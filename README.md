# Dual Universe Avionics Library
### By Stochasty

Improved avionics and flight controls for Dual Universe

## About DUAL
This library is intended to improve the default flying experience for all players by providing improved flight controls and ship handling.  Major features include automatically holding current heading (so you don't need to constantly pitch down on long flights around planets), much improved altitude control when hovering, and automatic takeoff/landing.

## Getting Started
The simplest way to install DUAL is via one of the three autoconf files in the /autoconf directory.

To install, place one of the provided .conf files in the Game/data/lua/autoconf/custom directory in your DU install folder, and then load the script by right-clicking on your construct's control unit and selecting 'Advanced/Run custom autoconfigure'.  You should see a script labelled 'Dual Universe Avionics Library' in that menu (if you don't, you may need to refresh the list of configuration scripts by selecting 'Advanced/Update custom autoconf list' from that same menu).

The three available autoconfigurations are:
- DU_Avionics_Library.conf will work for the majority of constructs
- DU_Avionics_Library_hovercraft.conf is intended for hover-only ground craft, and replicates much of the behavior of the default ground craft configuration (with better handling)
- DU_Avionics_Library_large.conf is intended for large constructs which have too many hover elements to link automatically.  Currently, in order to access ground distance for hover engines, they must be explicitly linked to a control unit or programming board.  This autoconf disables the auto-linking to provide more control for constructs with slot limitations.  Users can either manually link hover engines (slot doesn't matter), or choose not to link any.  If the latter, the script will still run fine, but won't be able to do automatic ground avoidance.

For more advanced users who'd like to incorporate these controls into their own custom scripts, I've provided library functions in the /lib directory.  avionics.lua contains all of the necessary control code for flying your ship.  The provided autoconf files above give examples of how to call the library from your control unit's Lua.

## Usage
Flight controls are designed to be very similar to the default DU controls.  A couple of key differences:
- 'G' now toggles between takeoff and landing modes.  All craft start in landing mode when you first board them, so you'll need to press 'G' to take off.
- Your ship will attempt to maintain altitude when hovering, rather than following terrain.  Use 'SPACE' and 'C' to ascend and descend.
- As an exception to the above, if you drop below the minimum hover distance (default 5m), your ship will fly up to try to avoid terrain.  You can adjust the minimum hover distance in flight using 'Alt-SPACE' and 'Alt-C'
- 'Alt-1' will automatically level your ship.  Note that this can be overrided by any other control input, so it's important to let the ship finish leveling before you make further course adjustments.

## Lua Parameters
These parameters control some of the ship performance settings, and can be edited by selecting 'Edit Lua PArameters' from the control unit's menu.
- pitchAccelerationFactor: controls how quickly your ship will pitch in response to command input.
- rollAccelerationFactor: controls how quickly your ship will roll in response to command input.
- yawAccelerationFactor: controls how quickly your ship will yaw in response to command input.
- autoBrakeThreshold: maximum speed at which to engage autobrakes.  Default is zero (never autobrake); higher values will attempt to stop you ship whenever your speed falls below the threshold.  Can be useful when you want to hold a ship in place without holding brake.
- isGroundCraft: toggle between Ground and Flying modes.  Primary difference between the modes is that Ground mode sets maximum pitch/roll at 30 degrees to prevent your ship from flipping.
- minGroundDistance: This changes the default minimum hover distance.
- landingSpeed: speed at which your ship will descend when landing mode is engaged.  Be careful setting this too high when you are carrying a heavy load, otherwise your hovers may not have enough thrust to stop you from crashing.
