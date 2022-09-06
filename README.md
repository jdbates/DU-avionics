# Dual Universe Avionics Library
### By Stochasty

Improved avionics and flight controls for Dual Universe

## About DUAL
This library is an attempt to overhaul the default DU flight controls to provide much better ship handling.

The default flight script, while functional, has poor responsiveness to control input, and almost no responsiveness to when control input is released.  This makes flying harder than necessary, especially for players using keyboard controls.  Also, the default ground engine control script is terrible - it's like trying to drive a car with no shock absorbers - any little bump or dip causes your ship to bounce around wildly.  Lastly, the default script has no correction for the curvature of the world, so on long flights around a planet your ship will literally end up pointing at the sky unless you continually apply pitch corrections.

This library is an attempt to fix all of these issues in an easy-to-use and easy-to-edit fashion.

### What it does
- Adds pitch, roll, and yaw correction when no control input is provided to attempt to main a constant heading in planet-local coordinates.
- Adds lateral and vertical velocity correction to ensure you ship flies the direction it's actually pointed - no more Tokyo drifting.  Vertical stabilizers now actually do their job.
- Improves the responsiveness of all control inputs, so that you ship turns when you tell it to turn and (more importantly) stops turning when you tell it to stop.
- Stabilizes altitude when in ground craft / hover mode.  No more instantly losing altitude when you drop off the edge of the market platform (although you may still lose some, depending on the height of the drop and your piloting talents).
- Adds automatic landing and auto-leveling functionality.

### What it does not
- DUAL is not a HUD.  It provides no additional information not already contained in the default flight widgets.
- DUAL is not an auto-pilot.  With the exception of auto-levelling and auto-landing, DUAL will not fly your ship for you.
- Overall, the flying experience with DUAL should be extremely similar to the default flying experience, but with better handling and more responsive controls.

## Getting Started
DUAL is provided both as a Lua library for those who wish to incorporate it into their own scripts and as stand-alone autoconf files for players who simply wish to use it out of the box.

The simplest way to install DUAL is to download the repository and then copy one of the three provided autoconf files into the Game/data/lua/autoconf/custom directory in your DU install folder.  To activate the script in game, you'll first need to refresh the autoconf list by right-clicking your control seat and selecting 'Advanced/Update custom autoconf list' from the menu, then loading the script by selecting 'Advanced/Run custom autoconfigure'.  You should see a script labelled 'Dual Universe Avionics Library', if you don't you either didn't refresh the list correctly or didn't place the autoconf file into the correct game folder.

There are three available preconfigured autoconf files in the /autoconf, designed for different types of constructs:
- DU_Avionics_Library.conf will work for the majority of flying constructs
- DU_Avionics_Library_hovercraft.conf is intended for hover-only ground craft, and replicates much of the behavior of the default ground craft configuration (with better handling)
- DU_Avionics_Library_large.conf is intended for large constructs which have too many hover engines / vertical boosters to be able to link them automatically.  Currently, in order to access ground distance for hover engines, they must be explicitly linked to a control unit or programming board.  This autoconf disables the auto-linking to provide more control for constructs with slot limitations.  Users can either manually link hover engines (slot doesn't matter), or choose not to link any.  The script will run fine without linked engines, however some of the altitude stabilization features will not function and those engines will revert back to the default hover behavior.  (There is also an option to use a programming board and a databank to get the ground distance, explained in more detail below.)

For more advanced users who'd like to incorporate these controls into their own custom scripts, I've provided library functions in the /lib directory.  avionics.lua contains all of the necessary control code for flying your ship.  The provided autoconf files above give examples of how to call the library from your control unit's Lua.

## Usage
Flight controls are designed to be very similar to the default DU controls.  A couple of key differences:
- 'G' now toggles between takeoff and landing modes.  All craft start in landing mode when you first board them, so you'll need to press 'G' to take off.
- Your ship will attempt to maintain altitude when hovering, rather than following terrain.  Use 'SPACE' and 'C' to ascend and descend.
- As an exception to the above, if you drop below a minimum hover distance, your ship will fly up to try to avoid terrain.  You can adjust the minimum hover distance in flight using 'Alt-SPACE' and 'Alt-C'
- 'Alt-1' will automatically level your ship.  Note that this can be overrided by any other control input, so it's important to let the ship finish leveling before you make further course adjustments.

## Lua Parameters
These parameters control some of the ship performance settings, and can be edited by selecting 'Edit Lua Parameters' from the control unit's menu.
- pitchAccelerationFactor: controls how quickly your ship will pitch in response to command input.
- rollAccelerationFactor: controls how quickly your ship will roll in response to command input.
- yawAccelerationFactor: controls how quickly your ship will yaw in response to command input.
- autoBrakeThreshold: maximum speed at which to engage autobrakes.  Default is zero (never autobrake); higher values will attempt to stop you ship whenever your speed falls below the threshold.  Can be useful when you want to hold a ship in place without holding brake.
- isGroundCraft: toggle between Ground and Flying modes.  Primary difference between the modes is that Ground mode sets maximum pitch/roll at 30 degrees to prevent your ship from flipping.
- minGroundDistance: This changes the default minimum hover distance.
- landingSpeed: speed at which your ship will descend when landing mode is engaged.  Be careful setting this too high when you are carrying a heavy load, otherwise your hovers may not have enough thrust to stop you from crashing.

## Using a Databank/Programming Board to Link Hover Engines
The library has built-in functionality to be able to read ground distance from a linked databank instead of linking the engines directly.  This can be enabled by adding one or more programming boards linked to the databank and to your hover engines.  An example .json file has been placed in the /json directory, just paste that script onto your PBs and add the necessary links (order doesn't matter, the script has automatic link detection).
