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
For the most part, the flight controls should be familiar to anyone whose already flown with the default flight script.  The only exception to that is that the 'G' key now toggles between landing and flight modes.  When you board your construct, it defaults to starting in 'landed' mode, so you will need to press the 'G' key to be able to takeoff.

Additionally, Option-1 will now automatically level your ship during flight (however, any other control inputs will override this, so make sure to let it finish levelling before you make further adjustments).

Lastly, 'Alt-C' and 'Alt-Space' now control your minimum hover distance.  This can be useful if you need to adjust your minimum altitude settings to help with terrain avoidance.