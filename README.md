# Dual Universe Avionics Library
### By Stochasty

Improved avionics and flight controls for Dual Universe

## About DUAL
This library is intended to improve the default flying experience for all players by providing improved flight controls and ship handling.  Major features include automatically holding current heading (so you don't need to constantly pitch down on long flights around planets), much improved altitude control when hovering, and automatic takeoff/landing.

## Getting Started
The simplest way to install DUAL is via one of the three autoconf files in the /autoconf directory.
- DU_Avionics_Library.conf will work for the majority of constructs
- DU_Avionics_Library_hovercraft.conf is intended for hover-only ground craft, and replicates much of the behavior of the default ground craft configuration (with better handling)
- DU_Avionics_Library_large.conf is intended for large constructs which have too many hover elements to link automatically.  Currently, in order to access ground distance for hover engines, they must be explicitly linked to a control unit or programming board.  This autoconf disables the auto-linking to provide more control for constructs with slot limitations.  Users can either manually link hover engines (slot doesn't matter), or choose not to link any.  If the latter, the script will still run fine, but won't be able to do automatic ground avoidance.

For more advanced users who'd like to incorporate these controls into their own custom scripts, I've provided library functions in the /lib directory.  avionics.lua contains all of the necessary control code for flying your ship.  The provided autoconf files above give examples of how to call the library from your control unit's Lua.
