# SkyCMD

A Ship Computer fit for the Enterprise!.

(Includes the atlas for the Settlers myDU server.)

## Overview
This is a completely command line driven autopilot for use on remote controllers aboard large strategicly positioned vessels. What this means is that the intention is not to have granular precision in the autopilot's workings, but instead have a convenient way of moving your mobile-base while you work on other parts of the ship.

## Usage
Commands are similiar in usage to my HUD based setup, [SkyHUD](https://github.com/Skygallant/SkyHUD).
* Paste destination coords into lua chat to set your destination immediately.
* Prefix the destination with a single worded name to save that destination to the databank.
* Type the name of a destination to recall it.
* Type "list" to recall all known destinations.
* type "allstop" to commence emergency braking procedures
The ship will immediately make a "as the crow flies" route towards that destination, **this means that currently the script is only suited for space travel**.

## Warping
If your destination is withing 2SU of a planetary body, the ship will periodically check for a warp destination. If it finds one it will initiate warp. Currently, the search only includes atlas bodies, but the warp initiation portion will accept any warp point.

## Compiling
* Run du-lua build to output the .conf file for use in control units, in-game.

## Setup
* Your ship must contain a:
  * Databank

#### Super thanks to
* [DU-LuaC](https://github.com/wolfe-labs/DU-LuaC)'s interactive CLI