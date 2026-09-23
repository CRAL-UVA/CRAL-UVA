# Maintenance

> This page is a starting point — expand it as more maintenance procedures are established.

## Battery Care

- Connect **only the red and black connector**, never the white connector, when plugging the battery into the Jackal — a connector has been fried doing this before.
- Pull firmly but carefully when unplugging the battery; avoid yanking on the connector.
- To avoid sparking when reconnecting the charger: plug the charger into the wall, unplug it, wait for the charger's light to turn off, plug the battery into the charger, then plug the charger back into the wall.
- Store batteries appropriately when not in use.

## Electronics Tray

- The internal electronics tray (motherboard + GPU) is not meant to be accessed often. Handle it with care — e.g. when connecting a monitor for direct login (see [User Manual](User_Manual.md)).

## Pre-Run Checklist

- Turn off the drive motor (the "M" button) unless autonomy/teleop testing explicitly requires it — reduces the risk of an accidental crash.
- Confirm WiFi connectivity (NETGEAR52) before relying on wireless SSH in the field.
- If sensors or odometry look wrong after boot, try restarting the relevant Clearpath service before further debugging — see [Troubleshooting](Troubleshooting.md).
