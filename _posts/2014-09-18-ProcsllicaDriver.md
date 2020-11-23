---
layout: text_entry
title: "athackst/prosilica_driver"
link: https://github.com/athackst/prosilica_driver
---
Check out the [nodelet version](https://github.com/athackst/prosilica_driver) of the prosilica driver I wrote!  

Additional features:  

* Diagnostic Status of the state of the driver
* Automatic setting of StreamBytesPerSecond when multiple cameras connect
* NEW: Software trigger mode (when hardware triggering isn’t an option) This lets you request frames from multiple cameras allowing them to have the same timestamp for easier stereo processing.
* Automatic reconnection on driver failure
