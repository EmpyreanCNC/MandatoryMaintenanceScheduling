# MandatoryMaintenanceScheduling
Modification of the original grblhal Odometer plugin to implement a mandatory maintenance schedule based on axis travel

* **WARNING USE AT OWN RISK - AI CODED**

* Startup: sets up the plugin. It allocates storage and checks it is suitable. Then it loads the saved odometer record, registers the settings and installs the hooks.
* Step counting: counts step pulses per axis during cycles and homing.
* Distance accounting: when a cycle or homing move finishes, converts the counted steps to distance, adds it to the running total and saves it.
* Maintenance check: compares each axis’s total with its setpoint and raises the maintenance alarm if one is exceeded.
* Spindle timing: times how long the spindle runs and saves the total when it stops.
* Reporting command: handles the $ODOMETERS command to report, view the previous log or reset.
* Saving and reset: writes the record to internal storage, or backs it up and clears it.

<img width="1204" height="1050" alt="image" src="https://github.com/user-attachments/assets/2bb1c171-8624-46db-a4b6-9b3ac5fe169b" />

<img width="991" height="1033" alt="image" src="https://github.com/user-attachments/assets/36cdc325-4515-4253-b194-69caf7fef4f3" />
