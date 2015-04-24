# Introduction #

Yertle is a robot I've been working on, it resembles a Turtlebot.  The hardware is pretty simple, these are the only parts:
  * A Pentium laptop onboard.
  * A Kinect for localization and as a camera.
  * An Arduino Uno for interfacing with the motors and rotary encoders.
  * A 12V, 9000mAh lead-acid battery.
  * Two motors for differential drive
  * Two rotary encoders.
  * A DC-DC converter (step-up and step-down) to regulate the battery output to 12V for the Kinect.

# Details #

This is the finished (for now) robot:

![https://lh5.googleusercontent.com/-TIOdgsRhGWA/USA7rPOW4WI/AAAAAAAADkY/zyidPpfPMJQ/s811/2013-02-16_21-08-37_598.jpg](https://lh5.googleusercontent.com/-TIOdgsRhGWA/USA7rPOW4WI/AAAAAAAADkY/zyidPpfPMJQ/s811/2013-02-16_21-08-37_598.jpg)


The PC is connected to a Arduino Uno, which has a SeedStudio motor control shield:

![https://lh5.googleusercontent.com/-VKvV931ShFE/USA7J80UeBI/AAAAAAAADjo/0SoF2qWmOXo/s1318/2013-02-16_21-06-04_966.jpg](https://lh5.googleusercontent.com/-VKvV931ShFE/USA7J80UeBI/AAAAAAAADjo/0SoF2qWmOXo/s1318/2013-02-16_21-06-04_966.jpg)


The 12V for the Kinect comes from a DC-DC converter.  A 12V lead-acid battery can be 13V when fully charged, but closer to 12V when it's not, so you need a voltage regulator to suppy the kinect.  Most regulators can only step down with a ~1.5V cutoff.  This means your battery needs to be >13.5V to supply 12V.  This handy converter can step down or up.  (It was a little pricey at about $20)

![https://lh6.googleusercontent.com/-PeMWrhfqszI/USA7FTrmH-I/AAAAAAAADjQ/f4IgBNjKHOM/s743/2013-02-16_21-05-47_311.jpg](https://lh6.googleusercontent.com/-PeMWrhfqszI/USA7FTrmH-I/AAAAAAAADjQ/f4IgBNjKHOM/s743/2013-02-16_21-05-47_311.jpg)