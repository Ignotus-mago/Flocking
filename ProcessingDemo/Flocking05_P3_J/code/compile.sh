#!/bin/sh

# compile java files in local directory com/ignofactory/steering/*.java
# using files following the -classpath switch, which are separated with a colon (:)
# you will need to edit this for your own file locations
javac -classpath /Users/paulhz/Documents/Processing3/libraries/IgnoCodeLib/library/IgnoCodeLib.jar:/Applications/Processing.app/Contents/Java/core.jar:/Users/paulhz/Documents/Processing3/libraries/video/library/video.jar com/ignofactory/steering/*.java
# save the .class files in com/ignofactory/steering/ to a steering.jar file
jar -cf steering.jar com/ignofactory/steering/*.class
