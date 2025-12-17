# CB speech processor with DSP and ESP32

What if we add the functionality from modern CB radios such as speech compressor, tone control, echo, roger beep, all kind of filters, frequency display, direct channel or frequency entry, scanning and much more to 45 year old CB radios…

I designed a few add-ons to do that using modern technology such as the DSP (digital signal processor) ADAU1701 combined with an ESP32-S3 microcontroller.

There are 3 videos on YouTube:

The speech processor:

https://youtu.be/4eEFWMnt7ZE

The audio processor / filter:

https://youtu.be/Z7jqYiwwui8

The external PLL controller:

https://youtu.be/uo3tecpW2wY

Have a look at the videos, there are 3 repositories for this project.

Standard disclaimer: this is a hobby project, you can use it to build one or as an inspiration for your own projects. I am not responsible for what you build or what you blow up. Some tweaking might be required.

A list of the speech processor features:

Amplifier.

Noise gate.

Compressor with adjustable threshold, ratio and post gain.

12 band equalizer.

6 presets.

Echo or slapback effect.

Pitch transposer effect.

Ring modulator effect.

End of transmission with a range of built in roger beeps (67) or uploadable mp3 clips (max 999) or recorded voice clips.

Oscilloscope.

Spectrum analyzer.

Voice recorder.

Voice clip recorder.

Autokeyer.

Text to morse messages.

VOX.

Headphone amplifier for monitoring.

**Hardware**

The design files include the schematic, PCB gerber and drill files, board view with and without components. I do have Carbide Create V6 files for the housing if someone wants them.

**DSP software**

The DSP ADAU1701 is a bit different, you don't write lines of code but rather drag and drop building blocks, configure and connect them. The configuration code is then generated and stored into the module's EEPROM, upon boot the DSP reads the code from the EEPROM on it's shared I2C bus.

The chain would be to use SigmaStudio, hook up the DSP module via it's I2C bus using a special USB programming interface, create a project, create the schematic, link-compile-download to load it into the DSP RAM for testing, do life testing, then write the compilation into EEPROM. Then we want the interaction with the ESP32 so some exports are done, files are copied and a python program is used to convert all files into one .H include file for the ESP32 that contains all address references for the building blocks and also the EEPROM code.

Unless you want to make changes to the DSP design you don't need to do all the above, the .H file is included with the source code and the ESP32 will check on boot if the EEPROM version matches and if not it will upload the new code to the EEPROM. I include the entire chain instructions just in case but you can skip those steps.

I included the DSP project file in case you want to open it in SigmaStudio (free software from Analog Devices) and have a look at how it is configured, you can make changes but be aware that 1-you need to complete the entire chain above to generate a new .H file, however there is no need to connect and program because that is handled by the ESP32 so you dont need the programming adapter, SigmaStudio does not write the version number so the ESP32 will allways overwrite the EEPROM with what's in his H file. 2-the DSP has a limited number of "instructions", each block takes up a number of instructions and my design is at 90% so there is not much room for extra blocks.

**ESP32**

I use an ESP32-S3 N16R8 Devkit module. I use version 1.8.19 of the Arduino IDE since that can be installed in portable mode, one of the biggest problems with open source code is that often breaking changes occur when libraries are updated, to avoid it i work in portable mode where the entire development including libraries is contained in one folder, this way i can freeze the design environment when the design is completed. You might encounter compilation errors when you are using other versions of some libraries, check with me for version numbers.

Further instructions are in the included files.
