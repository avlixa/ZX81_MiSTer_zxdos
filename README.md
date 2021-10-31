# [ZX80](https://en.wikipedia.org/wiki/ZX80) / [ZX81](https://en.wikipedia.org/wiki/ZX81) for ZXDOS+ Platform

Port of MiSTer version by Sorgelig

Port of MiST version by Szombathelyi Gyorgy

### Features:
- Based on Grant Searle's [ZX80 page](http://searle.x10host.com/zx80/zx80.html)
- Selectable ZX80/ZX81
  + ZX80 currently working only in RGB mode, VGA depends on monitor
- 16k/32k/48k RAM packs
- 8KB with CHR$128/UDG addon
- QS CHRS (press F1 to enable/disable the alternative chars) 
- [CHROMA81](http://www.fruitcake.plus.com/Sinclair/ZX81/Chroma/ChromaInterface.htm)
- Turbo in Slow mode: NoWait, x2, x8
- YM2149 sound chip (ZON X-81 compatible)
- Joystick types: Cursor, Sinclar, ZX81, ZXpand
- PAL/NTSC timings
- Turbo loading of .o and .p files
- Load via ear conector, available only if not .o or .p loaded
- Load alternative ROM.
- Load colorization and char files (not implemented in ZUNO/ZXDOS/ZXDOS+)
- Soft reset: CTRL-ALT-DEL or OSD
- Hard reset: CTRL-ALT-BCKSPC
- OSD menu: F5 or Fire 2 in joystick
- VGA/RGB: Scroll-lock
- BIOS default video at start-up
- Load default options via CONFIG.TXT file in /ZX81/ folder
- Other options:
  + F7  button: chip AY select, jt49 / ym2149
  + F8  button: on/off PAL/NTSC carrier signal for composite video
  + F9  button: on/off mic sound to ear
  + F10 button: on/off tape in sound

### Install
Optional: copy rom ZX8X.ROM on folder /zx81/roms: it is a concatenation of ZX81 rom (8k) + ZX80 rom (4k)

Copy .zx1/zx2/.zxd file and install it via BIOS

### CONFIG.TXT file structure
```
001010002000
0123456789AB
||||||||||||------- B-Slow mode speed: 0 - Original, 1 - NoWait, 2 - x2, 3 - x8
|||||||||||-------- A-CHR$128/UDG: 00 - 128 Chars, 1 - 64 Chars, 2 - Disabled
||||||||||--------- 9-Joystick: 0 - Cursor, 1 - Sinclair, 2 - ZX81
|||||||||---------- 8-Main RAM: 0 - 16KB, 1 - 32KB, 2 - 48KB, 3 - 1KB
||||||||----------- 7-Machine model: 0:ZX81, 1:ZX80
|||||||------------ 6-Composite video carrier signal: 0: off, 1: on
||||||------------- 5-Video frequency: 0:50Hz, 1:60Hz
|||||-------------- 4-Black border: 0: off, 1: on
||||--------------- 3-Inverse video: 0: off, 1: on
|||---------------- 2-CHROMA81: 0:Disabled, 1:Enabled
||----------------- 1-QS CHRS: 0: off, 1: on
|------------------ 0-Low RAM: 0: Off, 1: 8KB
```

### Tape loading
Selecting an .o (ZX80) or .p (ZX81) file opens the tape. 
The LOAD command will load it as it would be on a standard tape.
Reset (CTRL-ALT-DEL or the Reset OSD option) closes the .o or .p file.

### Joystick
OSD menu allows to switch joysticks between Cursor, Sinclar, ZX81. ZXpand joystick is always enabled.
Other kinds of joystick will be disabled if ZXpand access is detected to avoid collisions.
Only direct ZXpand joystick port is supported. $1FFE call is not supported as the ZXpand ROM is not used.

### Colorization and Char files
Not implemented yet: only supported those included in .p/.o file

Core supports .col and .chr files loading together with main .p file if the have the same name.
For proper .col file work, CHROMA81 should be enabled before loading. For .chr file QS CHRS should be enabled before loading.

Check [this site](http://www.fruitcake.plus.com/Sinclair/ZX81/Chroma/ChromaInterface_Software.htm) for games adapted for colors.

### Options
ZX81 has many options and for most games, it's better to set:
* Main RAM: 16KB
* Low RAM: 8KB
* CHR$128: 128 chars
* QD CHRS: enabled
* CHROMA81: enabled

Some games may require specific settings - check the proper sites.
