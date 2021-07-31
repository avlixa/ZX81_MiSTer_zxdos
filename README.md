# [ZX80](https://en.wikipedia.org/wiki/ZX80) / [ZX81](https://en.wikipedia.org/wiki/ZX81) for ZXDOS+ Platform

Port of MiSTer version by Sorgelic
Port of MiST version by Szombathelyi Gyorgy

### Features:
- Based on Grant Searle's [ZX80 page](http://searle.x10host.com/zx80/zx80.html)
- Selectable ZX80/ZX81
  + ZX80 currently working only in RGB mode
- 16k/32k/48k RAM packs
- 8KB with CHR$128/UDG addon (not tested)
- QS CHRS (press F1 to enable/disable the alternative chars) (not tested)
- [CHROMA81](http://www.fruitcake.plus.com/Sinclair/ZX81/Chroma/ChromaInterface.htm)
- Turbo in Slow mode: NoWait, x2, x8
- YM2149 sound chip (ZON X-81 compatible)
- Joystick types: Cursor, Sinclar, ZX81, ZXpand
- PAL/NTSC timings
- Turbo loading of .o and .p files
  + .o files not working
  + .p files: partially working
- Load alternative ROM.
- Load colorization and char files (not implemented in ZXDOS)
- Soft reset: CTRL-ALT-DEL or OSD
- Hard reset: CTRL-ALT-BCKSPC
- OSD menu: ESC or Fire 2 in joystick
- VGA/RGB: Scroll-lock

### Install
copy rom ZX8X.ROM on folder /zx81/roms: it is a concatenation of ZX81 rom (8k) + ZX80 rom (4k)
copy .zxd file and install it via BIOS

### Tape loading
Selecting an .o (ZX80) or .p (ZX81) file opens the tape. 
The LOAD command will load it as it would be on a standard tape.
Reset (CTRL-ALT-DEL or the Reset OSD option) closes the .o or .p file.

### Joystick
OSD menu allows to switch joysticks between Cursor, Sinclar, ZX81. ZXpand joystick is always enabled.
Other kinds of joystick will be disabled if ZXpand access is detected to avoid collisions.
Only direct ZXpand joystick port is supported. $1FFE call is not supported as the ZXpand ROM is not used.

### Colorization and Char files
Not implemented yet: only supported those included in .p file

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
