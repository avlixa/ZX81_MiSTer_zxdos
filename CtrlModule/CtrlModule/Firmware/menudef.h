//menudef.h - Definicion estructuras de menu


#include "menu.h"
#include "host.h"
#include "fileselector.h"


// Forward declaration.
void NoSelection(int row);
void Core_Credits(int row);
void Options_menu(int row);
void Keyboard_Help(int row);
void Reset(int row);
void Set_Menu_Parent(int row);
void Set_Menu_1o2(int tipo);
void Delay();
void MegaDelay();
void SetConfigIni();
int OSD_Puts(char *str);

struct menu_entry *topmenu;
char cfgini[13] = "000000000000";
extern int comp_carrier_on;

// Computer Model ZX81 / ZX80
static char *model_labels[]=
{
	"Computer Model: ZX81",
	"Computer Model: ZX80"
};
// Main RAM:16KB,32KB,48KB,1KB
static char *ram_labels[]=
{
	"Main RAM: 16KB",
	"Main RAM: 32KB",
	"Main RAM: 48KB",
	"Main RAM: 1KB"
};
// Joystick:Cursor,Sinclair,ZX81
static char *joystick_labels[]=
{
	"Joystick: Cursor",
	"Joystick: Sinclair",
	"Joystick: ZX81"
};
// CHR$128/UDG:128 Chars,64 Chars,Disabled
static char *chrudg_labels[]=
{
	"CHR$128/UDG: 128 Chars",
	"CHR$128/UDG: 64 Chars",
	"CHR$128/UDG: Disabled"
};
// Slow mode speed:Original,NoWait,x2,x8
static char *slowmode_labels[]=
{
	"Slow mode speed: Original",
	"Slow mode speed: NoWait",
	"Slow mode speed: x2",
	"Slow mode speed: x8"
};


// Options Menu for configuration of the core
struct menu_entry OptionsMenu[]=
{
	{MENU_ENTRY_CALLBACK,"= ZX81/ZX80 Configuration =",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"===========================",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CYCLE,(char *)model_labels,MENU_ACTION(2)},
	{MENU_ENTRY_CYCLE,(char *)ram_labels,MENU_ACTION(4)},
	{MENU_ENTRY_TOGGLE,"Low RAM: Off/8KB",MENU_ACTION(0)},
	{MENU_ENTRY_CYCLE,(char *)joystick_labels,MENU_ACTION(3)},
	{MENU_ENTRY_TOGGLE,"QS CHRS:Disabled/Enabled(F1)",MENU_ACTION(1)},
	{MENU_ENTRY_CYCLE,(char *)chrudg_labels,MENU_ACTION(3)},
	{MENU_ENTRY_TOGGLE,"CHROMA81: Disabled/Enabled",MENU_ACTION(2)},
	{MENU_ENTRY_TOGGLE,"Inverse video: Off/On",MENU_ACTION(3)},
	{MENU_ENTRY_TOGGLE,"Black border: Off/On",MENU_ACTION(4)},
	{MENU_ENTRY_CYCLE,(char *)slowmode_labels,MENU_ACTION(3)},
	{MENU_ENTRY_TOGGLE,"Video frequency: 50Hz/60Hz",MENU_ACTION(5)},
	{MENU_ENTRY_SUBMENU,"Go Back",MENU_ACTION(&topmenu)},
	{MENU_ENTRY_NULL,0,0}
};

// Our toplevel menu for ZX2
static struct menu_entry topmenu2[]=
{
	{MENU_ENTRY_CALLBACK,"== ZX81/ZX80 for ZXDOS ==",MENU_ACTION(&Core_Credits)},
	{MENU_ENTRY_CALLBACK,"=========================",MENU_ACTION(&Core_Credits)},
	{MENU_ENTRY_CALLBACK,"Reset",MENU_ACTION(&Reset)},
	{MENU_ENTRY_CALLBACK,"Load Tape (.p) \x10",MENU_ACTION(&FileSelectorP_Show)},
	{MENU_ENTRY_CALLBACK,"Load Tape (.o) \x10",MENU_ACTION(&FileSelectorO_Show)},
    {MENU_ENTRY_CALLBACK,"Load Rom  (.rom) \x10",MENU_ACTION(&FileSelectorROM_Show)},
	//{MENU_ENTRY_CALLBACK,"Load Rom  (.col) \x10",MENU_ACTION(&FileSelectorCOL_Show)},
	{MENU_ENTRY_CALLBACK,"Configuration options \x10",MENU_ACTION(&Options_menu)},
	{MENU_ENTRY_CALLBACK,"Keyboard Help",MENU_ACTION(&Keyboard_Help)},
	{MENU_ENTRY_CALLBACK,"Exit",MENU_ACTION(&Menu_Hide)},
	{MENU_ENTRY_NULL,0,0}
};

// Our toplevel menu for ZX1
static struct menu_entry topmenu1[]=
{
	{MENU_ENTRY_CALLBACK,"== ZX81/ZX80 for ZXUNO ==",MENU_ACTION(&Core_Credits)},
	{MENU_ENTRY_CALLBACK,"=========================",MENU_ACTION(&Core_Credits)},
	{MENU_ENTRY_CALLBACK,"Reset",MENU_ACTION(&Reset)},
	{MENU_ENTRY_CALLBACK,"Load Tape (.p) \x10",MENU_ACTION(&FileSelectorP_Show)},
	{MENU_ENTRY_CALLBACK,"Load Tape (.o) \x10",MENU_ACTION(&FileSelectorO_Show)},
    {MENU_ENTRY_CALLBACK,"Load Rom  (.rom) \x10",MENU_ACTION(&FileSelectorROM_Show)},
	//{MENU_ENTRY_CALLBACK,"Load Rom  (.col) \x10",MENU_ACTION(&FileSelectorCOL_Show)},
	{MENU_ENTRY_CALLBACK,"Configuration options \x10",MENU_ACTION(&Options_menu)},
	{MENU_ENTRY_CALLBACK,"Keyboard Help",MENU_ACTION(&Keyboard_Help)},
	{MENU_ENTRY_CALLBACK,"Exit",MENU_ACTION(&Menu_Hide)},
	{MENU_ENTRY_NULL,0,0}
};


// Keyboard Help
static struct menu_entry KeyboardHelp[]=
{
	{MENU_ENTRY_CALLBACK,"= ZX81/ZX80 Keyboard Help =",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"===========================",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"Scroll Lock: change between", MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"RGB and VGA video mode",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"Ctrl+Alt+Delete: Soft Reset" ,MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"Ctrl+Alt+Backspace: Hard reset" ,MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"F5 or joystick bt.2: to show",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"or hide the options menu.",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"WASD / cursor keys / joystick",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"to select menu option.",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"Enter / Fire to choose option.",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_SUBMENU,"OK",MENU_ACTION(&topmenu)},
	{MENU_ENTRY_NULL,0,0}
};

// Core Credits
static struct menu_entry CoreCredits[]=
{
	{MENU_ENTRY_CALLBACK,"= ZX81/ZX80 Core Credits  =",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"===========================",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"ZX81/ZX80 core for ZXUNO, ",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"ZXDOS and ZXDOS+ boards." ,MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"Original cores by:",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK," - Szombathelyi Gyorgy (Mist)",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK," - Sorgelig (Mister)",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK," - Jotego (JT49 degign)",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"Port made by: AvlixA",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_CALLBACK,"",MENU_ACTION(&NoSelection)},
	{MENU_ENTRY_SUBMENU,"OK",MENU_ACTION(&topmenu)},
	{MENU_ENTRY_NULL,0,0}
};


// An error message
static struct menu_entry loadfailed[]=
{
	{MENU_ENTRY_SUBMENU,"ROM loading failed",MENU_ACTION(loadfailed)},
	{MENU_ENTRY_SUBMENU,"OK",MENU_ACTION(&topmenu)},
	{MENU_ENTRY_NULL,0,0}
};

// An error message
static struct menu_entry tapeloaded[]=
{
	{MENU_ENTRY_SUBMENU,"Tape file Loaded.",MENU_ACTION(tapeloaded)},
	{MENU_ENTRY_SUBMENU,"Type LOAD \"\" + ENTER on ZX81",MENU_ACTION(tapeloaded)},
	{MENU_ENTRY_SUBMENU,"",MENU_ACTION(tapeloaded)},
	//{MENU_ENTRY_SUBMENU,"Then press Play and wait",MENU_ACTION(tapeloaded)},
	{MENU_ENTRY_SUBMENU,"There is no image when loading",MENU_ACTION(tapeloaded)},
	{MENU_ENTRY_SUBMENU,"",MENU_ACTION(tapeloaded)},
	{MENU_ENTRY_SUBMENU,"Continue",MENU_ACTION(&topmenu)},
	{MENU_ENTRY_NULL,0,0}
};


void NoSelection(int row)
{
}

void Core_Credits(int row)
{
	Menu_Set(CoreCredits);
}

void Options_menu(int row)
{
	Menu_Set(OptionsMenu);
}

void Keyboard_Help(int row)
{
	Menu_Set(KeyboardHelp);
}

void Reset(int row)
{
    if (row == 1) //reset with leds on
        HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_RESET|HOST_CONTROL_DIVERT_KEYBOARD|HOST_CONTROL_LED1|HOST_CONTROL_LED2;
    else
	    HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_RESET|HOST_CONTROL_DIVERT_KEYBOARD; // Reset host core
	Delay();
	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_KEYBOARD;
}

void Set_Menu_Parent(int row)
{
	tapeloaded[5].action=MENU_ACTION(Menu_Get()); // Set parent menu entry
	loadfailed[1].action=MENU_ACTION(Menu_Get()); // Set parent menu entry
	CoreCredits[12].action=MENU_ACTION(Menu_Get()); // Set parent menu entry
	KeyboardHelp[12].action=MENU_ACTION(Menu_Get()); // Set parent menu entry
	OptionsMenu[13].action=MENU_ACTION(Menu_Get()); // Set parent menu entry
}

void Set_Menu_1o2(int tipo) 
{
	if(tipo == 1)
		topmenu = topmenu1; //ZXUNO menu
	else
		topmenu = topmenu2; //ZXDOS menu
}

void Delay()
{
	int c=16384; // delay some cycles
	while(c)
	{
		c--;
	}
}

void MegaDelay()
{	int i=1;
	for (i=1;i<=576;i++)
	{
		Delay();
	}
}

//No puede estar en osd.c ya que utiliza un modo diferente de compilación
int OSD_Puts(char *str)
{
	int c;
	while((c=*str++))
		OSD_Putchar(c);
	return(1);
}

void SetConfigIni()
{
	dipsw=0; //traspaso de opciones a core. Default
	//dipsw
	//	bit 0: Low RAM: Off/8KB
	//	bit 1: QS CHRS: off/on
	//	bit 2: CHROMA81: Disabled/Enabled
	//  bit 3: Inverse video: Off/On
	//  bit 4: Black border: Off/On
	//  bit 5: Video frequency:50Hz,60Hz;
	//  bit 6: Composite video carrier signal: off/on
	//  bit 9-7: file type mask while downloading
	//	bit 10: Model: 0 - ZX81 / 1: ZX80
	//  bit 12-11: Main RAM: 00 - 16KB, 01 - 32KB, 10 - 48KB, 11 - 1KB
	//  bit 14-13: Joystick: 00 - Cursor, 01 - Sinclair, 10 - ZX81
	//  bit 16-15: CHR$128/UDG: 00 - 128 Chars, 01 - 64 Chars, 10 - Disabled
	//  bit 18-17: Slow mode speed: 00 - Original, 01 - NoWait, 10 - x2, 11 - x8
	//
	// config.txt
	// 000000000000
	// ||||||||||||------- 11-Slow mode speed: 0 - Original, 1 - NoWait, 2 - x2, 3 - x8
	// |||||||||||-------- 10-CHR$128/UDG: 00 - 128 Chars, 1 - 64 Chars, 2 - Disabled
	// ||||||||||--------- 9-Joystick: 0 - Cursor, 1 - Sinclair, 2 - ZX81
	// |||||||||---------- 8-Main RAM: 0 - 16KB, 1 - 32KB, 2 - 48KB, 3 - 1KB
	// ||||||||----------- 7-Machine model: 0:ZX81, 1:ZX80
	// |||||||------------ 6-Composite video carrier signal: 0: off, 1: on
	// ||||||------------- 5-Video frequency: 0:50Hz, 1:60Hz
	// |||||-------------- 4-Black border: 0: off, 1: on
	// ||||--------------- 3-Inverse video: 0: off, 1: on
	// |||---------------- 2-CHROMA81: 0:Disabled, 1:Enabled
	// ||----------------- 1-QS CHRS: 0: off, 1: on
	// |------------------ 0-Low RAM: 0: Off, 1: 8KB

	MENU_TOGGLE_VALUES = 0;

	if (cfgini[0]=='1') {
		MENU_TOGGLE_VALUES |= 1;
		dipsw|=1;	// Add in the Low RAM bit.
	}

	if (cfgini[1]=='1') {
		MENU_TOGGLE_VALUES |= 2;
		dipsw|=2;	// Add in the QS CHRS bit
	}

	if (cfgini[2]=='1') {
		MENU_TOGGLE_VALUES |= 4;
		dipsw|=4;	// Add in the CHROMA81 bit
	}

	if (cfgini[3]=='1') {
		MENU_TOGGLE_VALUES |= 8;
		dipsw|=8;	// Add in the Inverse video A bit
	}

	if (cfgini[4]=='1') {
		MENU_TOGGLE_VALUES |= 16;
		dipsw|=16;	// Add in the Black border bit
	}

	if (cfgini[5]=='1') {
		MENU_TOGGLE_VALUES |= 32;
		dipsw|=32;	// Add in the Video frequency bit
	}

	if (cfgini[6]=='1') {
		MENU_TOGGLE_VALUES |= 64;
		dipsw|=64;	// Composite video carrier signal on/off
		comp_carrier_on = 1;
	}

	MENU_CYCLE_VALUE(&OptionsMenu[2]) = cfgini[7] - 48; //7-Machine model: 0:ZX81, 1:ZX80
	MENU_CYCLE_VALUE(&OptionsMenu[3]) = cfgini[8] - 48; //8-Main RAM: 0 - 16KB, 1 - 32KB, 2 - 48KB, 3 - 1KB
	MENU_CYCLE_VALUE(&OptionsMenu[5]) = cfgini[9] - 48; //9-Joystick: 0 - Cursor, 1 - Sinclair, 2 - ZX81
	MENU_CYCLE_VALUE(&OptionsMenu[7]) = cfgini[10] - 48; //10-CHR$128/UDG: 00 - 128 Chars, 1 - 64 Chars, 2 - Disabled
	MENU_CYCLE_VALUE(&OptionsMenu[11]) = cfgini[11] - 48; //11-Slow mode speed: 0 - Original, 1 - NoWait, 2 - x2, 3 - x8

	dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[2])<<10);	 // Take the value of the model labels cycle menu entry.
	dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[3])<<11); // Take the value of the ram labels cycle menu entry.
	dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[5])<<13); // Take the value of the joystick labels cycle menu entry.
	dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[7])<<14); // Take the value of the chrudg labels cycle menu entry.
	dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[11])<<17); // Take the value of the slowmode labels cycle menu entry.

	HW_HOST(REG_HOST_SW)=dipsw;	// Send the new values to the hardware.

}
