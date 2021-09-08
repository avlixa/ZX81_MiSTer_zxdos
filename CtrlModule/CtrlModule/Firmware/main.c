#include "host.h"

#include "osd.h"
#include "keyboard.h"
#include "menu.h"
#include "ps2.h"
#include "minfat.h"
#include "spi.h"
#include "fileselector.h"

fileTYPE file;

extern int keys_p1[];
extern int keys_p2[];
extern int joy_pins;  //(SACUDLRB) => SACBRLDU
extern int currentrow;
int dipsw; //traspaso de opciones a core.
// bit 0-1 - 
// bit 2 - 
// bit 
int file_type=0; //tipo de fichero a cargar

//static char debugline[13][30];

#include "menudef.h"

/*
void ResetLoader()
{
	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DOWNLOADING;
	MegaDelay();
}
*/

/*
static int LoadKeys()
{
	int opened;
	
	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_RESET;
	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD; // Release reset but take control of the SD card
	// Anulamos para ZX81
	if((opened=FileOpen(&file,"KEYSP1     \0")))
	{
		if(FileRead(&file,sector_buffer))
		{
			
			keys_p1[0] = (int)sector_buffer[0];
			keys_p1[1] = (int)sector_buffer[1];
			keys_p1[2] = (int)sector_buffer[2];
			keys_p1[3] = (int)sector_buffer[3];
			keys_p1[4] = (int)sector_buffer[4];
		}		
	}
	
	if((opened=FileOpen(&file,"KEYSP2     \0")))
	{
		if(FileRead(&file,sector_buffer))
		{
			keys_p2[0] = (int)sector_buffer[0];
			keys_p2[1] = (int)sector_buffer[1];
			keys_p2[2] = (int)sector_buffer[2];
			keys_p2[3] = (int)sector_buffer[3];
			keys_p2[4] = (int)sector_buffer[4];
		}		
	}
	
}
*/

static int LoadROM_nomenu(const char *filename)
{
	int result=0;
	int opened;

//  dwsitch rom mask: 1111 1111 1100 0111 1111 - FFC7F 
//                .p: 0000 0000 0000 1000 0000 - 00080
//                .o: 0000 0000 0001 0000 0000 - 00100
//              .rom: 0000 0000 0011 1000 0000 - 00380

	switch(file_type) { //1 = .p, 2 = .o, 3 = .rom

    	case 2:  dipsw = (dipsw & 0xFFC7F) | 0x100; break;
		case 3: dipsw = (dipsw & 0xFFC7F) | 0x380; break;
		default: dipsw = (dipsw & 0xFFC7F) | 0x80;
	}	
	HW_HOST(REG_HOST_SW)=dipsw;

	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD|HOST_CONTROL_LED2; // Release reset but take control of the SD card

	if((opened=FileOpen(&file,filename)))
	{
		int filesize=file.size;
		unsigned int c=0;
		int bits;

		if (filesize == 5)
		{

			/*OSD_Puts(filename);
			MegaDelay();
			MegaDelay();
			MegaDelay();
			MegaDelay();
			MegaDelay();
			MegaDelay();
			Menu_Set(topmenu);*/
			HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD;
			return 1;

		}

		HW_HOST(REG_HOST_ROMSIZE) = file.size;

		//HOST_CONTROL_DOWNLOADING is "tape downloading"
		HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DOWNLOADING|HOST_CONTROL_DIVERT_SDCARD|HOST_CONTROL_LED2;
		MegaDelay();


		bits=0;
		c=filesize-1;
		while(c)
		{
			++bits;
			c>>=1;
		}
		bits-=9;

		result=1;

		while(filesize>0)
		{
			HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DOWNLOADING|HOST_CONTROL_DIVERT_SDCARD;
			OSD_ProgressBar(c,bits);
			if(FileRead(&file,sector_buffer))
			{
				int i;
				int *p=(int *)&sector_buffer;
				for(i=0;i<512;i+=4)
				//unsigned char *p=&sector_buffer;
				//for(i=0;i<512;i+=1)
				{
					unsigned int t=*p++;
					HW_HOST(REG_HOST_BOOTDATA)=t;
				}
				HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DOWNLOADING|HOST_CONTROL_DIVERT_SDCARD|HOST_CONTROL_LED2;
			}
			else
			{
				result=0;
				filesize=512;
			}
			FileNextSector(&file);
			filesize-=512;
			++c;
		}

		HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD; // Release "tape downloading" but maintin control of the SD card

	}
	HW_HOST(REG_HOST_ROMSIZE) = file.size;
	
	return(result);
}

static int LoadROM(const char *filename)
{
	int result=0;
	result=LoadROM_nomenu(filename);

	//Reset(0); //Don't reset on load tape in ZX81 core

	if(result)
		Menu_Set(tapeloaded);
	else
		Menu_Set(loadfailed);
	
	return(result);

}

int main(int argc,char **argv)
{
	int i;
	//int dipsw=0;
	dipsw=0; //traspaso de opciones a core. Default
	//dipsw
	//	bit 0: Low RAM: Off/8KB
	//	bit 1: QS CHRS: off/on
	//	bit 2: CHROMA81: Disabled/Enabled
	//  bit 3: Inverse video: Off/On
	//  bit 4: Black border: Off/On
	//  bit 5: Video frequency:50Hz,60Hz;
	//
	//	bit 10: Model: 0 - ZX81 / 1: ZX80
	//  bit 12-11: Main RAM: 00 - 16KB, 01 - 32KB, 10 - 48KB, 11 - 1KB
	//  bit 14-13: Joystick: 00 - Cursor, 01 - Sinclair, 10 - ZX81
	//  bit 16-15: CHR$128/UDG: 00 - 128 Chars, 01 - 64 Chars, 10 - Disabled
	//  bit 18-17: Slow mode speed: 00 - Original, 01 - NoWait, 10 - x2, 11 - x8

	// Put the host core in reset while we initialise...
	Reset(1); //reset and take control of keyboard with leds on
	Reset(0); //second reset for ZX81 core to be able to generate vblank used as interrupt

	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD;
	PS2Init();
	EnableInterrupts();

	OSD_Clear();
	for(i=0;i<4;++i)
	{
		PS2Wait();	// Wait for an interrupt - most likely VBlank, but could be PS/2 keyboard
		OSD_Show(1);	// Call this over a few frames to let the OSD figure out where to place the window.
	}
	//OSD_Show(0);	// 

	//Initialize SD Card
	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD|HOST_CONTROL_LED1; // Release reset but take control of the SD card
	OSD_Puts("Initializing SD card\n");

	if(!FindDrive())
		return(0);

	// Put the host core in reset while we initialise...
	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD|HOST_CONTROL_LED2; // Release reset but take control of the SD card
	OSD_Puts("Loading initial ROM...\n");

	//  dwsitch rom mask: 1111 1111 1100 0111 1111 - FFC7F 
	dipsw = (dipsw & 0xFFC7F) | 0x380;
	HW_HOST(REG_HOST_SW)=dipsw;
	
	if (LoadROM_nomenu("ZX81       ")) {
		if (LoadROM_nomenu("ROMS       ")) {
			if (!LoadROM_nomenu("ZX8X    ROM")) {	
				OSD_Puts("Error Loading ROM...\n");	
				LoadROM_nomenu("..         ");
				Reset(0);       // Reset once loaded ROM
			}
			else 
			{
				LoadROM_nomenu("..         ");
				Reset(0);       // Reset once loaded ROM
				OSD_Show(0);	// Hide OSD menu
			}
			
		}
		else{ OSD_Puts("Error Loading ROM...\n"); }
	}
	else{ OSD_Puts("Error Loading ROM...\n"); }

	HW_HOST(REG_HOST_CONTROL)=HOST_CONTROL_DIVERT_SDCARD; // Release reset but take control of the SD card

	//LoadKeys();
	FileSelector_SetLoadFunction(LoadROM);

	// Valores iniciales menu
	MENU_TOGGLE_VALUES = dipsw;
	HW_HOST(REG_HOST_SW)=dipsw;
    
	
	if(joy_pins & 0x100)
		topmenu = topmenu1; //ZXUNO menu
	else
		topmenu = topmenu2; //ZXDOS menu
	
	//topmenu = topmenu2; //ZXDOS menu
	
	Menu_Set(topmenu); //ZXUNO/ZXDOS menu

	Set_Menu_Parent(0);  // Set parent menu entry

	currentrow=3; //Load .p as default option
	Menu_Show();

	OSD_Show(0);	// Hide OSD menu
	Menu_Hide();

	while(1)
	{
		struct menu_entry *m;
		int visible;
		HandlePS2RawCodes();
		visible=Menu_Run();

		dipsw = 0;
	
		//dipsw 
		//	bit 0: Low RAM: Off/8KB
		//	bit 1: QS CHRS: off/on
		//	bit 2: CHROMA81: Disabled/Enabled
		//  bit 3: Inverse video: Off/On
		//  bit 4: Black border: Off/On
		//  bit 5: Video frequency:50Hz,60Hz;
		//
		//  bit 9-7: Downloading File type: 111 - rom, 001 - .p, 010 - .o, 
		//	bit 10: Model: 0 - ZX81 / 1: ZX80
		//  bit 12-11: Main RAM: 00 - 16KB, 01 - 32KB, 10 - 48KB, 11 - 1KB
		//  bit 14-13: Joystick: 00 - Cursor, 01 - Sinclair, 10 - ZX81
		//  bit 16-15: CHR$128/UDG: 00 - 128 Chars, 01 - 64 Chars, 10 - Disabled
		//  bit 18-17: Slow mode speed: 00 - Original, 01 - NoWait, 10 - x2, 11 - x8
			
		dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[2])<<10);	 // Take the value of the model labels cycle menu entry.
		dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[3])<<11); // Take the value of the ram labels cycle menu entry.
		dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[5])<<13); // Take the value of the joystick labels cycle menu entry.
		dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[7])<<14); // Take the value of the chrudg labels cycle menu entry.
		dipsw=dipsw|(MENU_CYCLE_VALUE(&OptionsMenu[11])<<17); // Take the value of the slowmode labels cycle menu entry.

		if(MENU_TOGGLE_VALUES&1)
			dipsw|=1;	// Add in the Low RAM bit.
		if(MENU_TOGGLE_VALUES&2)
			dipsw|=2;	// Add in the QS CHRS bit
		if(MENU_TOGGLE_VALUES&4)
			dipsw|=4;	// Add in the CHROMA81 bit
		if(MENU_TOGGLE_VALUES&8)
			dipsw|=8;	// Add in the Inverse video A bit
		if(MENU_TOGGLE_VALUES&16)
			dipsw|=16;	// Add in the Black border bit
		if(MENU_TOGGLE_VALUES&32)
			dipsw|=32;	// Add in the Video frequency bit
		HW_HOST(REG_HOST_SW)=dipsw;	// Send the new values to the hardware.

		// If the menu's visible, prevent keystrokes reaching the host core.
		HW_HOST(REG_HOST_CONTROL)=(visible ?
				HOST_CONTROL_DIVERT_KEYBOARD|HOST_CONTROL_DIVERT_SDCARD :
				HOST_CONTROL_DIVERT_SDCARD); // Maintain control of the SD card so the file selector can work.
																 // If the host needs SD card access then we would release the SD
																 // card here, and not attempt to load any further files.
	}
	return(0);
}

