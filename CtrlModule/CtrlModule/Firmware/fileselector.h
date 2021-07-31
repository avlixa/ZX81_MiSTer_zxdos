#ifndef FILESELECTOR_H
#define FILESELECTOR_H

void FileSelector_Show(int row);
void FileSelectorP_Show(int row);
void FileSelectorO_Show(int row);
void FileSelectorROM_Show(int row);
void FileSelector_SetLoadFunction(int (*func)(const char *filename));

#endif

