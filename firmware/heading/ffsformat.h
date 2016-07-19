/*  Copyright 2011 Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Specifies constants and structures used within the FLASH File System.  The
   header is used by both the runtime and the tool which builds the image on
   the PC.
*/
#ifndef _FFSFORMAT_H_
#define _FFSFORMAT_H_


/* The signature to be placed in SFileSystemHeader::FileSystemSignature.
   Only the first 8 bytes are used and the NULL terminator discarded. */
#define FILE_SYSTEM_SIGNATURE "FFileSys"

/* Header stored at the beginning of the file system image. */
typedef struct _SFileSystemHeader
{
    /* Signature should be set to FILE_SYSTEM_SIGNATURE. */
    char            FileSystemSignature[8];
    /* Number of entries in this file system image. */
    unsigned int    FileCount;
    /* The SFileSystemEntry[SFileSystemHeader::FileCount] array will start here. 
       These entries are to be sorted so that a binary search can be performed
       at file open time. */
} SFileSystemHeader;

/* Information about each file added to the file system image. */
typedef struct _SFileSystemEntry
{
    /* The 2 following offsets are relative to the beginning of the file 
       image. */
    unsigned int    FilenameOffset;
    unsigned int    FileBinaryOffset;
    unsigned int    FileBinarySize;
} SFileSystemEntry;


#endif /* _FFSFORMAT_H_ */
