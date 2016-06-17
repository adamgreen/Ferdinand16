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
/* Specifies the classes used to implement the FlashFileSystem which is a
   read-only file system that exists in the internal FLASH of the mbed
   device.
*/
#include <mbed.h>
#include <assert.h>
#include "FlashFileSystem.h"
#include "ffsformat.h"


// Set FFS_TRACE to 1 to enable tracing within the FlashFileSystem class.
#define FFS_TRACE 0
#if FFS_TRACE
    #define TRACE printf
#else
    static void __trace(...)
    {
        return;
    }
    #define TRACE __trace
#endif // FFS_TRACE



/* Constructor for FlashFileSystemFileHandle which initializes to the specified
   file entry in the image.
   
   Parameters:
    pFileStart is the beginning offset of this file in FLASH memory.
    pFileEnd is the ending offset (1 bytes past last valid byte) of this file
        FLASH memory.
*/
FlashFileSystemFileHandle::FlashFileSystemFileHandle(const char* pFileStart,
                                                     const char* pFileEnd)
{
    m_pFileStart = pFileStart;
    m_pFileEnd = pFileEnd;
    m_pCurr = pFileStart;
}


// Constructs a blank FlashFileSystemFileHandle object.
FlashFileSystemFileHandle::FlashFileSystemFileHandle()
{
    FlashFileSystemFileHandle(NULL, NULL);
}
    

/* Write the contents of a buffer to the file

   Parameters:
    pBuffer is the buffer from which to write.
    Length is the number of characters to write.

   Returns
    The number of characters written (possibly 0) on success, -1 on error.
 */
ssize_t FlashFileSystemFileHandle::write(const void* pBuffer, size_t Length)
{
    // This file system doesn't support writing.
    return -1;
}


/* Close the file

   Returns
    Zero on success, -1 on error.
*/
int FlashFileSystemFileHandle::close()
{
    m_pFileStart = NULL;
    m_pFileEnd = NULL;
    m_pCurr = NULL;
    
    return 0;
}

/*  Reads the contents of the file into a buffer.

   Parameters
    pBuffer is the buffer into which the read should occur.
    Length is the number of characters to read into pBuffer.

   Returns
    The number of characters read (zero at end of file) on success, -1 on error.
*/
ssize_t FlashFileSystemFileHandle::read(void* pBuffer, size_t Length)
{
    unsigned int    BytesLeft;

    // Don't read more bytes than what are left in the file.
    BytesLeft = m_pFileEnd - m_pCurr;
    if (Length > BytesLeft)
    {
        Length = BytesLeft;
    }
    
    // Copy the bytes from FLASH into the caller provided buffer.
    memcpy(pBuffer, m_pCurr, Length);
    
    // Update the file pointer.
    m_pCurr += Length;
    
    return Length;
}


/* Check if the handle is for a interactive terminal device .

   Parameters
    None
   Returns
    1 if it is a terminal, 0 otherwise
*/
int FlashFileSystemFileHandle::isatty()
{
    return 0;
}


/* Move the file position to a given offset from a given location.
 
   Parameters
    offset is the offset from whence to move to.
    whence - SEEK_SET for the start of the file, 
             SEEK_CUR for the current file position, or 
             SEEK_END for the end of the file.
  
   Returns
    New file position on success, -1 on failure or unsupported
*/
off_t FlashFileSystemFileHandle::lseek(off_t offset, int whence)
{
    switch(whence)
    {
    case SEEK_SET:
        m_pCurr = m_pFileStart + offset;
        break;
    case SEEK_CUR:
        m_pCurr += offset;
        break;
    case SEEK_END:
        m_pCurr = (m_pFileEnd - 1) + offset;
        break;
    default:
        TRACE("FlashFileSytem: Received unknown origin code (%d) for seek.\r\n", whence);
        return -1;
    }
    
    return (m_pCurr - m_pFileStart);
}


/* Flush any buffers associated with the FileHandle, ensuring it
   is up to date on disk.  Since the Flash file system is read-only, there
   is nothing to do here.
  
   Returns
    0 on success or un-needed, -1 on error
*/
int FlashFileSystemFileHandle::fsync()
{
    return 0;
}


/* Returns the length of the file.

   Parameters:
    None
   Returns:
    Length of file.
*/
off_t FlashFileSystemFileHandle::flen()
{
    return (m_pFileEnd - m_pFileStart);
}



/* Construct and initialize a directory handle enumeration object.

   Parameters:
    pFLASHBase points to the beginning of the file system in FLASH memory.
    pFirstFileEntry is a pointer to the first entry found in this directory.
    DirectoryNameLength is the length of the directory name for which this
        handle is being used to enumerate.
        
   Returns:
    Nothing.
*/
FlashFileSystemDirHandle::FlashFileSystemDirHandle(const char*              pFLASHBase,
                                                   const _SFileSystemEntry* pFirstFileEntry,
                                                   unsigned int             FileEntriesLeft,
                                                   unsigned int             DirectoryNameLength)
{
    m_pFLASHBase = pFLASHBase;
    m_pFirstFileEntry = pFirstFileEntry;
    m_pCurrentFileEntry = pFirstFileEntry;
    m_FileEntriesLeft = FileEntriesLeft;
    m_DirectoryNameLength = DirectoryNameLength;
    m_DirectoryEntry.d_name[0] = '\0';
}


// Used to construct a closed directory handle.
FlashFileSystemDirHandle::FlashFileSystemDirHandle()
{
    FlashFileSystemDirHandle(NULL, NULL, 0, 0);
}

                             
/* Closes the directory enumeration object.
  
   Parameters:
    None.
    
   Returns:
    0 on success, or -1 on error.
*/
int FlashFileSystemDirHandle::closedir()
{
    m_pFLASHBase = NULL;
    m_pFirstFileEntry = NULL;
    m_pCurrentFileEntry = NULL;
    m_FileEntriesLeft = 0;
    m_DirectoryNameLength = 0;
    m_DirectoryEntry.d_name[0] = '\0';
    
    return 0;
}


/* Return the directory entry at the current position, and
   advances the position to the next entry.
  
   Parameters:
    None.
    
   Returns:
    A pointer to a dirent structure representing the
    directory entry at the current position, or NULL on reaching
    end of directory or error.
*/
struct dirent* FlashFileSystemDirHandle::readdir()
{
    const char*  pPrevEntryName;
    const char*  pCurrentEntryName;
    char*        pSlash;
    size_t       PrefixLength;
    unsigned int FileEntriesUsed;
    unsigned int FileEntriesLeft;  
    
    // Just return now if we have already finished enumerating the entries in
    // the directory.
    if (!m_pCurrentFileEntry)
    {
        m_DirectoryEntry.d_name[0] = '\0';
        return NULL;
    }
    
    // Calculate the number of valid entries are left in the file entry array.
    FileEntriesUsed = m_pCurrentFileEntry - m_pFirstFileEntry;
    FileEntriesLeft = m_FileEntriesLeft - FileEntriesUsed;
    
    // Fill in the directory entry structure for the current entry.
    pPrevEntryName = m_pFLASHBase + 
                     m_pCurrentFileEntry->FilenameOffset;
    strncpy(m_DirectoryEntry.d_name, 
            pPrevEntryName + m_DirectoryNameLength, 
            sizeof(m_DirectoryEntry.d_name));
    m_DirectoryEntry.d_name[sizeof(m_DirectoryEntry.d_name) - 1] = '\0';
    
    // If the entry to be returned contains a slash then this is a directory
    // entry.
    pSlash = strchr(m_DirectoryEntry.d_name, '/');
    if (pSlash)
    {
        // I am truncating everything after the slash but leaving the
        // slash so that I can tell it is a directory and not a file.
        pSlash[1] = '\0';
    }
    
    // Skip entries that have the same prefix as the current entry.  This
    // will skip the files in the same sub-tree.
    PrefixLength = strlen(m_DirectoryEntry.d_name) + m_DirectoryNameLength;
    do
    {
        m_pCurrentFileEntry++;
        FileEntriesLeft--;
        pCurrentEntryName = m_pFLASHBase + m_pCurrentFileEntry->FilenameOffset;
    } while (FileEntriesLeft && 
             0 == strncmp(pPrevEntryName, pCurrentEntryName, PrefixLength));
    
    // If we have walked past the end of all file entries in the file system or
    // the prefix no longer matches this directory, then there are no more files
    // for this directory enumeration.
    if (0 == FileEntriesLeft || 
        0 != strncmp(pPrevEntryName, pCurrentEntryName, m_DirectoryNameLength))
    {
        m_pCurrentFileEntry = NULL;
    }
    
    // Return a pointer to the directory entry structure that was previously
    // setup.
    return &m_DirectoryEntry;
}


//Resets the position to the beginning of the directory.
void FlashFileSystemDirHandle::rewinddir()
{
    m_pCurrentFileEntry = m_pFirstFileEntry;
}


/* Returns the current position of the DirHandle.
 
   Parameters:
    None.
    
   Returns:
    The current position, or -1 on error.
*/
off_t FlashFileSystemDirHandle::telldir()
{
    return (off_t)m_pCurrentFileEntry;
}


/* Sets the position of the DirHandle.
 
   Parameters:
    Location is the location to seek to. Must be a value returned
        by telldir.
        
   Returns;
    Nothing.
*/
void FlashFileSystemDirHandle::seekdir(off_t Location)
{
    SFileSystemEntry*   pLocation = (SFileSystemEntry*)Location;
    
    assert ( NULL != pLocation &&
             pLocation > m_pFirstFileEntry &&
             (pLocation - m_pFirstFileEntry) < m_FileEntriesLeft );
    
    m_pCurrentFileEntry = pLocation;
}



// Structure used to hold context about current filename search.
struct SSearchContext
{
    // Name of file to be found in file system image.
    const char* pKey;
    // Base pointer for the file system image.
    const char* pFLASHBase;
};


/* Internal routine used as callback for bsearch() when searching for a
   requested filename in the FLASH file system image.
   
   pvKey is a pointer to the SSearchContext object for this search.
   pvEntry is a pointer to the current file system entry being checked by
    bsearch().
    
   Returns <0 if filename to find is lower in sort order than current entry.
            0 if filename to find is the same as the current entry.
           >0 if filename to find is higher in sort order than current entry.
*/
static int _CompareKeyToFileEntry(const void* pvKey, const void* pvEntry)
{
    const SSearchContext*       pContext = (const SSearchContext*)pvKey;
    const char*                 pKey = pContext->pKey;
    const SFileSystemEntry*     pEntry = (const SFileSystemEntry*)pvEntry;
    const char*                 pEntryName = pContext->pFLASHBase + pEntry->FilenameOffset;

    return strcmp(pKey, pEntryName);
}


/* Constructor for FlashFileSystem

   Parameters:
    pName is the root name to be used for this file system in fopen()
        pathnames.
    pFlashDrive (optional) is a pointer to the read-only file system (const char array).
        When pFlashDrive is not specified, it is up to the user to append the
        read-only file system file to the compiled binary.
    FlashSize (optional) is the size of the FLASH (KB) on the device to
        search through for the file system signature (default = 512).
*/
FlashFileSystem::FlashFileSystem(const char* pName, const uint8_t *pFlashDrive, const uint32_t FlashSize) : FileSystemLike(pName)
{
    static const char   FileSystemSignature[] = FILE_SYSTEM_SIGNATURE;
    SFileSystemHeader*  pHeader = NULL;
    char*               pCurr = (char*)(FlashSize * 1024) - sizeof(pHeader->FileSystemSignature);
    
    // Initialize the members
    m_pFLASHBase = NULL;
    m_FileCount = 0;
    m_pFileEntries = NULL;
    
    // Scan backwards through 512k FLASH looking for the file system signature
    // NOTE: The file system image should be located after this code itself
    //       so stop the search.
    if(pFlashDrive == NULL)
    {
        while (pCurr > FileSystemSignature)
        {
            if (0 == memcmp(pCurr, FileSystemSignature, sizeof(pHeader->FileSystemSignature)))
            {
                break;
            }
            pCurr--;
        }
        if (pCurr <= FileSystemSignature)
        {
            TRACE("FlashFileSystem: Failed to find file system image in ROM.\n");
            return;
        }
    }
    else
    {
        pCurr = (char *)pFlashDrive;
    }
    if (((unsigned int)pCurr & 0x3) != 0)
    {
        TRACE("FlashFileSystem: File system image at address %08X isn't 4-byte aligned.\n", pCurr);
        return;
    }
    
    // Record the location of the file system image in the member fields.
    m_pFLASHBase = pCurr;
    pHeader = (SFileSystemHeader*)m_pFLASHBase;
    m_FileCount = pHeader->FileCount;
    m_pFileEntries = (SFileSystemEntry*)(m_pFLASHBase + sizeof(*pHeader));
}


/* Opens specified file in FLASH file system when an appropriate call to
   fopen() is made.
   
   pFilename is the name of the file to be opened within the file system.
   Flags specify flags to determine open mode of file.  This file system only
    support O_RDONLY.
   
   Returns NULL if an error was encountered or a pointer to a FileHandle object
    representing the requrested file otherwise.
*/
FileHandle* FlashFileSystem::open(const char* pFilename, int Flags)
{
    const SFileSystemEntry*     pEntry = NULL;
    FlashFileSystemFileHandle*  pFileHandle = NULL;
    SSearchContext              SearchContext;
    
    TRACE("FlashFileSystem: Attempt to open file /FLASH/%s with flags:%x\r\n", pFilename, Flags);
    
    // Can't find the file if file system hasn't been mounted.
    if (!IsMounted())
    {
        return NULL;
    }

    // Can only open files in FLASH for read.
    if (O_RDONLY != Flags)
    {
        TRACE("FlashFileSystem: Can only open files for reading.\r\n");
    }
    
    // Attempt to find the specified file in the file system image.
    SearchContext.pKey = pFilename;
    SearchContext.pFLASHBase = m_pFLASHBase;
    pEntry = (const SFileSystemEntry*) bsearch(&SearchContext,
                                               m_pFileEntries,
                                               m_FileCount, 
                                               sizeof(*m_pFileEntries), 
                                               _CompareKeyToFileEntry);
    if(!pEntry)
    {
        // Create failure response.
        TRACE("FlashFileSystem: Failed to find '%s' in file system image.\n", pFilename);
        return NULL;
    }

    // Attempt to find a free file handle.
    pFileHandle = FindFreeFileHandle();
    if (!pFileHandle)
    {
        TRACE("FlashFileSystem: File handle table is full.\n");
        return NULL;
    }
    
    // Initialize the file handle and return it to caller.
    pFileHandle->SetEntry(m_pFLASHBase + pEntry->FileBinaryOffset,
                          m_pFLASHBase + pEntry->FileBinaryOffset + pEntry->FileBinarySize);
    return pFileHandle;
}

DirHandle*  FlashFileSystem::opendir(const char *pDirectoryName)
{
    const SFileSystemEntry* pEntry = m_pFileEntries;
    unsigned int            DirectoryNameLength;
    unsigned int            i;
    
    assert ( pDirectoryName);
    
    // Removing leading slash since the file system image doesn't contain
    // leading slashes.
    if ('/' == pDirectoryName[0])
    {
        pDirectoryName++;
    }
    
    // Make sure that the directory name length would include the trailing
    // slash.
    DirectoryNameLength = strlen(pDirectoryName);
    if (0 != DirectoryNameLength && '/' != pDirectoryName[DirectoryNameLength-1])
    {
        // Add the implicit slash to this count.
        DirectoryNameLength++;
    }
    
    // Search through the file entries from the beginning to find the first
    // entry which has pDirectoryName/ as the prefix.
    for (i = 0 ; i < m_FileCount ; i++)
    {
        const char* pEntryFilename = pEntry->FilenameOffset + m_pFLASHBase;
        
        if (0 == DirectoryNameLength ||
            (pEntryFilename == strstr(pEntryFilename, pDirectoryName) &&
             '/' == pEntryFilename[DirectoryNameLength-1]) )
        {
            // Found the beginning of the list of files/folders for the
            // requested directory so return it to the caller.
            FlashFileSystemDirHandle* pDirHandle = FindFreeDirHandle();
            if (!pDirHandle)
            {
                TRACE("FlashFileSystem: Dir handle table is full.\n");
                return NULL;
            }
            
            pDirHandle->SetEntry(m_pFLASHBase,
                                 pEntry,
                                 m_FileCount - (pEntry - m_pFileEntries),
                                 DirectoryNameLength);
            
            return pDirHandle;
        }
        
        // Advance to the next file entry
        pEntry++;
    }
    
    // Get here when the requested directory wasn't found.
    TRACE("FlashFileSystem: Failed to find '%s' directory in file system image.\n", 
          pDirectoryName);
    return NULL;
}


/* Protected method which attempts to find a free file handle in the object's
   file handle table.
   
   Parameters:
    None
    
   Returns:
    Pointer to first free file handle entry or NULL if the table is full.
*/
FlashFileSystemFileHandle* FlashFileSystem::FindFreeFileHandle()
{
    size_t  i;
    
    // Iterate through the file handle array, looking for a close file handle.
    for (i = 0 ; i < sizeof(m_FileHandles)/sizeof(m_FileHandles[0]) ; i++)
    {
        if (m_FileHandles[i].IsClosed())
        {
            return &(m_FileHandles[i]);
        }
    }
    
    // If we get here, then no free entries were found.
    return NULL;
}


/* Protected method which attempts to find a free dir handle in the object's
   directory handle table.
   
   Parameters:
    None
    
   Returns:
    Pointer to first free directory handle entry or NULL if the table is full.
*/
FlashFileSystemDirHandle* FlashFileSystem::FindFreeDirHandle()
{
    size_t  i;
    
    // Iterate through the direcotry handle array, looking for a closed 
    // directory handle.
    for (i = 0 ; i < sizeof(m_DirHandles)/sizeof(m_DirHandles[0]) ; i++)
    {
        if (m_DirHandles[i].IsClosed())
        {
            return &(m_DirHandles[i]);
        }
    }
    
    // If we get here, then no free entries were found.
    return NULL;
}
