/*  Copyright (C) 2016  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ConfigFile.h"


ConfigFile::ConfigFile()
{
    m_pData = NULL;
    m_pEnd = NULL;
}

ConfigFile::~ConfigFile()
{
    close();
}

int ConfigFile::open(const char* pFilename)
{
    int    result = -1;
    FILE*  pFile = NULL;
    long   fileSize = 0;
    size_t readResult = 0;

    close();

    pFile = fopen(pFilename, "r");
    if (!pFile)
        return result;

    fileSize = getFileSize(pFile);
    if (fileSize == 0)
        goto Exit;
    m_pData = (char*)malloc(fileSize + 1);
    if (!m_pData)
        goto Exit;
    m_pEnd = m_pData + fileSize;

    readResult = fread(m_pData, 1, fileSize, pFile);
    if (readResult != (size_t)fileSize)
        goto Exit;

    // Over-allocatec by one so that a NULL terminator could be inserted.
    *m_pEnd = '\0';

    result = 0;
Exit:
    if (result)
        close();
    fclose(pFile);

    return result;
}

long ConfigFile::getFileSize(FILE* pFile)
{
    int result = fseek(pFile, 0, SEEK_END);
    if (result)
        return 0;

    long fileSize = ftell(pFile);
    if (fileSize == -1 && errno != 0)
        return 0;

    result = fseek(pFile, 0, SEEK_SET);
    if (result)
        return 0;
    return fileSize;
}

void ConfigFile::close()
{
    free(m_pData);
    m_pData = NULL;
    m_pEnd = NULL;
}

bool ConfigFile::getIntVector(const char* pParamName, Vector<int16_t>* pVector)
{
    const char* pCurr = findParameter(pParamName);
    if (!pCurr)
        return false;

    pVector->x = strtol(m_pCurr, (char**)&m_pCurr, 0);
    if (!skipComma())
        return false;

    pVector->y = strtol(m_pCurr, (char**)&m_pCurr, 0);
    if (!skipComma())
        return false;

    pVector->z = strtol(m_pCurr, (char**)&m_pCurr, 0);
    skipWhitespace();
    if (*m_pCurr != '\r' && *m_pCurr != '\n' && *m_pCurr != '\0')
        return false;

    return true;
}

bool ConfigFile::getFloatVector(const char* pParamName, Vector<float>* pVector)
{
    const char* pCurr = findParameter(pParamName);
    if (!pCurr)
        return false;

    pVector->x = strtof(m_pCurr, (char**)&m_pCurr);
    if (!skipComma())
        return false;

    pVector->y = strtof(m_pCurr, (char**)&m_pCurr);
    if (!skipComma())
        return false;

    pVector->z = strtof(m_pCurr, (char**)&m_pCurr);
    skipWhitespace();
    if (*m_pCurr != '\r' && *m_pCurr != '\n' && *m_pCurr != '\0')
        return false;

    return true;
}

bool ConfigFile::skipComma()
{
    skipWhitespace();

    if (m_pCurr >= m_pEnd || *m_pCurr != ',')
        return false;
    m_pCurr++;

    skipWhitespace();
    return m_pCurr < m_pEnd;
}

const char* ConfigFile::findParameter(const char* pParamName)
{
    if (!m_pData)
        return NULL;

    int paramNameLength = strlen(pParamName);
    m_pCurr = m_pData;
    while ((m_pEnd - m_pCurr) > paramNameLength)
    {
        if (0 == strncmp(pParamName, m_pCurr, paramNameLength))
        {
            m_pCurr += paramNameLength;
            if (advancePastEqualSign())
                return m_pCurr;
        }

        advanceToNextLine();
    }

    return NULL;
}

bool ConfigFile::advancePastEqualSign()
{
    skipWhitespace();

    if (*m_pCurr != '=' || m_pCurr >= m_pEnd)
        return false;
    m_pCurr++;

    skipWhitespace();
    return m_pCurr < m_pEnd;
}

void ConfigFile::skipWhitespace()
{
    while (m_pCurr < m_pEnd && (*m_pCurr == ' ' || *m_pCurr == '\t'))
        m_pCurr++;
}

void ConfigFile::advanceToNextLine()
{
    skipUntilNewLine();
    skipNewLines();
}

void ConfigFile::skipUntilNewLine()
{
    while (m_pCurr < m_pEnd && *m_pCurr != '\r' && *m_pCurr != '\n')
        m_pCurr++;
}

void ConfigFile::skipNewLines()
{
    while (m_pCurr < m_pEnd && (*m_pCurr == '\r' || *m_pCurr == '\n'))
        m_pCurr++;
}
