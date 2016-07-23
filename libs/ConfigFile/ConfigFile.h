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
#ifndef CONFIG_FILE_H_
#define CONFIG_FILE_H_

#include <stdio.h>
#include "Vector.h"


class ConfigFile
{
public:
    ConfigFile();
    ~ConfigFile();

    int open(const char* pFilename);
    void close();

    bool getIntVector(const char* pParamName, Vector<int16_t>* pVector);
    bool getFloatVector(const char* pParamName, Vector<float>* pVector);
    bool getFloat(const char* pParamName, float* pFloat);

protected:
    long getFileSize(FILE* pFile);
    bool skipComma();
    const char* findParameter(const char* pParamName);
    bool advancePastEqualSign();
    void skipWhitespace();
    void advanceToNextLine();
    void skipUntilNewLine();
    void skipNewLines();

    char*       m_pData;
    char*       m_pEnd;
    const char* m_pCurr;
};

#endif /* CONFIG_FILE_H_ */
