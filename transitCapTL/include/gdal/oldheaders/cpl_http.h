/******************************************************************************
 * $Id: cpl_http.h 17130 2009-05-26 19:10:30Z rouault $
 *
 * Project:  Common Portability Library
 * Purpose:  Function wrapper for libcurl HTTP access.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2006, Frank Warmerdam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#ifndef CPL_HTTP_H_INCLUDED
#define CPL_HTTP_H_INCLUDED

#include "cpl_conv.h"
#include "cpl_string.h"
#include "cpl_vsi.h"

/**
 * \file cpl_http.h
 *
 * Interface for downloading HTTP, FTP documents
 */

CPL_C_START

/*! Describe a part of a multipart message */
typedef struct {
    /*! NULL terminated array of headers */ char **papszHeaders;
    
    /*! Buffer with data of the part     */ GByte *pabyData;
    /*! Buffer length                    */ int    nDataLen;
} CPLMimePart;

/*! Describe the result of a CPLHTTPFetch() call */
typedef struct {
    /*! HTTP status code : 200=success, value < 0 if request failed */ int     nStatus;
    /*! Content-Type of the response */                                char    *pszContentType;
    /*! Error message from curl, or NULL */                            char    *pszErrBuf;

    /*! Length of the pabyData buffer */                               int     nDataLen;
                                                                       int     nDataAlloc;
    /*! Buffer with downloaded data */                                 GByte   *pabyData;

    /*! Number of parts in a multipart message */                      int     nMimePartCount;
    /*! Array of parts (resolved by CPLHTTPParseMultipartMime()) */    CPLMimePart *pasMimePart;
} CPLHTTPResult;

int CPL_DLL   CPLHTTPEnabled( void );
CPLHTTPResult CPL_DLL *CPLHTTPFetch( const char *pszURL, char **papszOptions);
void CPL_DLL  CPLHTTPCleanup( void );
void CPL_DLL  CPLHTTPDestroyResult( CPLHTTPResult *psResult );
int  CPL_DLL  CPLHTTPParseMultipartMime( CPLHTTPResult *psResult );

CPL_C_END

#endif /* ndef CPL_HTTP_H_INCLUDED */
