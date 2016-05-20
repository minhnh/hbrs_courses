#ifndef TUTORIAL_UTILS_H_INCLUDED
#define TUTORIAL_UTILS_H_INCLUDED

#ifdef N_MAC_OSX_FRAMEWORKS
	#include <NCore/NCore.h>
#else
	#include <Core/NDefs.h>
#endif

// system headers
#ifndef N_WINDOWS
	#ifndef _GNU_SOURCE
		#define _GNU_SOURCE
	#endif
#else
	#ifndef _CRT_SECURE_NO_WARNINGS
		#define _CRT_SECURE_NO_WARNINGS
	#endif // !defined(_CRT_SECURE_NO_WARNINGS)
	#ifndef _CRT_NON_CONFORMING_SWPRINTFS
		#define _CRT_NON_CONFORMING_SWPRINTFS
	#endif // !defined(_CRT_NON_CONFORMING_SWPRINTFS)
#endif

#ifndef N_MAC_OSX_FRAMEWORKS
	#include <NCore.h>
#endif
#include <stdio.h>
#include <stdlib.h>

#ifdef N_WINDOWS

#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY
#endif

#ifdef N_UNICODE

#include <tchar.h>

#define main			_tmain
#define printf			_tprintf
#define sprintf			_stprintf
#define sprintf_s		_stprintf_s
#define strcpy			_tcscpy
#define strcmp			_tcscmp
#define _stricmp		_tcsicmp
#define strlen			_tcslen
#define fopen			_tfopen
#define fputc			_fputtc
#define fopen_s			_tfopen_s
#define atoi			_ttoi
#define atof			_wtof
#define getc			_gettc
#define getchar			_gettchar
#define sscanf			_stscanf
#define sscanf_s		_stscanf_s
#define fputs			_fputts
#define scanf			_tscanf
#define scanf_s			_tscanf_s
#define _makepath		_tmakepath
#define _makepath_s		_tmakepath_s
#define _splitpath_s	_tsplitpath_s
#define _splitpath		_tsplitpath
#define fgets			_fgetts
#define fprintf			_ftprintf
#endif

#else

#ifdef N_UNICODE
#error "Tutorial string operations do not support UNICODE"
#endif

#include <errno.h>
#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY(expr) \
	({ \
		long int _res; \
		do _res = (long int) (expr); \
		while (_res == -1L && errno == EINTR); \
		_res; \
	})
#endif

#define _stricmp strcasecmp

#endif

#ifdef N_CPP
extern "C"
{
#endif

#if defined(N_CPP) || defined(N_GCC) || defined(N_CLANG)
	#define TUTORIAL_INLINE inline
#else
	#define TUTORIAL_INLINE
#endif

static TUTORIAL_INLINE NResult PrintError(NResult result)
{
	NErrorReport(result);
	return result;
}

static NResult RetrieveErrorCode(NResult result, HNError hError)
{
	if (result == N_E_AGGREGATE && hError != NULL)
	{
		HNError hInnerError = NULL;
		NErrorGetInnerErrorEx(hError, &hInnerError);
		NErrorGetCodeEx(hInnerError, &result);
		result = RetrieveErrorCode(result, hInnerError);
		NObjectSet(NULL, &hInnerError);
	}
	return result;
}

static TUTORIAL_INLINE NResult PrintErrorWithError(NResult result, HNError hError)
{
	NErrorReportEx(result, hError);
	return RetrieveErrorCode(result, hError);
}

static TUTORIAL_INLINE NResult PrintErrorMsg(NChar * szErrorMessage, NResult result)
{
	fprintf(stderr, szErrorMessage, result);
	fprintf(stderr, N_T("\n"));
	return PrintError(result);
}

static TUTORIAL_INLINE NResult PrintErrorMsgWithError(NChar * szErrorMessage, HNError hError)
{
	NResult result;
	NErrorGetCodeEx(hError, &result);
	fprintf(stderr, szErrorMessage, result);
	fprintf(stderr, N_T("\n"));
	return PrintErrorWithError(result, hError);
}

static TUTORIAL_INLINE NResult PrintErrorMsgWithLastError(NChar * szErrorMessage, NResult result)
{
	HNError hError = NULL;

	fprintf(stderr, szErrorMessage, result);
	fprintf(stderr, N_T("\n"));

	NErrorGetLastEx(0, &hError);
	if (hError)
	{
		result = PrintErrorWithError(result, hError);
		NObjectSet(NULL, &hError);
	}
	else
	{
		PrintError(result);
	}

	return result;
}

static TUTORIAL_INLINE void OnStart(const NChar * szTitle, const NChar * szDescription, const NChar * szVersion, const NChar * szCopyright, int argc, NChar * * argv)
{
	int i;
	NResult result;

	printf(N_T("%s tutorial\n"), szTitle);
	printf(N_T("description: %s\n"), szDescription);
	printf(N_T("version: %s\n"), szVersion);
	printf(N_T("copyright: %s\n\n"), szCopyright);

	if(argc > 1)
	{
		printf(N_T("arguments:\n"));
		for(i = 1; i < argc; i++)
		{
			printf(N_T("\t%s\n"), argv[i]);
		}
		printf(N_T("\n"));
	}

	result = NCoreOnStart();
	if (NFailed(result))
	{
		PrintError(result);
	}
}

static TUTORIAL_INLINE void OnExit()
{
	NCoreOnExitEx(NFalse);
}

#ifdef N_CPP
}
#endif

#endif // TUTORIAL_UTILS_H_INCLUDED
