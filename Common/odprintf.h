//*********************************************************
// Visual Studio debugger output printf routine,
// from http://unixwiz.net/techtips/outputdebugstring.html,
// modified to use OutputDebugStringA.
//
//*********************************************************

#ifndef DX11_ODPRINTF_H
#define DX11_ODPRINTF_H

#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <Windows.h>

void __cdecl odprintf(const char *format, ...);

#ifdef DX11_ODPRINTF_IMPLEMENTATION
#if defined(DEBUG) | defined(_DEBUG)
void __cdecl odprintf(const char *format, ...)
{
	char    buf[4096], *p = buf;
	va_list args;
	int     n;

	va_start(args, format);
	n = _vsnprintf_s(p, sizeof buf - 3, 4096, format, args); // buf-3 is room for CR/LF/NUL
	va_end(args);

	p += (n < 0) ? sizeof buf - 3 : n;

	while (p > buf  &&  isspace(p[-1]))
		*--p = '\0';

	*p++ = '\r';
	*p++ = '\n';
	*p = '\0';

	OutputDebugStringA(buf);
}
#else
void __cdecl odprintf(const char *format, ...) {

}
#endif

#endif

#endif
