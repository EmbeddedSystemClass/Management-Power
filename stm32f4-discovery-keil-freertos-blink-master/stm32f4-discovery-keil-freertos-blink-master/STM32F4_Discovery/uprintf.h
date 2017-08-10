#ifndef UPRINTF_H__
#define UPRINTF_H__


static void printchar(char **str, int c);
static int uprints(char **out, const char *string, int width, int pad);
static int uprinti(char **out, int i, int b, int sg, int width, int pad, int letbase);
//static int uprint(char **out, const char *format, va_list args );
int uprintf(const char *format, ...);
int usprintf(char *out, const char *format, ...);

#endif
