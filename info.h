// info (DrawString) header file

#ifndef __INFO_H__
#define __INFO_H__

#include <windowsPorts/windows2linux.h>
#include <avxplugin.h>

using namespace avxsynth;

void DrawString(PVideoFrame &dst, int x, int y, const char *s, int bIsYUY2); 

#endif
