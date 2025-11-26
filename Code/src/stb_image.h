// Minimal PNG/JPG loader shim for macOS using ImageIO/CoreGraphics.
// Header-only, safe to include from multiple translation units.
#pragma once

#if defined(__APPLE__)
#include <CoreGraphics/CoreGraphics.h>
#include <ImageIO/ImageIO.h>
#include <CoreFoundation/CoreFoundation.h>
#include <cstdlib>

inline unsigned char *stbi_load(char const *filename, int *x, int *y, int *channels_in_file, int desired_channels){
  if(!filename) return nullptr;
  CFStringRef path = CFStringCreateWithCString(nullptr, filename, kCFStringEncodingUTF8);
  if(!path) return nullptr;
  CFURLRef url = CFURLCreateWithFileSystemPath(nullptr, path, kCFURLPOSIXPathStyle, false);
  CFRelease(path);
  if(!url) return nullptr;
  CGImageSourceRef src = CGImageSourceCreateWithURL(url, nullptr);
  CFRelease(url);
  if(!src) return nullptr;
  CGImageRef img = CGImageSourceCreateImageAtIndex(src, 0, nullptr);
  CFRelease(src);
  if(!img) return nullptr;
  size_t w = CGImageGetWidth(img);
  size_t h = CGImageGetHeight(img);
  size_t bpp = 4;
  size_t bytesPerRow = bpp*w;
  unsigned char* data = (unsigned char*)std::malloc(bytesPerRow*h);
  if(!data){ CGImageRelease(img); return nullptr; }
  CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
  CGContextRef ctx = CGBitmapContextCreate(data, w, h, 8, bytesPerRow, cs, kCGImageAlphaPremultipliedLast | kCGBitmapByteOrderDefault);
  CGColorSpaceRelease(cs);
  if(!ctx){ std::free(data); CGImageRelease(img); return nullptr; }
  CGContextDrawImage(ctx, CGRectMake(0,0,(CGFloat)w,(CGFloat)h), img);
  CGContextRelease(ctx);
  CGImageRelease(img);
  if(x) *x = (int)w;
  if(y) *y = (int)h;
  if(channels_in_file) *channels_in_file = 4;
  (void)desired_channels; // always RGBA
  return data;
}

inline void stbi_image_free(void *retval_from_stbi_load){
  std::free(retval_from_stbi_load);
}

#else
#error "This image loader shim is implemented only for macOS in this repository."
#endif


