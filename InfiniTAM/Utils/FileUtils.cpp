// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "FileUtils.h"

#include <stdio.h>
#include <fstream>

#ifdef USE_LIBPNG
#include <png.h>
#endif
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

static const char *pgm_ascii_id = "P2";
static const char *ppm_ascii_id = "P3";
static const char *pgm_id = "P5";
static const char *ppm_id = "P6";

typedef enum { MONO_8u, RGB_8u, MONO_16u, MONO_16s, RGBA_8u, FORMAT_UNKNOWN = -1 } FormatType;

struct PNGReaderData {
#ifdef USE_LIBPNG
	png_structp png_ptr;
	png_infop info_ptr;

	PNGReaderData(void)
	{ png_ptr = NULL; info_ptr = NULL; }
	~PNGReaderData(void)
	{ 
		if (info_ptr != NULL) png_destroy_info_struct(png_ptr, &info_ptr);
		if (png_ptr != NULL) png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
	}
#endif
};

static FormatType png_readheader(FILE *fp, int & width, int & height, PNGReaderData & internal)
{
	FormatType type = FORMAT_UNKNOWN;

#ifdef USE_LIBPNG
	png_byte color_type;
	png_byte bit_depth;

	unsigned char header[8];    // 8 is the maximum size that can be checked

	fread(header, 1, 8, fp);
	if (png_sig_cmp(header, 0, 8)) {
		//"not a PNG file"
		return type;
	}

	/* initialize stuff */
	internal.png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

	if (!internal.png_ptr) {
		//"png_create_read_struct failed"
		return type;
	}

	internal.info_ptr = png_create_info_struct(internal.png_ptr);
	if (!internal.info_ptr) {
		//"png_create_info_struct failed"
		return type;
	}

	if (setjmp(png_jmpbuf(internal.png_ptr))) {
		//"setjmp failed"
		return type;
	}

	png_init_io(internal.png_ptr, fp);
	png_set_sig_bytes(internal.png_ptr, 8);

	png_read_info(internal.png_ptr, internal.info_ptr);

	width = png_get_image_width(internal.png_ptr, internal.info_ptr);
	height = png_get_image_height(internal.png_ptr, internal.info_ptr);
	color_type = png_get_color_type(internal.png_ptr, internal.info_ptr);
	bit_depth = png_get_bit_depth(internal.png_ptr, internal.info_ptr);

	if (color_type == PNG_COLOR_TYPE_GRAY) {
		if (bit_depth == 8) type = MONO_8u;
		else if (bit_depth == 16) type = MONO_16u;
		// bit depths 1, 2 and 4 are not accepted
	} else if (color_type == PNG_COLOR_TYPE_RGB) {
		if (bit_depth == 8) type = RGB_8u;
		// bit depth 16 is not accepted
	} else if (color_type == PNG_COLOR_TYPE_RGBA) {
		if (bit_depth == 8) type = RGBA_8u;
		// bit depth 16 is not accepted
	}
	// other color types are not accepted
#endif

	return type;
}

static bool png_readdata(FILE *f, int xsize, int ysize, PNGReaderData & internal, void *data_ext)
{
#ifdef USE_LIBPNG
	if (setjmp(png_jmpbuf(internal.png_ptr))) return false;

	png_read_update_info(internal.png_ptr, internal.info_ptr);

	/* read file */
	if (setjmp(png_jmpbuf(internal.png_ptr))) return false;

	int bytesPerRow = png_get_rowbytes(internal.png_ptr, internal.info_ptr);

	png_byte *data = (png_byte*)data_ext;
	png_bytep *row_pointers = new png_bytep[ysize];
	for (int y=0; y<ysize; y++) row_pointers[y] = &(data[bytesPerRow*y]);

	png_read_image(internal.png_ptr, row_pointers);
	png_read_end(internal.png_ptr, NULL);

	delete[] row_pointers;

	return true;
#else
	return false;
#endif
}

static FormatType pnm_readheader(FILE *f, int *xsize, int *ysize, bool *binary)
{
	char tmp[1024];
	FormatType type = FORMAT_UNKNOWN;
	int xs = 0, ys = 0, max_i = 0;
	bool isBinary = true;

	/* read identifier */
	if (fscanf(f, "%[^ \n\t]", tmp) != 1) return type;
	if (!strcmp(tmp, pgm_id)) type = MONO_8u;
	else if (!strcmp(tmp, pgm_ascii_id)) { type = MONO_8u; isBinary = false; }
	else if (!strcmp(tmp, ppm_id)) type = RGB_8u;
	else if (!strcmp(tmp, ppm_ascii_id)) { type = RGB_8u; isBinary = false; }
	else return type;

	/* read size */
	if (!fscanf(f, "%i", &xs)) return FORMAT_UNKNOWN;
	if (!fscanf(f, "%i", &ys)) return FORMAT_UNKNOWN;

	if (!fscanf(f, "%i", &max_i)) return FORMAT_UNKNOWN;
	if (max_i < 0) return FORMAT_UNKNOWN;
	else if (max_i <= (1 << 8)) {}
	else if ((max_i <= (1 << 15)) && (type == MONO_8u)) type = MONO_16s;
	else if ((max_i <= (1 << 16)) && (type == MONO_8u)) type = MONO_16u;
	else return FORMAT_UNKNOWN;
	fgetc(f);

	if (xsize) *xsize = xs;
	if (ysize) *ysize = ys;
	if (binary) *binary = isBinary;

	return type;
}

template<class T>
static bool pnm_readdata_ascii_helper(FILE *f, int xsize, int ysize, int channels, T *data)
{
	for (int y = 0; y < ysize; ++y) for (int x = 0; x < xsize; ++x) for (int c = 0; c < channels; ++c) {
		int v;
		if (!fscanf(f, "%i", &v)) return false;
		*data++ = v;
	}
	return true;
}

static bool pnm_readdata_ascii(FILE *f, int xsize, int ysize, FormatType type, void *data)
{
	int channels = 0;
	switch (type)
	{
	case MONO_8u:
		channels = 1;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (unsigned char*)data);
	case RGB_8u:
		channels = 3;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (unsigned char*)data);
	case MONO_16s:
		channels = 1;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (short*)data);
	case MONO_16u:
		channels = 1;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (unsigned short*)data);
	case FORMAT_UNKNOWN:
	default: break;
	}
	return false;
}

static bool pnm_readdata_binary(FILE *f, int xsize, int ysize, FormatType type, void *data)
{
	int channels = 0;
	int bytesPerSample = 0;
	switch (type)
	{
	case MONO_8u: bytesPerSample = sizeof(unsigned char); channels = 1; break;
	case RGB_8u: bytesPerSample = sizeof(unsigned char); channels = 3; break;
	case MONO_16s: bytesPerSample = sizeof(short); channels = 1; break;
	case MONO_16u: bytesPerSample = sizeof(unsigned short); channels = 1; break;
	case FORMAT_UNKNOWN:
	default: break;
	}
	if (bytesPerSample == 0) return false;

	size_t tmp = fread(data, bytesPerSample, xsize*ysize*channels, f);
	if (tmp != (size_t)xsize*ysize*channels) return false;
	return (data != NULL);
}

static bool pnm_writeheader(FILE *f, int xsize, int ysize, FormatType type)
{
	const char *pnmid = NULL;
	int max = 0;
	switch (type) {
	case MONO_8u: pnmid = pgm_id; max = 256; break;
	case RGB_8u: pnmid = ppm_id; max = 255; break;
	case MONO_16s: pnmid = pgm_id; max = 32767; break;
	case MONO_16u: pnmid = pgm_id; max = 65535; break;
	case FORMAT_UNKNOWN:
	default: return false;
	}
	if (pnmid == NULL) return false;

	fprintf(f, "%s\n", pnmid);
	fprintf(f, "%i %i\n", xsize, ysize);
	fprintf(f, "%i\n", max);

	return true;
}

static bool pnm_writedata(FILE *f, int xsize, int ysize, FormatType type, const void *data)
{
	int channels = 0;
	int bytesPerSample = 0;
	switch (type)
	{
	case MONO_8u: bytesPerSample = sizeof(unsigned char); channels = 1; break;
	case RGB_8u: bytesPerSample = sizeof(unsigned char); channels = 3; break;
	case MONO_16s: bytesPerSample = sizeof(short); channels = 1; break;
	case MONO_16u: bytesPerSample = sizeof(unsigned short); channels = 1; break;
	case FORMAT_UNKNOWN:
	default: break;
	}
	fwrite(data, bytesPerSample, channels*xsize*ysize, f);
	return true;
}

void SaveImageToFile(const ITMUChar4Image* image, const char* fileName, bool flipVertical)
{
	// Wrap the RGBA image to be saved with a cv::Mat.
	cv::Mat imageWrapper(image->noDims.y, image->noDims.x, CV_8UC4, const_cast<ORUtils::Vector4<unsigned char>* >(image->GetData(MEMORYDEVICE_CPU)));

	// Convert the image to BGR so that it can be saved properly.
	cv::Mat outImage;
	cv::cvtColor(imageWrapper, outImage, cv::COLOR_RGBA2BGR);

	if(flipVertical)
	{
		// Flip around the x axis, converting from a top-left to a bottom-left origin.
		cv::flip(outImage, outImage, 0);
	}

	// Write the converted image to disk.
	cv::imwrite(fileName, outImage);
}

void SaveImageToFile(const ITMShortImage* image, const char* fileName)
{
	// Wrap the image to be saved with a cv::Mat.
	cv::Mat imageWrapper(image->noDims.y, image->noDims.x, CV_16SC1, const_cast<short*>(image->GetData(MEMORYDEVICE_CPU)));

	// Convert the image to an unsigned short image to allow it to be saved properly.
	cv::Mat outImage;
	imageWrapper.convertTo(outImage, CV_16U);

	// Write the converted image to disk.
	cv::imwrite(fileName, outImage);
}

void SaveImageToFile(const ITMFloatImage* image, const char* fileName)
{
	// Wrap the image to be saved with a cv::Mat.
	cv::Mat imageWrapper(image->noDims.y, image->noDims.x, CV_32FC1, const_cast<float*>(image->GetData(MEMORYDEVICE_CPU)));

	// Convert the image to an unsigned short image, scaling by 1000 and automatically saturating negative float values to 0.
	cv::Mat outImage;
	imageWrapper.convertTo(outImage, CV_16U, 1000.0);

	// Write the converted image to disk.
	cv::imwrite(fileName, outImage);
}

bool ReadImageFromFile(ITMUChar4Image* image, const char* fileName)
{
	// Read in a 24-bit BGR image.
	cv::Mat cvImage = cv::imread(fileName, cv::IMREAD_COLOR);

	// If the image wasn't read in successfully, early out.
	if(cvImage.empty() || cvImage.type() != CV_8UC3)
	{
		image->Clear();
		return false;
	}

	// Resize the output image to be the same size as the image we've just read in (this is a no-op in most cases).
	ORUtils::Vector2<int> newSize(cvImage.cols, cvImage.rows);
	image->ChangeDims(newSize);

	// Wrap the output image with a cv::Mat.
	cv::Mat imageWrapper(cvImage.size(), CV_8UC4, image->GetData(MEMORYDEVICE_CPU));

	// Copy the image we've just read in into the output image, converting from BGR to RGBA in the process.
	cv::cvtColor(cvImage, imageWrapper, cv::COLOR_BGR2RGBA);

	return true;
}

bool ReadImageFromFile(ITMShortImage *image, const char *fileName)
{
	// Read in the image. (Note that short images are read as CV_16UC1.)
	cv::Mat cvImage = cv::imread(fileName, cv::IMREAD_ANYDEPTH);

	// If the image wasn't read in successfully, early out.
	if(cvImage.empty() || cvImage.type() != CV_16UC1)
	{
		image->Clear();
		return false;
	}

	// Resize the output image to be the same size as the image we've just read in (this is a no-op in most cases).
	ORUtils::Vector2<int> newSize(cvImage.cols, cvImage.rows);
	image->ChangeDims(newSize);

	// Wrap the output image with a cv::Mat.
	cv::Mat imageWrapper(cvImage.size(), CV_16SC1, image->GetData(MEMORYDEVICE_CPU));

	// Copy the image we've just read in into the output image, converting from 16U to 16S in the process.
	cvImage.convertTo(imageWrapper, CV_16S);

	return true;
}

