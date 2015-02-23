/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Centrum Wiskunde Informatica.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include<pcl/io/jpeg_io.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <stdio.h>

#include "pcl/io/io_exception.h"

using namespace pcl;
using namespace io;

// JPEGLib struct for dealing with IO messages
static struct JPEGIOErrorMgr {
struct jpeg_error_mgr pub_;	/* "public" fields */
  jmp_buf setjmp_buffer_;	/* for return to caller */
};

// pointer to the JPEG IO error pointer
typedef struct JPEGIOErrorMgr * JPEGIOErrorPtr;

// exit handler, from libjpeg turbo
METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  JPEGIOErrorPtr myerr = (JPEGIOErrorPtr) cinfo->err;
  
  (*cinfo->err->output_message) (cinfo);
  
  // Return control to the setjmp point 
  longjmp(myerr->setjmp_buffer_, 1);
}


///////////////////////// JPEG READER /////////////////////////////////////////
bool
pcl::io::JPEGReader::readJPEG(const std::vector<uint8_t> &jpeg_in_dat, PCLImage &im_out)
{
  std::string file_name;
  return readJPEG(jpeg_in_dat,file_name,false,im_out);
}

bool
pcl::io::JPEGReader::readJPEG(const std::string &jpeg_in_file, PCLImage &im_out)
{
  std::vector<uint8_t> empty_data;
  return readJPEG(empty_data,jpeg_in_file,true,im_out);
}

bool
pcl::io::JPEGReader::readJPEG(const std::vector<uint8_t> &jpeg_in_dat, const std::string &file_name, bool read_file, PCLImage &im_out)
{
  // jpeg structures
  struct jpeg_decompress_struct cinfo_;
  struct JPEGIOErrorMgr jpeg_err_;

  // set the error handlers
  cinfo_.err = jpeg_std_error(&jpeg_err_.pub_);
  jpeg_err_.pub_.error_exit = my_error_exit;

  // establish setjump to return
  if (setjmp(jpeg_err_.setjmp_buffer_)) 
  {
    // jpeg has signalled an error cleanup and return
    jpeg_destroy_decompress(&cinfo_);
	return false;
  }

  // Now we can initialize the JPEG decompression object
  jpeg_create_decompress(&cinfo_);

  // optional file stream
  FILE *in_file;

  // specify data source to jpeg lib
  if(!read_file){
    jpeg_mem_src(&cinfo_, (unsigned char *) jpeg_in_dat.data(), jpeg_in_dat.size());
  }
  else{
    in_file = fopen( file_name.c_str(),"rb");
	jpeg_stdio_src(&cinfo_, in_file);
  }

  // read the header and start decompress
  jpeg_read_header(&cinfo_, TRUE);
  jpeg_start_decompress(&cinfo_);

  // prepare to readout the image line by line with libjpeg turbo
  int row_stride = cinfo_.output_width * cinfo_.output_components;
  JSAMPARRAY buffer = (*cinfo_.mem->alloc_sarray)((j_common_ptr) &cinfo_, JPOOL_IMAGE, row_stride,1);
	 
  // resize the output image
  if(im_out.data.size() != row_stride*cinfo_.output_height )
    im_out.data.resize(row_stride*cinfo_.output_height);

  /* Here we use the library's state variable cinfo.output_scanline as the
  * loop counter, so that we don't have to keep track ourselves.
  */
  while (cinfo_.output_scanline < cinfo_.output_height) {
    // read a scanline and copy it to the output image
	(void) jpeg_read_scanlines(&cinfo_, buffer, 1);
	std::copy(buffer[0],buffer[0] + row_stride, &im_out.data[row_stride * (cinfo_.output_scanline -1) ] );
  }

  // Finish decompression
  (void) jpeg_finish_decompress(&cinfo_);
   
  // return the image dimensions, output components
  im_out.width = cinfo_.image_width;
  im_out.height =  cinfo_.image_height;
  im_out.step = cinfo_.output_components;

  // set the color space to the output image
  switch(cinfo_.out_color_space)
  {
    case(JCS_GRAYSCALE):
      im_out.encoding = "MONO";
	  break;
	case(JCS_RGB):
      im_out.encoding = "RGB";
	  break;
	case(JCS_YCbCr):
	  im_out.encoding = "YUV";
	  break;
	default:
	  im_out.encoding = "unknown";
    break;
  }
  // done
  return true;
}
////////////////////////////// END JPEG READER //////////////////////////


////////////////////////////// JPEG WRITER /////////////////////////////
bool
pcl::io::JPEGWriter::writeJPEG(const PCLImage &im_in, std::vector<uint8_t> &cdat, int quality)
{
	std::string fname;
	return writeJPEG(im_in, cdat, fname, quality, false);
}

bool
pcl::io::JPEGWriter::writeJPEG(const PCLImage &im_in, const std::string &file_name, int quality)
{
	std::vector<uint8_t> em_dat;
	return writeJPEG(im_in, em_dat, file_name, quality, true);
}

bool
pcl::io::JPEGWriter::writeJPEG(const PCLImage &im_in, std::vector<uint8_t> &cdat, const std::string &file_name, int quality, bool write_file)
{
  // structures for jpeg compression
  struct jpeg_compress_struct cinfo_;
  struct JPEGIOErrorMgr jpeg_err_;

  // set the error handlers
  cinfo_.err = jpeg_std_error(&jpeg_err_.pub_);
  jpeg_err_.pub_.error_exit = my_error_exit;

  // data structures for writing the jpeg image (can be declared in the encoder routine)
  JSAMPROW row_pointer_[1];
  int row_stride_;

  // return if no data in the image
  if(!im_in.data.size())
    return false;
  if(!im_in.width)
	  return false;
  if(!im_in.height)
	  return false;
  
  // structures to store the resulting compressed data
  unsigned long out_data_size=0;
  unsigned char * out_buffer;

  FILE *l_o_file = std::fopen(file_name.c_str(),"wb");

  /* Now we can initialize the JPEG compression object. */
  jpeg_create_compress(&cinfo_);
  
  // either write the output file or write to a membuffer
  if(!write_file)
  {
    jpeg_mem_dest(&cinfo_, &out_buffer ,  &out_data_size);
  }
  else
  {
	if(l_o_file)
	{
	  jpeg_stdio_dest(&cinfo_, l_o_file);
	}
	else
	{
	  return false;
	}
  }
  // supply the image height and width to the codec
  cinfo_.image_width = im_in.width; 	/* image width and height, in pixels */
  cinfo_.image_height = im_in.height;

  // we only support YUV RGB as common in PCL
  if(im_in.encoding.compare("YUV") == 0){
    cinfo_.in_color_space = JCS_RGB;
	cinfo_.input_components = 3;
  }
  else if(im_in.encoding.compare("RGB") == 0){
    cinfo_.in_color_space = JCS_RGB;
	cinfo_.input_components = 3;
  }
  else if(im_in.encoding.compare("MONO") == 0){
    cinfo_.in_color_space = JCS_GRAYSCALE;
	cinfo_.input_components = 1;
  }
  else if(im_in.encoding.compare("GRAY") == 0){
    cinfo_.in_color_space = JCS_GRAYSCALE;
	cinfo_.input_components = 1;
  }
  else{
    cinfo_.in_color_space = JCS_RGB;
	cinfo_.input_components = 3;
  }

  // jpeg settings
  jpeg_set_defaults(&cinfo_); // 
  jpeg_set_quality(&cinfo_, quality, TRUE /* limit to baseline-JPEG values */);
  jpeg_start_compress(&cinfo_, TRUE); //
  row_stride_ = im_in.width * cinfo_.input_components;	// JSAMPLEs per row in image_buffer 

  // scan lines and compress them
  while (cinfo_.next_scanline < cinfo_.image_height) {
    row_pointer_[0] = (unsigned char *) & im_in.data[(cinfo_.next_scanline) * row_stride_];
    (void) jpeg_write_scanlines(&cinfo_, row_pointer_, 1);
  }
  // finished
  jpeg_finish_compress(&cinfo_);
  
  // resize the output to match the actual datasize
  if(!write_file){
    cdat.resize(out_data_size);
    std::copy( (uint8_t *) out_buffer, (uint8_t *) (out_buffer +out_data_size) , cdat.data());
  }
  jpeg_destroy_compress(&cinfo_);
  
  if(write_file)
    fclose(l_o_file);

  return true;
}
////////////////////////////// END JPEG WRITER /////////////////////////////