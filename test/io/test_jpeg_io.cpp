#include <gtest/gtest.h>
#include <pcl/common/io.h>
#include <pcl/PCLImage.h>
#include <pcl/io/jpeg_io.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>

using namespace std;

string jpeg_dir_;

vector<pcl::PCLImage> ims_;
vector<std::string> jpeg_files_;

TEST (PCL, JPEGRead)
{
  auto im_it = ims_.begin();
  std::cout << " number of jpeg files " << jpeg_files_.size() << std::endl;

  for(auto it = jpeg_files_.begin(); it != jpeg_files_.end(); ++it, ++im_it)
  {
	// load the image file
	pcl::io::JPEGReader::readJPEG(*it, *im_it);
	
	// write to a compressed buffer
	std::vector<uint8_t> cdat;
	pcl::io::JPEGWriter::writeJPEG(*im_it,cdat);

	// read from the compressed buffer
	pcl::PCLImage im_out;
	pcl::io::JPEGReader::readJPEG(cdat, im_out);

	// check the image width and height
	EXPECT_EQ(im_out.width,im_it->width) << " in and output not equal in width ";
	EXPECT_EQ(im_out.height,im_it->height) << " in and output not equal in width ";
	  
	// write the output decoded file to the folder with jpeg files
	std::string out_name = boost::filesystem::path(*it).parent_path().string() +"\\decoded_" + boost::filesystem::path(*it).filename().string(); 
	pcl::io::JPEGWriter::writeJPEG(im_out, out_name );
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test files were given. Please add the path to jpeg_sequences to this test. (see pcl\test\jpeg_sequences) " << std::endl;
    return (-1);
  }

  std::string jpeg_sequences = argv[1];
  jpeg_dir_ = jpeg_sequences;
  
  // Get jpeg files
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (jpeg_dir_); itr != end_itr; ++itr)
  {
#if BOOST_FILESYSTEM_VERSION == 3
    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".JPG" )
#else
    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->leaf ())) == ".JPG" )
#endif
    {
#if BOOST_FILESYSTEM_VERSION == 3
      jpeg_files_.push_back (itr->path ().string ());
      std::cout << "added: " << itr->path ().string () << std::endl;
#else
      jpeg_files_.push_back (itr->path().string ());
      std::cout << "added: " << itr->path() << std::endl;
#endif
    }
  }
  sort (jpeg_files_.begin (), jpeg_files_.end ());
  // And load them
  for (size_t i = 0; i < jpeg_files_.size (); i++)
  {
    pcl::PCLImage im;
    ims_.push_back (im);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
