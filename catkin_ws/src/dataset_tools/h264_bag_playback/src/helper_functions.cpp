#include <helper_functions.hpp>

#include <glob.h>

std::string remove_last_of_string(const std::string& filename, const std::string divider_string) {
  size_t lastdot = filename.find_last_of(divider_string);
  if (lastdot == std::string::npos) return filename;
  return filename.substr(0, lastdot);
}


std::string keep_last_of_string(const std::string& filename, const std::string divider_string) {
  size_t lastdot = filename.find_last_of(divider_string);
  if (lastdot == std::string::npos) return filename;
  return filename.substr(lastdot + 1);
}

void get_files_pattern(const std::string &pattern, std::vector<std::string> &fileList)
{
  //Declare glob_t for storing the results of globbing
  glob_t globbuf;

  //Glob.. GLOB_TILDE tells the globber to expand "~" in the pattern to the home directory
  glob( pattern.c_str(), GLOB_TILDE, NULL, &globbuf);

  for( int i = 0; i < globbuf.gl_pathc; ++i )
    fileList.push_back( globbuf.gl_pathv[i] );

  //Free the globbuf structure
  if( globbuf.gl_pathc > 0 )
    globfree( &globbuf );
}

