#ifndef H264_BAG_TOOLS_HELPER_FUNCTIONS_HPP
#define H264_BAG_TOOLS_HELPER_FUNCTIONS_HPP

#include <string>
#include <vector>

std::string remove_last_of_string(const std::string& filename, const std::string divider_string);

std::string keep_last_of_string(const std::string& filename, const std::string divider_string);

void get_files_pattern(const std::string &pattern, std::vector<std::string> &fileList);


#endif //H264_BAG_TOOLS_HELPER_FUNCTIONS_HPP
